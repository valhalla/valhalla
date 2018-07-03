#include "baldr/datetime.h"
#include "midgard/logging.h"
#include "thor/timedep.h"
#include <algorithm>
#include <iostream> // TODO remove if not needed
#include <map>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

// TODO - compute initial label count based on estimated route length
constexpr uint64_t kInitialEdgeLabelCount = 500000;

// Default constructor
TimeDepReverse::TimeDepReverse() : AStarPathAlgorithm() {
  mode_ = TravelMode::kDrive;
  travel_type_ = 0;
  adjacencylist_ = nullptr;
  max_label_count_ = std::numeric_limits<uint32_t>::max();
  dest_tz_index_ = 0;
  access_mode_ = kAutoAccess;
}

// Destructor
TimeDepReverse::~TimeDepReverse() {
  Clear();
}

// Get the timezone at the destination.
int TimeDepReverse::GetDestinationTimezone(GraphReader& graphreader) {
  if (edgelabels_rev_.size() == 0) {
    return -1;
  }
  GraphId node = edgelabels_rev_[0].endnode();
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return -1;
  }
  return tile->node(node)->timezone();
}

// Initialize prior to finding best path
void TimeDepReverse::Init(const PointLL& origll, const PointLL& destll) {
  // Set the origin lat,lon (since this is reverse path) and cost factor
  // in the A* heuristic
  astarheuristic_.Init(origll, costing_->AStarCostFactor());

  // Get the initial cost based on A* heuristic from destination
  float mincost = astarheuristic_.Get(destll);

  // Reserve size for edge labels - do this here rather than in constructor so
  // to limit how much extra memory is used for persistent objects.
  // TODO - reserve based on estimate based on distance and route type.
  edgelabels_rev_.reserve(kInitialEdgeLabelCount);

  // Set up lambda to get sort costs
  const auto edgecost = [this](const uint32_t label) { return edgelabels_rev_[label].sortcost(); };

  // Construct adjacency list, clear edge status.
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing_->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_.reset(new DoubleBucketQueue(mincost, range, bucketsize, edgecost));
  edgestatus_.clear();

  // Get hierarchy limits from the costing. Get a copy since we increment
  // transition counts (i.e., this is not a const reference).
  hierarchy_limits_ = costing_->GetHierarchyLimits();
}

// Expand from the node along the forward search path. Immediately expands
// from the end node of any transition edge (so no transition edges are added
// to the adjacency list or BDEdgeLabel list). Does not expand transition
// edges if from_transition is false.
void TimeDepReverse::ExpandReverse(GraphReader& graphreader,
                                   const GraphId& node,
                                   const BDEdgeLabel& pred,
                                   const uint32_t pred_idx,
                                   const DirectedEdge* opp_pred_edge,
                                   const bool from_transition,
                                   uint64_t localtime,
                                   const odin::Location& destination,
                                   std::pair<int32_t, float>& best_path) {
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }
  const NodeInfo* nodeinfo = tile->node(node);
  if (!costing_->Allowed(nodeinfo)) {
    return;
  }

  // Adjust for time zone (if different from timezone at the destination).
  if (nodeinfo->timezone() != dest_tz_index_) {
    DateTime::timezone_diff(false, localtime, DateTime::get_tz_db().from_index(nodeinfo->timezone()),
                            DateTime::get_tz_db().from_index(dest_tz_index_));
  }

  // Expand from end node.
  uint32_t max_shortcut_length = static_cast<uint32_t>(pred.distance() * 0.5f);
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  EdgeStatusInfo* es = edgestatus_.GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++directededge, ++edgeid, ++es) {
    // Handle transition edges - expand from the end node of the transition
    // (unless this is called from a transition).
    if (directededge->trans_up()) {
      if (!from_transition) {
        hierarchy_limits_[node.level()].up_transition_count++;
        ExpandReverse(graphreader, directededge->endnode(), pred, pred_idx, opp_pred_edge, true,
                      localtime, destination, best_path);
      }
      continue;
    } else if (directededge->trans_down()) {
      if (!from_transition &&
          !hierarchy_limits_[directededge->endnode().level()].StopExpanding(pred.distance())) {
        ExpandReverse(graphreader, directededge->endnode(), pred, pred_idx, opp_pred_edge, true,
                      localtime, destination, best_path);
      }
      continue;
    }

    // Skip shortcut edges for time dependent routes
    if (directededge->is_shortcut()) {
      continue;
    }

    // Skip this edge if permanently labeled (best path already found to this
    // directed edge) or if no access for this mode.
    if (es->set() == EdgeSet::kPermanent || !(directededge->reverseaccess() & access_mode_)) {
      continue;
    }

    // Get end node tile, opposing edge Id, and opposing directed edge.
    const GraphTile* t2 =
        directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
    if (t2 == nullptr) {
      continue;
    }
    GraphId oppedge = t2->GetOpposingEdgeId(directededge);
    const DirectedEdge* opp_edge = t2->directededge(oppedge);

    // Skip this edge if no access is allowed (based on costing method)
    // or if a complex restriction prevents transition onto this edge.
    if (!costing_->AllowedReverse(directededge, pred, opp_edge, t2, oppedge, localtime,
                                  nodeinfo->timezone()) ||
        costing_->Restricted(directededge, pred, edgelabels_rev_, tile, edgeid, false, localtime,
                             nodeinfo->timezone())) {
      continue;
    }

    Cost tc = costing_->TransitionCostReverse(directededge->localedgeidx(), nodeinfo, opp_edge,
                                              opp_pred_edge);
    Cost newcost = pred.cost() + costing_->EdgeCost(opp_edge, t2->GetSpeed(opp_edge, localtime,
                                                                           nodeinfo->timezone()));
    newcost.cost += tc.cost;

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge).
    auto p = destinations_.find(edgeid);
    if (p != destinations_.end()) {
      // Subtract partial cost and time
      newcost -= p->second;

      // Find the destination edge and update cost to include the edge score.
      // Note - with high edge scores the convergence test fails some routes
      // so reduce the edge score.
      for (const auto& destination_edge : destination.path_edges()) {
        if (destination_edge.graph_id() == edgeid) {
          newcost.cost += destination_edge.distance();
        }
      }
      newcost.cost = std::max(0.0f, newcost.cost);

      // Mark this as the best connection if that applies. This allows
      // a path to be formed even if the convergence test fails (can
      // happen with large edge scores)
      if (best_path.first == -1 || newcost.cost < best_path.second) {
        best_path.first = (es->set() == EdgeSet::kTemporary) ? es->index() : edgelabels_rev_.size();
        best_path.second = newcost.cost;
      }
    }

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (es->set() == EdgeSet::kTemporary) {
      BDEdgeLabel& lab = edgelabels_rev_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_->decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, tc);
      }
      continue;
    }

    // If this is a destination edge the A* heuristic is 0. Otherwise the
    // sort cost (with A* heuristic) is found using the lat,lng at the
    // end node of the directed edge.
    float dist = 0.0f;
    float sortcost = newcost.cost;
    if (p == destinations_.end()) {
      const GraphTile* t2 =
          directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (t2 == nullptr) {
        continue;
      }
      sortcost += astarheuristic_.Get(t2->node(directededge->endnode())->latlng(), dist);
    }

    // Add edge label, add to the adjacency list and set edge status
    uint32_t idx = edgelabels_rev_.size();
    edgelabels_rev_.emplace_back(pred_idx, edgeid, oppedge, directededge, newcost, sortcost, dist,
                                 mode_, tc, (pred.not_thru_pruning() || !directededge->not_thru()));
    adjacencylist_->add(idx);
    *es = {EdgeSet::kTemporary, idx};
  }
}

// Calculate time-dependent best path using a reverse search. Supports
// "arrive-by" routes.
std::vector<PathInfo> TimeDepReverse::GetBestPath(odin::Location& origin,
                                                  odin::Location& destination,
                                                  GraphReader& graphreader,
                                                  const std::shared_ptr<DynamicCost>* mode_costing,
                                                  const TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();
  access_mode_ = costing_->access_mode();

  // date_time must be set on the destination.
  if (!destination.has_date_time()) {
    LOG_ERROR("TimeDepReverse called without time set on the destination location");
    return {};
  }

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  // Note: because we can correlate to more than one place for a given PathLocation
  // using edges.front here means we are only setting the heuristics to one of them
  // alternate paths using the other correlated points to may be harder to find
  PointLL origin_new(origin.path_edges(0).ll().lng(), origin.path_edges(0).ll().lat());
  PointLL destination_new(destination.path_edges(0).ll().lng(), destination.path_edges(0).ll().lat());
  Init(origin_new, destination_new);
  float mindist = astarheuristic_.GetDistance(origin_new);

  // Initialize the locations. For a reverse path search the destination location
  // is used as the "origin" and the origin location is used as the "destination".
  uint32_t density = SetDestination(graphreader, origin);
  SetOrigin(graphreader, destination, origin);

  // Set the destination timezone
  dest_tz_index_ = GetDestinationTimezone(graphreader);

  // Update hierarchy limits
  ModifyHierarchyLimits(mindist, density);

  // Set route start time (seconds from epoch)
  uint64_t start_time =
      DateTime::seconds_since_epoch(destination.date_time(),
                                    DateTime::get_tz_db().from_index(dest_tz_index_));
  // Find shortest path
  uint32_t nc = 0; // Count of iterations with no convergence
                   // towards destination
  std::pair<int32_t, float> best_path = std::make_pair(-1, 0.0f);
  const GraphTile* tile;
  size_t total_labels = 0;
  while (true) {
    // Allow this process to be aborted
    size_t current_labels = edgelabels_rev_.size();
    if (interrupt &&
        total_labels / kInterruptIterationsInterval < current_labels / kInterruptIterationsInterval) {
      (*interrupt)();
    }
    total_labels = current_labels;

    // Abort if max label count is exceeded
    if (total_labels > max_label_count_) {
      return {};
    }

    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->pop();
    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " + std::to_string(edgelabels_rev_.size()));
      return {};
    }

    // Copy the BDEdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    BDEdgeLabel pred = edgelabels_rev_[predindex];
    if (destinations_.find(pred.edgeid()) != destinations_.end()) {
      // Check if a trivial path. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        if (IsTrivial(pred.edgeid(), origin, destination)) {
          return FormPath(graphreader, predindex);
        }
      } else {
        return FormPath(graphreader, predindex);
      }
    }

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge (this will allow loops/around the block cases)
    if (!pred.origin()) {
      edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Check that distance is converging towards the destination. Return route
    // failure if no convergence for TODO iterations
    float dist2dest = pred.distance();
    if (dist2dest < mindist) {
      mindist = dist2dest;
      nc = 0;
    } else if (nc++ > 50000) {
      if (best_path.first >= 0) {
        return FormPath(graphreader, best_path.first);
      } else {
        LOG_ERROR("No convergence to destination after = " + std::to_string(edgelabels_rev_.size()));
        return {};
      }
    }

    // Do not expand based on hierarchy level based on number of upward
    // transitions and distance to the destination
    if (hierarchy_limits_[pred.endnode().level()].StopExpanding(dist2dest)) {
      continue;
    }

    // Set local time (subtract elapsed time along the path from the start
    // time). TODO: adjust for time zone if different than starting tz
    uint64_t localtime = start_time - (double)pred.cost().secs;

    // Get the opposing predecessor directed edge. Need to make sure we get
    // the correct one if a transition occurred
    const DirectedEdge* opp_pred_edge =
        graphreader.GetGraphTile(pred.opp_edgeid())->directededge(pred.opp_edgeid());

    // Expand forward from the end node of the predecessor edge.
    ExpandReverse(graphreader, pred.endnode(), pred, predindex, opp_pred_edge, false, localtime,
                  destination, best_path);
  }
  return {}; // Should never get here
}

// The origin of the reverse path is the destination location.
// TODO - how do we set the
void TimeDepReverse::SetOrigin(GraphReader& graphreader,
                               odin::Location& origin,
                               odin::Location& destination) {
  // Only skip outbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.path_edges().begin(), origin.path_edges().end(),
                [&has_other_edges](const odin::Location::PathEdge& e) {
                  has_other_edges = has_other_edges || !e.begin_node();
                });

  // Check if the origin edge matches a destination edge at the node.
  auto trivial_at_node = [this, &destination](const odin::Location::PathEdge& edge) {
    auto p = destinations_.find(edge.graph_id());
    if (p != destinations_.end()) {
      for (const auto& destination_edge : destination.path_edges()) {
        if (destination_edge.graph_id() == edge.graph_id()) {
          return true;
        }
      }
    }
    return false;
  };

  // Iterate through edges and add to adjacency list
  Cost c;
  const NodeInfo* nodeinfo = nullptr;
  const NodeInfo* closest_ni = nullptr;
  for (const auto& edge : origin.path_edges()) {
    // If the origin (real destination) is at a node, skip any outbound
    // edges (so any opposing inbound edges are not considered) unless the
    // destination is also at the same end node (trivial path).
    if (has_other_edges && edge.begin_node() && !trivial_at_node(edge)) {
      continue;
    }

    // Get the directed edge
    GraphId edgeid(edge.graph_id());
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the opposing directed edge, continue if we cannot get it
    GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid);
    if (!opp_edge_id.Is_Valid()) {
      continue;
    }
    const DirectedEdge* opp_dir_edge = graphreader.GetOpposingEdge(edgeid);

    // Get cost
    Cost cost = costing_->EdgeCost(directededge, tile->GetSpeed(directededge)) * edge.percent_along();
    float dist = astarheuristic_.GetDistance(tile->node(opp_dir_edge->endnode())->latlng());

    // We need to penalize this location based on its score (distance in meters from input)
    // We assume the slowest speed you could travel to cover that distance to start/end the route
    // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
    // Perhaps need to adjust score?
    cost.cost += edge.distance();

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge) if the
    // destination is in a forward direction along the edge. Add back in
    // the edge score/penalty to account for destination edges farther from
    // the input location lat,lon.
    auto p = destinations_.find(edgeid);
    if (p != destinations_.end()) {
      if (IsTrivial(edgeid, origin, destination)) {
        // Find the destination edge and update cost.
        for (const auto& destination_edge : destination.path_edges()) {
          if (destination_edge.graph_id() == edgeid) {
            // a trivial route passes along a single edge, meaning that the
            // destination point must be on this edge, and so the distance
            // remaining must be zero.
            GraphId id(destination_edge.graph_id());
            const DirectedEdge* dest_diredge = tile->directededge(id);
            Cost dest_cost = costing_->EdgeCost(dest_diredge, tile->GetSpeed(dest_diredge)) *
                             (1.0f - destination_edge.percent_along());
            cost.secs -= p->second.secs;
            cost.cost -= dest_cost.cost;
            cost.cost += destination_edge.distance();
            cost.cost = std::max(0.0f, cost.cost);
            dist = 0.0;
          }
        }
      }

      // Store the closest node info
      if (closest_ni == nullptr) {
        closest_ni = nodeinfo;
      }
    }

    // Compute sortcost
    float sortcost = cost.cost + astarheuristic_.Get(dist);

    // Add BDEdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path. Make sure the opposing
    // edge (edgeid) is set.
    // DO NOT SET EdgeStatus - it messes up trivial paths with oneways
    uint32_t idx = edgelabels_rev_.size();
    edgelabels_rev_.emplace_back(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, sortcost,
                                 dist, mode_, c, false);
    adjacencylist_->add(idx);

    // Set the initial not_thru flag to false. There is an issue with not_thru
    // flags on small loops. Set this to false here to override this for now.
    edgelabels_rev_.back().set_not_thru(false);

    // Set the origin flag
    edgelabels_rev_.back().set_origin();
  }
}

// Add destination edges at the origin location. If the location is at a node
// skip any outbound edges since the path search is reversed.
// TODO - test to make sure that excluding outbound edges is what we want!
uint32_t TimeDepReverse::SetDestination(GraphReader& graphreader, const odin::Location& dest) {
  // Only skip outbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(dest.path_edges().begin(), dest.path_edges().end(),
                [&has_other_edges](const odin::Location::PathEdge& e) {
                  has_other_edges = has_other_edges || !e.begin_node();
                });

  // For each edge
  uint32_t density = 0;
  for (const auto& edge : dest.path_edges()) {
    // If destination is at a node skip any outbound edges
    if (has_other_edges && edge.begin_node()) {
      continue;
    }

    // Keep the id and the cost to traverse the partial distance for the
    // remainder of the edge. This cost is subtracted from the total cost
    // up to the end of the destination edge.
    GraphId id(edge.graph_id());
    const GraphTile* tile = graphreader.GetGraphTile(id);
    const DirectedEdge* directededge = tile->directededge(id);

    // The opposing edge Id is added as a destination since the search
    // is done in reverse direction.
    const GraphTile* t2 =
        directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
    if (t2 == nullptr) {
      continue;
    }
    GraphId oppedge = t2->GetOpposingEdgeId(directededge);
    destinations_[oppedge] = costing_->EdgeCost(directededge, tile->GetSpeed(directededge)) *
                             (1.0f - edge.percent_along());

    // Edge score (penalty) is handled within GetPath. Do not add score here.

    // Get the tile relative density
    density = tile->header()->density();
  }
  return density;
}

// Form the path from the adjacency list.
std::vector<PathInfo> TimeDepReverse::FormPath(GraphReader& graphreader, const uint32_t dest) {
  // Metrics to track
  LOG_DEBUG("path_cost::" + std::to_string(edgelabels_rev_[dest].cost().cost));
  LOG_DEBUG("path_iterations::" + std::to_string(edgelabels_rev_.size()));

  // Get the transition cost at the last edge of the reverse path
  float tc = edgelabels_rev_[dest].transition_secs();

  // Form the reverse path from the destination (true origin) using opposing
  // edges.
  float secs = 0.0f;
  std::vector<PathInfo> path;
  uint32_t edgelabel_index = dest;
  while (edgelabel_index != kInvalidLabel) {
    const BDEdgeLabel& edgelabel = edgelabels_rev_[edgelabel_index];

    // Get elapsed time on the edge, then add the transition cost at
    // prior edge.
    uint32_t predidx = edgelabel.predecessor();
    if (predidx == kInvalidLabel) {
      secs += edgelabel.cost().secs;
    } else {
      secs += edgelabel.cost().secs - edgelabels_rev_[predidx].cost().secs;
    }
    secs += tc;
    path.emplace_back(edgelabel.mode(), secs, edgelabel.opp_edgeid(), 0);

    // Check if this is a ferry
    if (edgelabel.use() == Use::kFerry) {
      has_ferry_ = true;
    }

    // Update edgelabel_index and transition cost to apply at next iteration
    edgelabel_index = predidx;
    tc = edgelabel.transition_secs();
  }
  return path;
}

} // namespace thor
} // namespace valhalla
