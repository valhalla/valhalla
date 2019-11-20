#include "baldr/datetime.h"
#include "midgard/constants.h"
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
  seconds_of_week_ = 0;
  access_mode_ = kAutoAccess;
}

// Destructor
TimeDepReverse::~TimeDepReverse() {
  Clear();
}

// Initialize prior to finding best path
void TimeDepReverse::Init(const midgard::PointLL& origll, const midgard::PointLL& destll) {
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
bool TimeDepReverse::ExpandReverse(GraphReader& graphreader,
                                   const GraphId& node,
                                   BDEdgeLabel& pred,
                                   const uint32_t pred_idx,
                                   const DirectedEdge* opp_pred_edge,
                                   const bool from_transition,
                                   uint64_t localtime,
                                   int32_t seconds_of_week,
                                   const valhalla::Location& destination,
                                   std::pair<int32_t, float>& best_path) {
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return false;
  }
  const NodeInfo* nodeinfo = tile->node(node);
  if (!costing_->Allowed(nodeinfo)) {
    return false;
  }

  // Adjust for time zone (if different from timezone at the destination).
  if (nodeinfo->timezone() != dest_tz_index_) {
    // Get the difference in seconds between the destination tz and current tz
    int tz_diff =
        DateTime::timezone_diff(localtime, DateTime::get_tz_db().from_index(nodeinfo->timezone()),
                                DateTime::get_tz_db().from_index(dest_tz_index_));
    localtime += tz_diff;
    seconds_of_week = DateTime::normalize_seconds_of_week(seconds_of_week + tz_diff);
  }

  // Expand from end node.
  EdgeMetadata meta = EdgeMetadata::make(node, nodeinfo, tile, edgestatus_);

  bool found_valid_edge = false;
  bool found_uturn = false;
  EdgeMetadata uturn_meta = {};

  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, meta.increment_pointers()) {
    // Begin by checking if this is the opposing edge to pred.
    // If so, it means we are attempting a u-turn. In that case, lets wait with evaluating
    // this edge until last. If any other edges were emplaced, it means we should not
    // even try to evaluate a u-turn since u-turns should only happen for deadends
    if (pred.opp_local_idx() == meta.edge->localedgeidx()) {
      uturn_meta = meta;
      found_uturn = true;
      continue;
    }

    found_valid_edge = ExpandReverseInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx, meta,
                                          tile, localtime, seconds_of_week, destination, best_path) ||
                       found_valid_edge;
  }

  // Handle transitions - expand from the end node of each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      if (trans->up()) {
        hierarchy_limits_[node.level()].up_transition_count++;
        found_valid_edge = ExpandReverse(graphreader, trans->endnode(), pred, pred_idx, opp_pred_edge,
                                         true, localtime, seconds_of_week, destination, best_path) ||
                           found_valid_edge;
      } else if (!hierarchy_limits_[trans->endnode().level()].StopExpanding(pred.distance())) {
        found_valid_edge = ExpandReverse(graphreader, trans->endnode(), pred, pred_idx, opp_pred_edge,
                                         true, localtime, seconds_of_week, destination, best_path) ||
                           found_valid_edge;
      }
    }
  }

  if (!from_transition) {
    // Now, after having looked at all the edges, including edges on other levels,
    // we can say if this is a deadend or not, and if so, evaluate the uturn-edge (if it exists)
    if (!found_valid_edge && found_uturn) {
      // If we found no suitable edge to add, it means we're at a deadend
      // so lets go back and re-evaluate a potential u-turn

      pred.set_deadend(true);

      // Decide if we should expand a shortcut or the non-shortcut edge...
      bool was_uturn_shortcut_added = false;

      // TODO Is there a shortcut that supersedes our u-turn?
      if (was_uturn_shortcut_added) {
        found_valid_edge = true;
      } else {
        // We didn't add any shortcut of the uturn, therefore evaluate the regular uturn instead
        found_valid_edge =
            ExpandReverseInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx, uturn_meta, tile,
                               localtime, seconds_of_week, destination, best_path) ||
            found_valid_edge;
      }
    }
  }
  return found_valid_edge;
}

// Runs in the inner loop of `ExpandForward`, essentially evaluating if
// the edge described in `meta` should be placed on the stack
// as well as doing just that.
//
// Returns true if any edge _could_ have been expanded after restrictions etc.
inline bool TimeDepReverse::ExpandReverseInner(GraphReader& graphreader,
                                               const BDEdgeLabel& pred,
                                               const baldr::DirectedEdge* opp_pred_edge,
                                               const NodeInfo* nodeinfo,
                                               const uint32_t pred_idx,
                                               const EdgeMetadata& meta,
                                               const GraphTile* tile,
                                               uint64_t localtime,
                                               uint32_t seconds_of_week,
                                               const valhalla::Location& destination,
                                               std::pair<int32_t, float>& best_path) {

  // Skip shortcut edges for time dependent routes. Also skip this edge if permanently labeled (best
  // path already found to this directed edge) or if no access for this mode.
  if (meta.edge->is_shortcut() || meta.edge_status->set() == EdgeSet::kPermanent ||
      !(meta.edge->reverseaccess() & access_mode_)) {
    return false;
  }

  // Get end node tile, opposing edge Id, and opposing directed edge.
  const GraphTile* t2 =
      meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
  if (t2 == nullptr) {
    return false;
  }
  GraphId oppedge = t2->GetOpposingEdgeId(meta.edge);
  const DirectedEdge* opp_edge = t2->directededge(oppedge);

  // Skip this edge if no access is allowed (based on costing method)
  // or if a complex restriction prevents transition onto this edge.
  bool has_time_restrictions = false;
  if (!costing_->AllowedReverse(meta.edge, pred, opp_edge, t2, oppedge, localtime,
                                nodeinfo->timezone(), has_time_restrictions) ||
      costing_->Restricted(meta.edge, pred, edgelabels_rev_, tile, meta.edge_id, false, localtime,
                           nodeinfo->timezone())) {
    return false;
  }

  Cost tc =
      costing_->TransitionCostReverse(meta.edge->localedgeidx(), nodeinfo, opp_edge, opp_pred_edge);
  Cost newcost = pred.cost() + costing_->EdgeCost(opp_edge, t2, seconds_of_week);
  newcost.cost += tc.cost;

  // If this edge is a destination, subtract the partial/remainder cost
  // (cost from the dest. location to the end of the edge).
  auto p = destinations_.find(meta.edge_id);
  if (p != destinations_.end()) {
    // Subtract partial cost and time
    newcost -= p->second;

    // Find the destination edge and update cost to include the edge score.
    // Note - with high edge scores the convergence test fails some routes
    // so reduce the edge score.
    for (const auto& destination_edge : destination.path_edges()) {
      if (destination_edge.graph_id() == meta.edge_id) {
        newcost.cost += destination_edge.distance();
      }
    }
    newcost.cost = std::max(0.0f, newcost.cost);

    // Mark this as the best connection if that applies. This allows
    // a path to be formed even if the convergence test fails (can
    // happen with large edge scores)
    if (best_path.first == -1 || newcost.cost < best_path.second) {
      best_path.first = (meta.edge_status->set() == EdgeSet::kTemporary) ? meta.edge_status->index()
                                                                         : edgelabels_rev_.size();
      best_path.second = newcost.cost;
    }
  }

  // Check if edge is temporarily labeled and this path has less cost. If
  // less cost the predecessor is updated and the sort cost is decremented
  // by the difference in real cost (A* heuristic doesn't change)
  if (meta.edge_status->set() == EdgeSet::kTemporary) {
    BDEdgeLabel& lab = edgelabels_rev_[meta.edge_status->index()];
    if (newcost.cost < lab.cost().cost) {
      float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
      adjacencylist_->decrease(meta.edge_status->index(), newsortcost);
      lab.Update(pred_idx, newcost, newsortcost, tc, has_time_restrictions);
    }
    return true;
  }

  // If this is a destination edge the A* heuristic is 0. Otherwise the
  // sort cost (with A* heuristic) is found using the lat,lng at the
  // end node of the directed edge.
  float dist = 0.0f;
  float sortcost = newcost.cost;
  if (p == destinations_.end()) {
    const GraphTile* t2 =
        meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
    if (t2 == nullptr) {
      return false;
    }
    sortcost += astarheuristic_.Get(t2->get_node_ll(meta.edge->endnode()), dist);
  }

  // Add edge label, add to the adjacency list and set edge status
  uint32_t idx = edgelabels_rev_.size();
  edgelabels_rev_.emplace_back(pred_idx, meta.edge_id, oppedge, meta.edge, newcost, sortcost, dist,
                               mode_, tc, (pred.not_thru_pruning() || !meta.edge->not_thru()),
                               has_time_restrictions);
  adjacencylist_->add(idx);
  *meta.edge_status = {EdgeSet::kTemporary, idx};

  return true;
}

// Calculate time-dependent best path using a reverse search. Supports
// "arrive-by" routes.
std::vector<std::vector<PathInfo>>
TimeDepReverse::GetBestPath(valhalla::Location& origin,
                            valhalla::Location& destination,
                            GraphReader& graphreader,
                            const std::shared_ptr<DynamicCost>* mode_costing,
                            const TravelMode mode,
                            const Options& options) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();
  access_mode_ = costing_->access_mode();

  // date_time must be set on the destination. Log an error but allow routes for now.
  if (!destination.has_date_time()) {
    LOG_ERROR("TimeDepReverse called without time set on the destination location");
    // return {};
  }

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  // Note: because we can correlate to more than one place for a given PathLocation
  // using edges.front here means we are only setting the heuristics to one of them
  // alternate paths using the other correlated points to may be harder to find
  midgard::PointLL origin_new(origin.path_edges(0).ll().lng(), origin.path_edges(0).ll().lat());
  midgard::PointLL destination_new(destination.path_edges(0).ll().lng(),
                                   destination.path_edges(0).ll().lat());
  Init(origin_new, destination_new);
  float mindist = astarheuristic_.GetDistance(origin_new);

  // Initialize the locations. For a reverse path search the destination location
  // is used as the "origin" and the origin location is used as the "destination".
  uint32_t density = SetDestination(graphreader, origin);
  SetOrigin(graphreader, destination, origin);

  // Set the destination timezone
  dest_tz_index_ =
      edgelabels_rev_.size() == 0 ? 0 : GetTimezone(graphreader, edgelabels_rev_[0].endnode());
  if (dest_tz_index_ == 0) {
    // TODO - do not throw exception at this time
    LOG_ERROR("Could not get the timezone at the destination");
  }

  // Update hierarchy limits
  ModifyHierarchyLimits(mindist, density);

  // Set route start time (seconds from epoch)
  uint64_t start_time =
      DateTime::seconds_since_epoch(destination.date_time(),
                                    DateTime::get_tz_db().from_index(dest_tz_index_));

  // Set seconds from beginning of the week
  seconds_of_week_ = DateTime::day_of_week(destination.date_time()) * midgard::kSecondsPerDay +
                     DateTime::seconds_from_midnight(destination.date_time());

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
      // Check if a trivial path using opposing edge. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        // Use opposing edge.
        if (IsTrivial(pred.opp_edgeid(), origin, destination)) {
          return {FormPath(graphreader, predindex)};
        }
      } else {
        return {FormPath(graphreader, predindex)};
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
        return {FormPath(graphreader, best_path.first)};
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

    // Set local time and seconds of the week.
    uint32_t secs = static_cast<uint32_t>(pred.cost().secs);
    uint64_t localtime = start_time - secs;
    int32_t seconds_of_week = DateTime::normalize_seconds_of_week(seconds_of_week_ - secs);

    // Get the opposing predecessor directed edge. Need to make sure we get
    // the correct one if a transition occurred
    const DirectedEdge* opp_pred_edge =
        graphreader.GetGraphTile(pred.opp_edgeid())->directededge(pred.opp_edgeid());

    // Expand forward from the end node of the predecessor edge.
    ExpandReverse(graphreader, pred.endnode(), pred, predindex, opp_pred_edge, false, localtime,
                  seconds_of_week, destination, best_path);
  }
  return {}; // Should never get here
}

// The origin of the reverse path is the destination location.
// TODO - how do we set the
void TimeDepReverse::SetOrigin(GraphReader& graphreader,
                               valhalla::Location& origin,
                               valhalla::Location& destination) {
  // Only skip outbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.path_edges().begin(), origin.path_edges().end(),
                [&has_other_edges](const valhalla::Location::PathEdge& e) {
                  has_other_edges = has_other_edges || !e.begin_node();
                });

  // Check if the origin edge matches a destination edge at the node.
  auto trivial_at_node = [this, &destination](const valhalla::Location::PathEdge& edge) {
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
    Cost cost = costing_->EdgeCost(directededge, tile) * edge.percent_along();
    float dist = astarheuristic_.GetDistance(tile->get_node_ll(opp_dir_edge->endnode()));

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
    auto p = destinations_.find(opp_edge_id);
    if (p != destinations_.end()) {
      // Reverse the origin and destination in the IsTrivial call.
      if (IsTrivial(edgeid, destination, origin)) {
        // Find the destination edge and update cost.
        for (const auto& destination_edge : destination.path_edges()) {
          if (destination_edge.graph_id() == edgeid) {
            // a trivial route passes along a single edge, meaning that the
            // destination point must be on this edge, and so the distance
            // remaining must be zero.
            GraphId id(destination_edge.graph_id());
            const DirectedEdge* dest_diredge = tile->directededge(id);
            Cost dest_cost =
                costing_->EdgeCost(dest_diredge, tile) * (1.0f - destination_edge.percent_along());
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
                                 dist, mode_, c, false, false);
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
uint32_t TimeDepReverse::SetDestination(GraphReader& graphreader, const valhalla::Location& dest) {
  // Only skip outbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(dest.path_edges().begin(), dest.path_edges().end(),
                [&has_other_edges](const valhalla::Location::PathEdge& e) {
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
    destinations_[oppedge] = costing_->EdgeCost(directededge, tile) * (1.0f - edge.percent_along());

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
  Cost tc(edgelabels_rev_[dest].transition_cost(), edgelabels_rev_[dest].transition_secs());

  // Form the reverse path from the destination (true origin) using opposing
  // edges.
  std::vector<PathInfo> path;
  Cost cost;
  uint32_t edgelabel_index = dest;
  while (edgelabel_index != kInvalidLabel) {
    const BDEdgeLabel& edgelabel = edgelabels_rev_[edgelabel_index];

    // Get elapsed time on the edge, then add the transition cost at
    // prior edge.
    uint32_t predidx = edgelabel.predecessor();
    if (predidx == kInvalidLabel) {
      cost += edgelabel.cost();
    } else {
      cost += edgelabel.cost() - edgelabels_rev_[predidx].cost();
    }
    cost += tc;
    path.emplace_back(edgelabel.mode(), cost.secs, edgelabel.opp_edgeid(), 0, cost.cost,
                      edgelabel.has_time_restriction());

    // Check if this is a ferry
    if (edgelabel.use() == Use::kFerry) {
      has_ferry_ = true;
    }

    // Update edgelabel_index and transition cost to apply at next iteration
    edgelabel_index = predidx;
    tc.secs = edgelabel.transition_secs();
    tc.cost = edgelabel.transition_cost();
  }

  return path;
}

} // namespace thor
} // namespace valhalla
