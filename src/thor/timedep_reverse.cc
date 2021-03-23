#include "baldr/datetime.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "thor/timedep.h"
#include <algorithm>
#include <string>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

// TODO - compute initial label count based on estimated route length
constexpr uint64_t kInitialEdgeLabelCount = 500000;

// Number of iterations to allow with no convergence to the destination
constexpr uint32_t kMaxIterationsWithoutConvergence = 1800000;

// Default constructor
TimeDepReverse::TimeDepReverse(uint32_t max_reserved_labels_count)
    : TimeDepForward(max_reserved_labels_count) {
  mode_ = TravelMode::kDrive;
  travel_type_ = 0;
  max_label_count_ = std::numeric_limits<uint32_t>::max();
  access_mode_ = kAutoAccess;
}

// Destructor
TimeDepReverse::~TimeDepReverse() {
}

void TimeDepReverse::Clear() {
  TimeDepForward::Clear();
  if (edgelabels_rev_.size() > max_reserved_labels_count_) {
    edgelabels_rev_.resize(max_reserved_labels_count_);
    edgelabels_rev_.shrink_to_fit();
  }
  edgelabels_rev_.clear();
  adjacencylist_rev_.clear();
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

  // Construct adjacency list, clear edge status.
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing_->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_rev_.reuse(mincost, range, bucketsize, &edgelabels_rev_);
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
                                   const TimeInfo& time_info,
                                   const valhalla::Location& destination,
                                   std::pair<int32_t, float>& best_path) {
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  graph_tile_ptr tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return false;
  }
  const NodeInfo* nodeinfo = tile->node(node);

  // Update the time information
  auto offset_time = time_info.reverse(pred.cost().secs, static_cast<int>(nodeinfo->timezone()));

  if (!costing_->Allowed(nodeinfo)) {
    const DirectedEdge* opp_edge;
    const GraphId opp_edge_id = graphreader.GetOpposingEdgeId(pred.edgeid(), opp_edge, tile);
    EdgeStatusInfo* opp_status = edgestatus_.GetPtr(opp_edge_id, tile);
    pred.set_deadend(true);
    return ExpandReverseInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx,
                              {opp_edge, opp_edge_id, opp_status}, tile, offset_time, destination,
                              best_path);
  }

  // Expand from end node.
  EdgeMetadata meta = EdgeMetadata::make(node, nodeinfo, tile, edgestatus_);

  bool disable_uturn = false;
  EdgeMetadata uturn_meta{};

  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++meta) {
    // Begin by checking if this is the opposing edge to pred.
    // If so, it means we are attempting a u-turn. In that case, lets wait with evaluating
    // this edge until last. If any other edges were emplaced, it means we should not
    // even try to evaluate a u-turn since u-turns should only happen for deadends
    uturn_meta = pred.opp_local_idx() == meta.edge->localedgeidx() ? meta : uturn_meta;

    // Expand but only if this isnt the uturn, we'll try that later if nothing else works out
    disable_uturn = (pred.opp_local_idx() != meta.edge->localedgeidx() &&
                     ExpandReverseInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx, meta,
                                        tile, offset_time, destination, best_path)) ||
                    disable_uturn;
  }

  // Handle transitions - expand from the end node of each transition
  if (nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      // if this is a downward transition (ups are always allowed) AND we are no longer allowed OR
      // we cant get the tile at that level (local extracts could have this problem) THEN bail
      graph_tile_ptr trans_tile = nullptr;
      if ((!trans->up() &&
           hierarchy_limits_[trans->endnode().level()].StopExpanding(pred.distance())) ||
          !(trans_tile = graphreader.GetGraphTile(trans->endnode()))) {
        continue;
      }
      // setup for expansion at this level
      hierarchy_limits_[node.level()].up_transition_count += trans->up();
      const auto* trans_node = trans_tile->node(trans->endnode());
      EdgeMetadata trans_meta =
          EdgeMetadata::make(trans->endnode(), trans_node, trans_tile, edgestatus_);
      // expand the edges from this node at this level
      for (uint32_t i = 0; i < trans_node->edge_count(); ++i, ++trans_meta) {
        disable_uturn =
            ExpandReverseInner(graphreader, pred, opp_pred_edge, trans_node, pred_idx, trans_meta,
                               trans_tile, offset_time, destination, best_path) ||
            disable_uturn;
      }
    }
  }

  // Now, after having looked at all the edges, including edges on other levels,
  // we can say if this is a deadend or not, and if so, evaluate the uturn-edge (if it exists)
  if (!disable_uturn && uturn_meta) {
    // If we found no suitable edge to add, it means we're at a deadend
    // so lets go back and re-evaluate a potential u-turn
    pred.set_deadend(true);

    // TODO Is there a shortcut that supersedes our u-turn?
    // Decide if we should expand a shortcut or the non-shortcut edge...

    // We didn't add any shortcut of the uturn, therefore evaluate the regular uturn instead
    disable_uturn = ExpandReverseInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx,
                                       uturn_meta, tile, offset_time, destination, best_path) ||
                    disable_uturn;
  }

  return disable_uturn;
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
                                               const graph_tile_ptr& tile,
                                               const TimeInfo& time_info,
                                               const valhalla::Location& destination,
                                               std::pair<int32_t, float>& best_path) {

  // Skip shortcut edges for time dependent routes. Also skip this edge if permanently labeled (best
  // path already found to this directed edge) or if no access for this mode.
  if (meta.edge->is_shortcut() || !(meta.edge->reverseaccess() & access_mode_)) {
    return false;
  }
  // Skip this edge if permanently labeled (best path already found to this
  // directed edge)
  if (meta.edge_status->set() == EdgeSet::kPermanent) {
    return true; // This is an edge we _could_ have expanded, so return true
  }

  // Get end node tile, opposing edge Id, and opposing directed edge.
  graph_tile_ptr t2 =
      meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
  if (t2 == nullptr) {
    return false;
  }
  GraphId oppedge = t2->GetOpposingEdgeId(meta.edge);
  const DirectedEdge* opp_edge = t2->directededge(oppedge);

  // Skip this edge if no access is allowed (based on costing method)
  // or if a complex restriction prevents transition onto this edge.
  uint8_t restriction_idx = -1;
  const auto reset_func = [this](const GraphId& edgeid) {
    edgestatus_.Update(edgeid, EdgeSet::kUnreachedOrReset);
  };
  if (!costing_->AllowedReverse(meta.edge, pred, opp_edge, t2, oppedge, time_info.local_time,
                                nodeinfo->timezone(), restriction_idx) ||
      costing_->Restricted(meta.edge, pred, edgelabels_rev_, tile, meta.edge_id, false, reset_func,
                           time_info.local_time, nodeinfo->timezone())) {
    return false;
  }

  // Get cost. Use opposing edge for EdgeCost. Separate the transition seconds so we
  // can properly recover elapsed time on the reverse path.
  auto transition_cost =
      costing_->TransitionCostReverse(meta.edge->localedgeidx(), nodeinfo, opp_edge, opp_pred_edge,
                                      pred.has_measured_speed());
  uint8_t flow_sources;
  auto edge_cost = costing_->EdgeCost(opp_edge, t2, time_info.second_of_week, flow_sources);
  Cost newcost = pred.cost() + edge_cost;
  newcost.cost += transition_cost.cost;

  // If this edge is a destination, subtract the partial/remainder cost
  // (cost from the dest. location to the end of the edge).
  auto p = destinations_percent_along_.find(meta.edge_id);
  if (p != destinations_percent_along_.end()) {
    // Adapt cost to potentially not using the entire destination edge
    newcost -= edge_cost * p->second;

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
      adjacencylist_rev_.decrease(meta.edge_status->index(), newsortcost);
      lab.Update(pred_idx, newcost, newsortcost, transition_cost, restriction_idx);
    }
    return true;
  }

  // If this is a destination edge the A* heuristic is 0. Otherwise the
  // sort cost (with A* heuristic) is found using the lat,lng at the
  // end node of the directed edge.
  float dist = 0.0f;
  float sortcost = newcost.cost;
  if (p == destinations_percent_along_.end()) {
    graph_tile_ptr t2 =
        meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
    if (t2 == nullptr) {
      return false;
    }
    sortcost += astarheuristic_.Get(t2->get_node_ll(meta.edge->endnode()), dist);
  }

  // Add edge label, add to the adjacency list and set edge status
  uint32_t idx = edgelabels_rev_.size();
  edgelabels_rev_.emplace_back(pred_idx, meta.edge_id, oppedge, meta.edge, newcost, sortcost, dist,
                               mode_, transition_cost,
                               (pred.not_thru_pruning() || !meta.edge->not_thru()),
                               (pred.closure_pruning() || !(costing_->IsClosed(meta.edge, tile))),
                               static_cast<bool>(flow_sources & kDefaultFlowMask), restriction_idx);
  adjacencylist_rev_.add(idx);
  *meta.edge_status = {EdgeSet::kTemporary, idx};

  return true;
}

// Calculate time-dependent best path using a reverse search. Supports
// "arrive-by" routes.
std::vector<std::vector<PathInfo>>
TimeDepReverse::GetBestPath(valhalla::Location& origin,
                            valhalla::Location& destination,
                            GraphReader& graphreader,
                            const sif::mode_costing_t& mode_costing,
                            const TravelMode mode,
                            const Options& /*options*/) {
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

  // Get time information for backward search
  auto reverse_time_info = TimeInfo::make(destination, graphreader, &tz_cache_);

  // Initialize the locations. For a reverse path search the destination location
  // is used as the "origin" and the origin location is used as the "destination".
  uint32_t density = SetDestination(graphreader, origin);
  SetOrigin(graphreader, destination, origin, reverse_time_info.second_of_week);

  // Update hierarchy limits
  ModifyHierarchyLimits(mindist, density);

  // Find shortest path
  uint32_t nc = 0; // Count of iterations with no convergence
                   // towards destination
  std::pair<int32_t, float> best_path = std::make_pair(-1, 0.0f);
  graph_tile_ptr tile;
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
    uint32_t predindex = adjacencylist_rev_.pop();

    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " + std::to_string(edgelabels_rev_.size()));
      return {};
    }

    // Copy the BDEdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    BDEdgeLabel pred = edgelabels_rev_[predindex];
    if (destinations_percent_along_.find(pred.edgeid()) != destinations_percent_along_.end()) {
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
    } else if (nc++ > kMaxIterationsWithoutConvergence) {
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

    // Get the opposing predecessor directed edge. Need to make sure we get
    // the correct one if a transition occurred
    const DirectedEdge* opp_pred_edge =
        graphreader.GetGraphTile(pred.opp_edgeid())->directededge(pred.opp_edgeid());

    // Expand forward from the end node of the predecessor edge.
    ExpandReverse(graphreader, pred.endnode(), pred, predindex, opp_pred_edge, reverse_time_info,
                  destination, best_path);
  }
  return {}; // Should never get here
}

// The origin of the reverse path is the destination location.
// TODO - how do we set the
void TimeDepReverse::SetOrigin(GraphReader& graphreader,
                               const valhalla::Location& origin,
                               const valhalla::Location& destination,
                               const uint32_t seconds_of_week) {
  // Only skip outbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.path_edges().begin(), origin.path_edges().end(),
                [&has_other_edges](const valhalla::Location::PathEdge& e) {
                  has_other_edges = has_other_edges || !e.begin_node();
                });

  // Check if the origin edge matches a destination edge at the node.
  auto trivial_at_node = [this, &destination](const valhalla::Location::PathEdge& edge) {
    auto p = destinations_percent_along_.find(edge.graph_id());
    if (p != destinations_percent_along_.end()) {
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
    graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the opposing directed edge, continue if we cannot get it
    GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid);
    if (!opp_edge_id.Is_Valid()) {
      continue;
    }
    const DirectedEdge* opp_dir_edge = graphreader.GetOpposingEdge(edgeid);

    // Get cost
    uint8_t flow_sources;
    Cost cost =
        costing_->EdgeCost(directededge, tile, seconds_of_week, flow_sources) * edge.percent_along();
    float dist = astarheuristic_.GetDistance(tile->get_node_ll(opp_dir_edge->endnode()));

    // We need to penalize this location based on its score (distance in meters from input)
    // We assume the slowest speed you could travel to cover that distance to start/end the route
    // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
    // Perhaps need to adjust score?
    cost.cost += edge.distance();

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge) if the
    // destination is in a forward direction along the edge.
    auto settled_dest_edge = destinations_percent_along_.find(opp_edge_id);
    if (settled_dest_edge != destinations_percent_along_.end()) {
      // Reverse the origin and destination in the IsTrivial call.
      if (IsTrivial(edgeid, destination, origin)) {
        // Find the destination edge and update cost.
        for (const auto& dest_path_edge : destination.path_edges()) {
          if (dest_path_edge.graph_id() == edgeid) {
            // a trivial route passes along a single edge, meaning that the
            // destination point must be on this edge, and so the distance
            // remaining must be zero.
            GraphId id(dest_path_edge.graph_id());
            const DirectedEdge* dest_edge = tile->directededge(id);
            Cost remainder_cost = costing_->EdgeCost(dest_edge, tile, seconds_of_week, flow_sources) *
                                  (dest_path_edge.percent_along());
            // Remove the cost of the final "unused" part of the destination edge
            cost -= remainder_cost;
            // Add back in the edge score/penalty to account for destination edges
            // farther from the input location lat,lon.
            cost.cost += dest_path_edge.distance();
            cost.cost = std::max(0.0f, cost.cost);
            dist = 0.0;
          }
        }
      }
    }
    // Store the closest node info
    if (closest_ni == nullptr) {
      closest_ni = nodeinfo;
    }

    // Compute sortcost
    float sortcost = cost.cost + astarheuristic_.Get(dist);

    // Add BDEdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path. Make sure the opposing
    // edge (edgeid) is set.
    // DO NOT SET EdgeStatus - it messes up trivial paths with oneways
    uint32_t idx = edgelabels_rev_.size();
    edgelabels_rev_.emplace_back(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, sortcost,
                                 dist, mode_, c, false, !(costing_->IsClosed(directededge, tile)),
                                 static_cast<bool>(flow_sources & kDefaultFlowMask), -1);
    adjacencylist_rev_.add(idx);

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
    auto tile = graphreader.GetGraphTile(id);
    const DirectedEdge* directededge = tile->directededge(id);

    // The opposing edge Id is added as a destination since the search
    // is done in reverse direction.
    auto t2 = directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
    if (!t2) {
      continue;
    }
    GraphId oppedge = t2->GetOpposingEdgeId(directededge);
    destinations_percent_along_[oppedge] = edge.percent_along();

    // Edge score (penalty) is handled within GetPath. Do not add score here.

    // Get the tile relative density
    density = tile->header()->density();
  }
  return density;
}

// Form the path from the adjacency list.
std::vector<PathInfo> TimeDepReverse::FormPath(GraphReader& /*graphreader*/, const uint32_t dest) {
  // Metrics to track
  LOG_DEBUG("path_cost::" + std::to_string(edgelabels_rev_[dest].cost().cost));
  LOG_DEBUG("path_iterations::" + std::to_string(edgelabels_rev_.size()));

  // Form the reverse path from the destination (true origin) using opposing edges.
  std::vector<PathInfo> path;
  Cost cost, previous_transition_cost;
  uint32_t edgelabel_index = dest;
  while (edgelabel_index != kInvalidLabel) {
    const BDEdgeLabel& edgelabel = edgelabels_rev_[edgelabel_index];

    // Get elapsed time on the edge, then add the transition cost at prior edge.
    uint32_t predidx = edgelabel.predecessor();
    if (predidx == kInvalidLabel) {
      cost += edgelabel.cost();
    } else {
      cost += edgelabel.cost() - edgelabels_rev_[predidx].cost();
    }
    cost += previous_transition_cost;
    path.emplace_back(edgelabel.mode(), cost, edgelabel.opp_edgeid(), 0, edgelabel.restriction_idx(),
                      previous_transition_cost);

    // Check if this is a ferry
    if (edgelabel.use() == Use::kFerry) {
      has_ferry_ = true;
    }

    // Update edgelabel_index and transition cost to apply at next iteration
    edgelabel_index = predidx;
    // We apply the turn cost at the beginning of the edge, as is done in the forward path
    // Semantically this can be thought of is, how much time did it take to turn onto this edge
    // To do this we need to carry the cost forward to the next edge in the path so we cache it here
    previous_transition_cost = edgelabel.transition_cost();
  }

  return path;
}

} // namespace thor
} // namespace valhalla
