#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "thor/timedep.h"
#include <algorithm>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

constexpr uint32_t kInitialEdgeLabelCount = 500000;

// Number of iterations to allow with no convergence to the destination
constexpr uint32_t kMaxIterationsWithoutConvergence = 1800000;

// Default constructor
TimeDepForward::TimeDepForward(uint32_t max_reserved_labels_count)
    : PathAlgorithm(), max_label_count_(std::numeric_limits<uint32_t>::max()),
      mode_(TravelMode::kDrive), travel_type_(0),
      max_reserved_labels_count_(max_reserved_labels_count) {
}

// Destructor
TimeDepForward::~TimeDepForward() {
}

// Clear the temporary information generated during path construction.
void TimeDepForward::Clear() {
  // Clear the edge labels and destination list. Reset the adjacency list
  // and clear edge status.
  if (edgelabels_.size() > max_reserved_labels_count_) {
    edgelabels_.resize(max_reserved_labels_count_);
    edgelabels_.shrink_to_fit();
  }
  edgelabels_.clear();
  destinations_percent_along_.clear();
  adjacencylist_.clear();
  edgestatus_.clear();

  // Set the ferry flag to false
  has_ferry_ = false;
}

// Expand from the node along the forward search path. Immediately expands
// from the end node of any transition edge (so no transition edges are added
// to the adjacency list or EdgeLabel list). Does not expand transition
// edges if from_transition is false.
bool TimeDepForward::ExpandForward(GraphReader& graphreader,
                                   const GraphId& node,
                                   EdgeLabel& pred,
                                   const uint32_t pred_idx,
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
  auto offset_time = time_info.forward(pred.cost().secs, static_cast<int>(nodeinfo->timezone()));

  if (!costing_->Allowed(nodeinfo)) {
    const DirectedEdge* opp_edge;
    const GraphId opp_edge_id = graphreader.GetOpposingEdgeId(pred.edgeid(), opp_edge, tile);
    // Check if edge is null before using it (can happen with regional data sets)
    pred.set_deadend(true);
    return opp_edge &&
           ExpandForwardInner(graphreader, pred, nodeinfo, pred_idx,
                              {opp_edge, opp_edge_id, edgestatus_.GetPtr(opp_edge_id, tile)}, tile,
                              offset_time, destination, best_path);
  }

  // Expand from start node.
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
                     ExpandForwardInner(graphreader, pred, nodeinfo, pred_idx, meta, tile,
                                        offset_time, destination, best_path)) ||
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
        disable_uturn = ExpandForwardInner(graphreader, pred, trans_node, pred_idx, trans_meta,
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
    disable_uturn = ExpandForwardInner(graphreader, pred, nodeinfo, pred_idx, uturn_meta, tile,
                                       offset_time, destination, best_path) ||
                    disable_uturn;
  }

  return disable_uturn;
}

// Runs in the inner loop of `ExpandForward`, essentially evaluating if
// the edge described in `meta` should be placed on the stack
// as well as doing just that.
//
// Returns true if any edge _could_ have been expanded after restrictions etc.
inline bool TimeDepForward::ExpandForwardInner(GraphReader& graphreader,
                                               const EdgeLabel& pred,
                                               const NodeInfo* nodeinfo,
                                               const uint32_t pred_idx,
                                               const EdgeMetadata& meta,
                                               const graph_tile_ptr& tile,
                                               const TimeInfo& time_info,
                                               const valhalla::Location& destination,
                                               std::pair<int32_t, float>& best_path) {

  // Skip this edge if permanently labeled (best path already found to this
  // directed edge)
  if (meta.edge_status->set() == EdgeSet::kPermanent) {
    return true; // This is an edge we _could_ have expanded, so return true
  }
  // Skip shortcut edges for time dependent routes, if no access is allowed to this edge
  // (based on costing method)
  uint8_t restriction_idx = -1;
  if (meta.edge->is_shortcut() ||
      !costing_->Allowed(meta.edge, pred, tile, meta.edge_id, time_info.local_time,
                         nodeinfo->timezone(), restriction_idx) ||
      costing_->Restricted(meta.edge, pred, edgelabels_, tile, meta.edge_id, true, &edgestatus_,
                           time_info.local_time, nodeinfo->timezone())) {
    return false;
  }

  // Compute the cost to the end of this edge
  uint8_t flow_sources;
  auto edge_cost = costing_->EdgeCost(meta.edge, tile, time_info.second_of_week, flow_sources);
  auto transition_cost = costing_->TransitionCost(meta.edge, nodeinfo, pred);
  Cost newcost = pred.cost() + edge_cost + transition_cost;

  // If this edge is a destination, subtract the partial/remainder cost
  // (cost from the dest. location to the end of the edge).
  auto dest_edge = destinations_percent_along_.find(meta.edge_id);
  if (dest_edge != destinations_percent_along_.end()) {
    // Adapt cost to potentially not using the entire destination edge
    newcost -= edge_cost * (1.0f - dest_edge->second);

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
                                                                         : edgelabels_.size();
      best_path.second = newcost.cost;
    }
  }

  // Check if edge is temporarily labeled and this path has less cost. If
  // less cost the predecessor is updated and the sort cost is decremented
  // by the difference in real cost (A* heuristic doesn't change)
  if (meta.edge_status->set() == EdgeSet::kTemporary) {
    EdgeLabel& lab = edgelabels_[meta.edge_status->index()];
    if (newcost.cost < lab.cost().cost) {
      float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
      adjacencylist_.decrease(meta.edge_status->index(), newsortcost);
      lab.Update(pred_idx, newcost, newsortcost, transition_cost, restriction_idx);
    }
    return true;
  }

  // If this is a destination edge the A* heuristic is 0. Otherwise the
  // sort cost (with A* heuristic) is found using the lat,lng at the
  // end node of the directed edge.
  float dist = 0.0f;
  float sortcost = newcost.cost;
  if (dest_edge == destinations_percent_along_.end()) {
    graph_tile_ptr t2 =
        meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
    if (t2 == nullptr) {
      return false;
    }
    sortcost += astarheuristic_.Get(t2->get_node_ll(meta.edge->endnode()), dist);
  }

  // Add to the adjacency list and edge labels.
  uint32_t idx = edgelabels_.size();
  edgelabels_.emplace_back(pred_idx, meta.edge_id, meta.edge, newcost, sortcost, dist, mode_, 0,
                           transition_cost, restriction_idx,
                           (pred.closure_pruning() || !(costing_->IsClosed(meta.edge, tile))),
                           static_cast<bool>(flow_sources & kDefaultFlowMask),
                           costing_->TurnType(pred.opp_local_idx(),nodeinfo,meta.edge));
  *meta.edge_status = {EdgeSet::kTemporary, idx};
  adjacencylist_.add(idx);
  return true;
}

// Calculate time-dependent best path using a forward search. Supports
// "depart-at" routes.
std::vector<std::vector<PathInfo>>
TimeDepForward::GetBestPath(valhalla::Location& origin,
                            valhalla::Location& destination,
                            GraphReader& graphreader,
                            const sif::mode_costing_t& mode_costing,
                            const TravelMode mode,
                            const Options& /*options*/) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  // Note: because we can correlate to more than one place for a given PathLocation
  // using edges.front here means we are only setting the heuristics to one of them
  // alternate paths using the other correlated points to may be harder to find
  midgard::PointLL origin_new(origin.path_edges(0).ll().lng(), origin.path_edges(0).ll().lat());
  midgard::PointLL destination_new(destination.path_edges(0).ll().lng(),
                                   destination.path_edges(0).ll().lat());
  Init(origin_new, destination_new);
  float mindist = astarheuristic_.GetDistance(origin_new);

  // Get time information for forward
  auto forward_time_info = TimeInfo::make(origin, graphreader, &tz_cache_);

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  uint32_t density = SetDestination(graphreader, destination);
  // Call SetOrigin with kFreeFlowSecondOfDay for now since we don't yet have
  // a timezone for converting a date_time of "current" to seconds_of_week
  SetOrigin(graphreader, origin, destination, forward_time_info.second_of_week);

  // Update hierarchy limits
  ModifyHierarchyLimits(mindist, density);

  // Find shortest path
  uint32_t nc = 0; // Count of iterations with no convergence
                   // towards destination
  std::pair<int32_t, float> best_path = std::make_pair(-1, 0.0f);
  size_t total_labels = 0;
  while (true) {
    // Allow this process to be aborted
    size_t current_labels = edgelabels_.size();
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
    const uint32_t predindex = adjacencylist_.pop();
    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " + std::to_string(edgelabels_.size()));
      return {};
    }

    // Copy the EdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    EdgeLabel pred = edgelabels_[predindex];
    if (destinations_percent_along_.find(pred.edgeid()) != destinations_percent_along_.end()) {
      // Check if a trivial path. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        if (IsTrivial(pred.edgeid(), origin, destination)) {
          return {FormPath(predindex)};
        }
      } else {
        return {FormPath(predindex)};
      }
    }

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge (this will allow loops/around the block cases)
    if (!pred.origin()) {
      edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Check that distance is converging towards the destination. Return route
    // failure if no convergence for TODO iterations. NOTE: due to somewhat high
    // penalty for entering a destination only (private) road this value needs to
    // be high.
    float dist2dest = pred.distance();
    if (dist2dest < mindist) {
      mindist = dist2dest;
      nc = 0;
    } else if (nc++ > kMaxIterationsWithoutConvergence) {
      if (best_path.first >= 0) {
        return {FormPath(best_path.first)};
      } else {
        LOG_ERROR("No convergence to destination after = " + std::to_string(edgelabels_.size()));
        return {};
      }
    }

    // Do not expand based on hierarchy level based on number of upward
    // transitions and distance to the destination
    if (hierarchy_limits_[pred.endnode().level()].StopExpanding(dist2dest)) {
      continue;
    }

    // Expand forward from the end node of the predecessor edge.
    ExpandForward(graphreader, pred.endnode(), pred, predindex, forward_time_info, destination,
                  best_path);
  }
  return {}; // Should never get here
}

// Initialize prior to finding best path
void TimeDepForward::Init(const midgard::PointLL& origll, const midgard::PointLL& destll) {
  LOG_TRACE("Orig LL = " + std::to_string(origll.lat()) + "," + std::to_string(origll.lng()));
  LOG_TRACE("Dest LL = " + std::to_string(destll.lat()) + "," + std::to_string(destll.lng()));

  // Set the destination and cost factor in the A* heuristic
  astarheuristic_.Init(destll, costing_->AStarCostFactor());

  // Get the initial cost based on A* heuristic from origin
  float mincost = astarheuristic_.Get(origll);

  // Reserve size for edge labels - do this here rather than in constructor so
  // to limit how much extra memory is used for persistent objects.
  // TODO - reserve based on estimate based on distance and route type.
  edgelabels_.reserve(std::min(max_reserved_labels_count_, kInitialEdgeLabelCount));

  // Construct adjacency list, clear edge status.
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing_->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_.reuse(mincost, range, bucketsize, &edgelabels_);
  edgestatus_.clear();

  // Get hierarchy limits from the costing. Get a copy since we increment
  // transition counts (i.e., this is not a const reference).
  hierarchy_limits_ = costing_->GetHierarchyLimits();
}

// Modulate the hierarchy expansion within distance based on density at
// the destination (increase distance for lower densities and decrease
// for higher densities) and the distance between origin and destination
// (increase for shorter distances).
void TimeDepForward::ModifyHierarchyLimits(const float dist, const uint32_t /*density*/) {
  // TODO - default distance below which we increase expansion within
  // distance. This is somewhat temporary to address route quality on shorter
  // routes - hopefully we will mark the data somehow to indicate how to
  // use the hierarchy when approaching the destination (or use a
  // bi-directional search without hierarchies for shorter routes).
  float factor = 1.0f;
  if (25000.0f < dist && dist < 100000.0f) {
    factor = std::min(3.0f, 100000.0f / dist);
  }
  /* TODO - need a reliable density factor near the destination (e.g. tile level?)
  // Low density - increase expansion within distance.
  // High density - decrease expansion within distance.
  if (density < 8) {
    float f = 1.0f + (8.0f - density) * 0.125f;
    factor *= f;
  } else if (density > 8) {
    float f = 0.5f + (15.0f - density) * 0.0625;
    factor *= f;
  }*/
  // TODO - just arterial for now...investigate whether to alter local as well
  hierarchy_limits_[1].expansion_within_dist *= factor;
}

// Add an edge at the origin to the adjacency list
void TimeDepForward::SetOrigin(GraphReader& graphreader,
                               const valhalla::Location& origin,
                               const valhalla::Location& destination,
                               const uint32_t seconds_of_week) {
  // Only skip inbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.path_edges().begin(), origin.path_edges().end(),
                [&has_other_edges](const valhalla::Location::PathEdge& e) {
                  has_other_edges = has_other_edges || !e.end_node();
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
  for (const auto& edge : origin.path_edges()) {
    // If origin is at a node - skip any inbound edge (dist = 1) unless the
    // destination is also at the same end node (trivial path).
    if (has_other_edges && edge.end_node() && !trivial_at_node(edge)) {
      continue;
    }

    // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
    GraphId edgeid(edge.graph_id());
    if (costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
      continue;
    }

    // Get the directed edge
    const auto tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the tile at the end node. Skip if tile not found as we won't be
    // able to expand from this origin edge.
    const auto endtile = graphreader.GetGraphTile(directededge->endnode());
    if (endtile == nullptr) {
      continue;
    }

    // Get cost
    uint8_t flow_sources;
    Cost cost = costing_->EdgeCost(directededge, tile, seconds_of_week, flow_sources) *
                (1.0f - edge.percent_along());
    float dist = astarheuristic_.GetDistance(endtile->get_node_ll(directededge->endnode()));

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
    auto settled_dest_edge = destinations_percent_along_.find(edgeid);
    if (settled_dest_edge != destinations_percent_along_.end()) {
      if (IsTrivial(edgeid, origin, destination)) {
        // Find the destination edge and update cost.
        for (const auto& dest_path_edge : destination.path_edges()) {
          if (dest_path_edge.graph_id() == edgeid) {
            // a trivial route passes along a single edge, meaning that the
            // destination point must be on this edge, and so the distance
            // remaining must be zero.
            GraphId id(dest_path_edge.graph_id());
            const DirectedEdge* dest_edge = tile->directededge(id);
            Cost remainder_cost = costing_->EdgeCost(dest_edge, tile, seconds_of_week, flow_sources) *
                                  (1.0f - dest_path_edge.percent_along());
            // Remove the cost of the final "unused" part of the destination edge
            cost -= remainder_cost;
            // Add back in the edge score/penalty to account for destination edges
            // farther from the input location lat,lon.
            cost.cost += dest_path_edge.distance();
            cost.cost = std::max(0.0f, cost.cost);
            dist = 0.0;
            break;
          }
        }
      }
    }

    // Compute sortcost
    float sortcost = cost.cost + astarheuristic_.Get(dist);

    // Add EdgeLabel to the adjacency list (but do not set its status).
    // Set the predecessor edge index to invalid to indicate the origin
    // of the path.
    uint32_t d = static_cast<uint32_t>(directededge->length() * (1.0f - edge.percent_along()));
    EdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost, sortcost, dist, mode_, d, Cost{},
                         baldr::kInvalidRestriction, !(costing_->IsClosed(directededge, tile)),
                         static_cast<bool>(flow_sources & kDefaultFlowMask),
                         sif::InternalTurn::kNoTurn);
    // Set the origin flag
    edge_label.set_origin();

    // Add EdgeLabel to the adjacency list
    uint32_t idx = edgelabels_.size();
    edgelabels_.push_back(edge_label);
    adjacencylist_.add(idx);

    // DO NOT SET EdgeStatus - it messes up trivial paths with oneways
  }
}

// Add a destination edge
uint32_t TimeDepForward::SetDestination(GraphReader& graphreader, const valhalla::Location& dest) {
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

    // Disallow any user avoided edges if the avoid location is behind the destination along the edge
    GraphId edgeid(edge.graph_id());
    if (costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
      continue;
    }

    // Keep the cost to traverse the partial distance for the remainder of the edge. This cost
    // is subtracted from the total cost up to the end of the destination edge.
    destinations_percent_along_[edge.graph_id()] = edge.percent_along();

    // Edge score (penalty) is handled within GetPath. Do not add score here.

    // Get the tile relative density
    auto tile = graphreader.GetGraphTile(edgeid);
    density = tile->header()->density();
  }
  return density;
}

// Form the path from the adjacency list.
std::vector<PathInfo> TimeDepForward::FormPath(const uint32_t dest) {
  // Metrics to track
  LOG_DEBUG("path_cost::" + std::to_string(edgelabels_[dest].cost().cost));
  LOG_DEBUG("path_iterations::" + std::to_string(edgelabels_.size()));

  // Work backwards from the destination
  std::vector<PathInfo> path;
  for (auto edgelabel_index = dest; edgelabel_index != kInvalidLabel;
       edgelabel_index = edgelabels_[edgelabel_index].predecessor()) {
    const EdgeLabel& edgelabel = edgelabels_[edgelabel_index];
    path.emplace_back(edgelabel.mode(), edgelabel.cost(), edgelabel.edgeid(), 0,
                      edgelabel.restriction_idx(), edgelabel.transition_cost());

    // Check if this is a ferry
    if (edgelabel.use() == Use::kFerry) {
      has_ferry_ = true;
    }
  }

  // Reverse the list and return
  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace thor
} // namespace valhalla
