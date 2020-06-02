#include "baldr/datetime.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "thor/timedep.h"
#include <algorithm>
#include <map>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

constexpr uint64_t kInitialEdgeLabelCount = 500000;

// Number of iterations to allow with no convergence to the destination
constexpr uint32_t kMaxIterationsWithoutConvergence = 800000;

// Default constructor
TimeDepForward::TimeDepForward() : AStarPathAlgorithm() {
  mode_ = TravelMode::kDrive;
  travel_type_ = 0;
  adjacencylist_ = nullptr;
  max_label_count_ = std::numeric_limits<uint32_t>::max();
}

// Destructor
TimeDepForward::~TimeDepForward() {
  Clear();
}

// Expand from the node along the forward search path. Immediately expands
// from the end node of any transition edge (so no transition edges are added
// to the adjacency list or EdgeLabel list). Does not expand transition
// edges if from_transition is false.
bool TimeDepForward::ExpandForward(GraphReader& graphreader,
                                   const GraphId& node,
                                   EdgeLabel& pred,
                                   const uint32_t pred_idx,
                                   const bool from_transition,
                                   const TimeInfo& time_info,
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

  // Update the time information
  auto offset_time =
      from_transition ? time_info
                      : time_info.forward(pred.cost().secs, static_cast<int>(nodeinfo->timezone()));
  // std::cout << pred.edgeid() << " " << offset_time << std::endl;

  // Expand from start node.
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

    found_valid_edge = ExpandForwardInner(graphreader, pred, nodeinfo, pred_idx, meta, tile,
                                          offset_time, destination, best_path) ||
                       found_valid_edge;
  }

  // Handle transitions - expand from the end node of each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      if (trans->up()) {
        hierarchy_limits_[node.level()].up_transition_count++;
        found_valid_edge = ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true,
                                         offset_time, destination, best_path) ||
                           found_valid_edge;
      } else if (!hierarchy_limits_[trans->endnode().level()].StopExpanding(pred.distance())) {
        found_valid_edge = ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true,
                                         offset_time, destination, best_path) ||
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
        found_valid_edge = ExpandForwardInner(graphreader, pred, nodeinfo, pred_idx, uturn_meta, tile,
                                              offset_time, destination, best_path) ||
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
inline bool TimeDepForward::ExpandForwardInner(GraphReader& graphreader,
                                               const EdgeLabel& pred,
                                               const NodeInfo* nodeinfo,
                                               const uint32_t pred_idx,
                                               const EdgeMetadata& meta,
                                               const GraphTile* tile,
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
  bool has_time_restrictions = false;
  if (meta.edge->is_shortcut() ||
      !costing_->Allowed(meta.edge, pred, tile, meta.edge_id, time_info.local_time,
                         nodeinfo->timezone(), has_time_restrictions) ||
      costing_->Restricted(meta.edge, pred, edgelabels_, tile, meta.edge_id, true, &edgestatus_,
                           time_info.local_time, nodeinfo->timezone())) {
    return false;
  }

  // Compute the cost to the end of this edge
  auto edge_cost = costing_->EdgeCost(meta.edge, tile, time_info.second_of_week);
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
      adjacencylist_->decrease(meta.edge_status->index(), newsortcost);
      lab.Update(pred_idx, newcost, newsortcost, transition_cost, has_time_restrictions);
    }
    return true;
  }

  // If this is a destination edge the A* heuristic is 0. Otherwise the
  // sort cost (with A* heuristic) is found using the lat,lng at the
  // end node of the directed edge.
  float dist = 0.0f;
  float sortcost = newcost.cost;
  if (dest_edge == destinations_percent_along_.end()) {
    const GraphTile* t2 =
        meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
    if (t2 == nullptr) {
      return false;
    }
    sortcost += astarheuristic_.Get(t2->get_node_ll(meta.edge->endnode()), dist);
  }

  // Add to the adjacency list and edge labels.
  uint32_t idx = edgelabels_.size();
  edgelabels_.emplace_back(pred_idx, meta.edge_id, meta.edge, newcost, sortcost, dist, mode_, 0,
                           transition_cost, has_time_restrictions);
  *meta.edge_status = {EdgeSet::kTemporary, idx};
  adjacencylist_->add(idx);
  return true;
}

// Calculate time-dependent best path using a forward search. Supports
// "depart-at" routes.
std::vector<std::vector<PathInfo>>
TimeDepForward::GetBestPath(valhalla::Location& origin,
                            valhalla::Location& destination,
                            GraphReader& graphreader,
                            const std::shared_ptr<DynamicCost>* mode_costing,
                            const TravelMode mode,
                            const Options& options) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();

  // date_time must be set on the origin. Log an error but allow routes for now.
  if (!origin.has_date_time()) {
    LOG_ERROR("TimeDepForward called without time set on the origin location");
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

  // Get time information for forward
  auto forward_time_info = TimeInfo::make(origin, graphreader, &tz_cache_);

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  uint32_t density = SetDestination(graphreader, destination);
  // Call SetOrigin with kFreeFlowSecondOfDay for now since we don't yet have
  // a timezone for converting a date_time of "current" to seconds_of_week
  SetOrigin(graphreader, origin, destination,
            kInvalidSecondsOfWeek /*forward_time_info.second_of_week*/);

  // Update hierarchy limits
  ModifyHierarchyLimits(mindist, density);

  // Find shortest path
  uint32_t nc = 0; // Count of iterations with no convergence
                   // towards destination
  std::pair<int32_t, float> best_path = std::make_pair(-1, 0.0f);
  const GraphTile* tile;
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
    uint32_t predindex = adjacencylist_->pop();
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
    ExpandForward(graphreader, pred.endnode(), pred, predindex, false, forward_time_info, destination,
                  best_path);
  }
  return {}; // Should never get here
}

} // namespace thor
} // namespace valhalla
