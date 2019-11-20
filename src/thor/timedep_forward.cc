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

constexpr uint64_t kInitialEdgeLabelCount = 500000;

// Number of iterations to allow with no convergence to the destination
constexpr uint32_t kMaxIterationsWithoutConvergence = 200000;

// Default constructor
TimeDepForward::TimeDepForward() : AStarPathAlgorithm() {
  mode_ = TravelMode::kDrive;
  travel_type_ = 0;
  adjacencylist_ = nullptr;
  max_label_count_ = std::numeric_limits<uint32_t>::max();
  origin_tz_index_ = 0;
  seconds_of_week_ = 0;
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

  // Adjust for time zone (if different from timezone at the start).
  if (nodeinfo->timezone() != origin_tz_index_) {
    // Get the difference in seconds between the origin tz and current tz
    int tz_diff =
        DateTime::timezone_diff(localtime, DateTime::get_tz_db().from_index(origin_tz_index_),
                                DateTime::get_tz_db().from_index(nodeinfo->timezone()));
    localtime += tz_diff;
    seconds_of_week = DateTime::normalize_seconds_of_week(seconds_of_week + tz_diff);
  }

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
                                          localtime, seconds_of_week, destination, best_path) ||
                       found_valid_edge;
  }

  // Handle transitions - expand from the end node of each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      if (trans->up()) {
        hierarchy_limits_[node.level()].up_transition_count++;
        found_valid_edge = ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true,
                                         localtime, seconds_of_week, destination, best_path) ||
                           found_valid_edge;
      } else if (!hierarchy_limits_[trans->endnode().level()].StopExpanding(pred.distance())) {
        found_valid_edge = ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true,
                                         localtime, seconds_of_week, destination, best_path) ||
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
inline bool TimeDepForward::ExpandForwardInner(GraphReader& graphreader,
                                               const EdgeLabel& pred,
                                               const NodeInfo* nodeinfo,
                                               const uint32_t pred_idx,
                                               const EdgeMetadata& meta,
                                               const GraphTile* tile,
                                               uint64_t localtime,
                                               uint32_t seconds_of_week,
                                               const valhalla::Location& destination,
                                               std::pair<int32_t, float>& best_path) {

  // Skip shortcut edges for time dependent routes. Also skip this edge if permanently labeled
  // (best path already found to this directed edge), if no access is allowed to this edge
  // (based on costing method), or if a complex restriction exists.
  bool has_time_restrictions = false;
  if (meta.edge->is_shortcut() || meta.edge_status->set() == EdgeSet::kPermanent ||
      !costing_->Allowed(meta.edge, pred, tile, meta.edge_id, localtime, nodeinfo->timezone(),
                         has_time_restrictions) ||
      costing_->Restricted(meta.edge, pred, edgelabels_, tile, meta.edge_id, true, localtime,
                           nodeinfo->timezone())) {
    return false;
  }

  // Compute the cost to the end of this edge
  Cost newcost = pred.cost() + costing_->EdgeCost(meta.edge, tile, seconds_of_week) +
                 costing_->TransitionCost(meta.edge, nodeinfo, pred);

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
      lab.Update(pred_idx, newcost, newsortcost, has_time_restrictions);
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

  // Add to the adjacency list and edge labels.
  uint32_t idx = edgelabels_.size();
  edgelabels_.emplace_back(pred_idx, meta.edge_id, meta.edge, newcost, sortcost, dist, mode_, 0,
                           has_time_restrictions);
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

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  uint32_t density = SetDestination(graphreader, destination);
  SetOrigin(graphreader, origin, destination);

  // Set the origin timezone to be the timezone at the end node
  origin_tz_index_ = edgelabels_.size() == 0 ? 0 : GetTimezone(graphreader, edgelabels_[0].endnode());
  if (origin_tz_index_ == 0) {
    // TODO - do not throw exception at this time
    LOG_ERROR("Could not get the timezone at the origin");
  }

  // Set route start time (seconds from epoch)
  uint64_t start_time =
      DateTime::seconds_since_epoch(origin.date_time(),
                                    DateTime::get_tz_db().from_index(origin_tz_index_));

  // Set seconds from beginning of the week
  seconds_of_week_ = DateTime::day_of_week(origin.date_time()) * midgard::kSecondsPerDay +
                     DateTime::seconds_from_midnight(origin.date_time());

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
    if (destinations_.find(pred.edgeid()) != destinations_.end()) {
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

    // Set local time and seconds of the week.
    uint64_t localtime = start_time + static_cast<uint32_t>(pred.cost().secs);
    int32_t seconds_of_week = seconds_of_week_ + static_cast<uint32_t>(pred.cost().secs);
    if (seconds_of_week > midgard::kSecondsPerWeek) {
      seconds_of_week -= midgard::kSecondsPerWeek;
    }

    // Expand forward from the end node of the predecessor edge.
    ExpandForward(graphreader, pred.endnode(), pred, predindex, false, localtime, seconds_of_week,
                  destination, best_path);
  }
  return {}; // Should never get here
}

} // namespace thor
} // namespace valhalla
