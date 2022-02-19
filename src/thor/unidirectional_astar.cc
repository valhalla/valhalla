#include "thor/unidirectional_astar.h"
#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include <algorithm>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

constexpr uint32_t kInitialEdgeLabelCount = 500000;

// Number of iterations to allow with no convergence to the destination
constexpr uint32_t kMaxIterationsWithoutConvergence = 1800000;

template <const ExpansionType expansion_direction, const bool FORWARD>
UnidirectionalAStar<expansion_direction, FORWARD>::UnidirectionalAStar(
    const boost::property_tree::ptree& config)
    : PathAlgorithm(config.get<uint32_t>("max_reserved_labels_count", kInitialEdgeLabelCount),
                    config.get<bool>("clear_reserved_memory", false)),
      max_label_count_(std::numeric_limits<uint32_t>::max()), mode_(travel_mode_t::kDrive),
      travel_type_(0), access_mode_{kAutoAccess} {
}

// Default constructor
template UnidirectionalAStar<ExpansionType::forward>::UnidirectionalAStar(
    const boost::property_tree::ptree& config);

// Default constructor
template UnidirectionalAStar<ExpansionType::reverse>::UnidirectionalAStar(
    const boost::property_tree::ptree& config);

// Clear the temporary information generated during path construction.
template <const ExpansionType expansion_direction, const bool FORWARD>
void UnidirectionalAStar<expansion_direction, FORWARD>::Clear() {
  // Clear the edge labels and destination list. Reset the adjacency list
  // and clear edge status.
  auto reservation = clear_reserved_memory_ ? 0 : max_reserved_labels_count_;
  if (edgelabels_.size() > reservation) {
    edgelabels_.resize(reservation);
    edgelabels_.shrink_to_fit();
  }
  edgelabels_.clear();
  destinations_percent_along_.clear();
  adjacencylist_.clear();
  edgestatus_.clear();

  // Set the ferry flag to false
  has_ferry_ = false;
}

template void UnidirectionalAStar<ExpansionType::forward>::Clear();
template void UnidirectionalAStar<ExpansionType::reverse>::Clear();

// Expand from the node along the forward search path. Immediately expands
// from the end node of any transition edge (so no transition edges are added
// to the adjacency list or EdgeLabel list). Does not expand transition
// edges if from_transition is false.
template <const ExpansionType expansion_direction, const bool FORWARD>
bool UnidirectionalAStar<expansion_direction, FORWARD>::Expand(GraphReader& graphreader,
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
  auto offset_time =
      FORWARD ? time_info.forward(pred.cost().secs, static_cast<int>(nodeinfo->timezone()))
              : time_info.reverse(pred.cost().secs, static_cast<int>(nodeinfo->timezone()));

  if (!costing_->Allowed(nodeinfo)) {
    const DirectedEdge* opp_edge;
    const GraphId opp_edge_id = graphreader.GetOpposingEdgeId(pred.edgeid(), opp_edge, tile);
    // Check if edge is null before using it (can happen with regional data sets)
    pred.set_deadend(true);
    return opp_edge && ExpandInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx,
                                   {opp_edge, opp_edge_id, edgestatus_.GetPtr(opp_edge_id, tile)},
                                   tile, offset_time, destination, best_path);
  }

  // Expand from <expansion_direction> node.
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
                     ExpandInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx, meta, tile,
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
        disable_uturn = ExpandInner(graphreader, pred, opp_pred_edge, trans_node, pred_idx,
                                    trans_meta, trans_tile, offset_time, destination, best_path) ||
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
    disable_uturn = ExpandInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx, uturn_meta,
                                tile, offset_time, destination, best_path) ||
                    disable_uturn;
  }

  return disable_uturn;
}

// Runs in the inner loop of `Expand`, essentially evaluating if
// the edge described in `meta` should be placed on the stack
// as well as doing just that.
//
// Returns true if any edge _could_ have been expanded after restrictions etc.
template <const ExpansionType expansion_direction, const bool FORWARD>
inline bool UnidirectionalAStar<expansion_direction, FORWARD>::ExpandInner(
    GraphReader& graphreader,
    const BDEdgeLabel& pred,
    const baldr::DirectedEdge* opp_pred_edge,
    const NodeInfo* nodeinfo,
    const uint32_t pred_idx,
    const EdgeMetadata& meta,
    const graph_tile_ptr& tile,
    const TimeInfo& time_info,
    const valhalla::Location& destination,
    std::pair<int32_t, float>& best_path) {

  // Skip shortcut edges for time dependent routes
  // TODO(danpat): why?
  if (meta.edge->is_shortcut()) {
    return false;
  }

  if (!FORWARD) {
    // Skip this edge if permanently labeled (best path already found to this directed edge) or if no
    // access for this mode.
    if (!(meta.edge->reverseaccess() & access_mode_)) {
      return false;
    }
  }

  // Skip this edge if permanently labeled (best path already found to this directed edge)
  if (meta.edge_status->set() == EdgeSet::kPermanent) {
    return true; // This is an edge we _could_ have expanded, so return true
  }

  graph_tile_ptr t2 = nullptr;
  GraphId opp_edge_id;
  const DirectedEdge* opp_edge = nullptr;
  if (!FORWARD) {
    // Get end node tile, opposing edge Id, and opposing directed edge.
    t2 = meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
    if (t2 == nullptr) {
      return false;
    }
    opp_edge_id = t2->GetOpposingEdgeId(meta.edge);
    opp_edge = t2->directededge(opp_edge_id);
  }

  /*
   * NOTE:
   * When bidirectional a* degenerates to unidirectional, in practice we probably
   * need to handle these same problems with the destination edge and
   * path_distance.. probably.. but did not check
   */
  // Skip shortcut edges for time dependent routes, if no access is allowed to this edge
  // (based on costing method)
  uint8_t restriction_idx = kInvalidRestriction;
  auto dest_edge = destinations_percent_along_.find(meta.edge_id);
  const bool is_dest = dest_edge != destinations_percent_along_.end();
  if (FORWARD) {
    if (!costing_->Allowed(meta.edge, is_dest, pred, tile, meta.edge_id, time_info.local_time,
                           nodeinfo->timezone(), restriction_idx) ||
        costing_->Restricted(meta.edge, pred, edgelabels_, tile, meta.edge_id, true, &edgestatus_,
                             time_info.local_time, nodeinfo->timezone())) {
      return false;
    }
  } else {
    if (!costing_->AllowedReverse(meta.edge, pred, opp_edge, t2, opp_edge_id, time_info.local_time,
                                  nodeinfo->timezone(), restriction_idx) ||
        costing_->Restricted(meta.edge, pred, edgelabels_, tile, meta.edge_id, false, &edgestatus_,
                             time_info.local_time, nodeinfo->timezone())) {
      return false;
    }
  }

  // Compute the cost to the end of this edge
  uint8_t flow_sources;
  auto edge_cost = FORWARD ? costing_->EdgeCost(meta.edge, tile, time_info, flow_sources)
                           : costing_->EdgeCost(opp_edge, t2, time_info, flow_sources);

  sif::Cost transition_cost =
      FORWARD ? costing_->TransitionCost(meta.edge, nodeinfo, pred)
              : costing_->TransitionCostReverse(meta.edge->localedgeidx(), nodeinfo, opp_edge,
                                                opp_pred_edge,
                                                static_cast<bool>(flow_sources & kDefaultFlowMask),
                                                pred.internal_turn());

  Cost newcost = pred.cost() + edge_cost;
  newcost += transition_cost;

  // If this edge is a destination, subtract the partial/remainder cost
  // (cost from the dest. location to the end of the edge).
  if (is_dest) {
    // Adapt cost to potentially not using the entire destination edge
    newcost -= edge_cost * (FORWARD ? (1.0f - dest_edge->second) : dest_edge->second);

    // Find the destination edge and update cost to include the edge score.
    // Note - with high edge scores the convergence test fails some routes
    // so reduce the edge score.
    for (const auto& destination_edge : destination.correlation().edges()) {
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
    // TODO(danpat): can we slices down to EdgeLabel here safely?
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
  if (!is_dest) {
    graph_tile_ptr t2 =
        meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
    if (t2 == nullptr) {
      return false;
    }
    sortcost += astarheuristic_.Get(t2->get_node_ll(meta.edge->endnode()), dist);
  }

  if (FORWARD) {
    // Add to the adjacency list and edge labels.
    uint32_t idx = edgelabels_.size();
    edgelabels_.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, newcost, sortcost, dist,
                             mode_, transition_cost,
                             (pred.not_thru_pruning() || !meta.edge->not_thru()),
                             (pred.closure_pruning() || !(costing_->IsClosed(meta.edge, tile))),
                             static_cast<bool>(flow_sources & kDefaultFlowMask),
                             costing_->TurnType(pred.opp_local_idx(), nodeinfo, meta.edge),
                             restriction_idx);
    // TODO: the BDEdgeLabel constructor doesnt have a way to set the path distance and the distance
    //  so we work around it by using the update function. in the future we could just make edge
    //  labels simple structs with public access
    auto path_distance = static_cast<uint32_t>(
        pred.path_distance() + meta.edge->length() * (is_dest ? dest_edge->second : 1.f) + .5f);
    edgelabels_.back().Update(pred_idx, newcost, sortcost, transition_cost, path_distance,
                              restriction_idx);
    *meta.edge_status = {EdgeSet::kTemporary, idx};
    adjacencylist_.add(idx);
  } else {
    // Add edge label, add to the adjacency list and set edge status
    uint32_t idx = edgelabels_.size();
    edgelabels_.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, newcost, sortcost, dist,
                             mode_, transition_cost,
                             (pred.not_thru_pruning() || !meta.edge->not_thru()),
                             (pred.closure_pruning() || !(costing_->IsClosed(meta.edge, tile))),
                             static_cast<bool>(flow_sources & kDefaultFlowMask),
                             costing_->TurnType(meta.edge->localedgeidx(), nodeinfo, opp_edge,
                                                opp_pred_edge),
                             restriction_idx);
    *meta.edge_status = {EdgeSet::kTemporary, idx};
    adjacencylist_.add(idx);
  }

  return true;
}

// Form the path from the adjacency list in the _forward_ direction
template <>
std::vector<PathInfo> UnidirectionalAStar<ExpansionType::forward>::FormPath(const uint32_t dest) {
  // Metrics to track
  LOG_DEBUG("path_cost::" + std::to_string(edgelabels_[dest].cost().cost));
  LOG_DEBUG("path_iterations::" + std::to_string(edgelabels_.size()));

  // Work backwards from the destination
  std::vector<PathInfo> path;
  for (auto edgelabel_index = dest; edgelabel_index != kInvalidLabel;
       edgelabel_index = edgelabels_[edgelabel_index].predecessor()) {
    const EdgeLabel& edgelabel = edgelabels_[edgelabel_index];
    path.emplace_back(edgelabel.mode(), edgelabel.cost(), edgelabel.edgeid(), 0,
                      edgelabel.path_distance(), edgelabel.restriction_idx(),
                      edgelabel.transition_cost());

    // Check if this is a ferry
    if (edgelabel.use() == Use::kFerry) {
      has_ferry_ = true;
    }
  }

  // Reverse the list and return
  std::reverse(path.begin(), path.end());
  return path;
}

// Form the path from the adjacency list in the _reverse_ direction
template <>
std::vector<PathInfo> UnidirectionalAStar<ExpansionType::reverse>::FormPath(const uint32_t dest) {
  // Metrics to track
  LOG_DEBUG("path_cost::" + std::to_string(edgelabels_[dest].cost().cost));
  LOG_DEBUG("path_iterations::" + std::to_string(edgelabels_.size()));

  // Form the reverse path from the destination (true origin) using opposing edges.
  std::vector<PathInfo> path;
  Cost cost, previous_transition_cost;
  uint32_t edgelabel_index = dest;
  while (edgelabel_index != kInvalidLabel) {
    const BDEdgeLabel& edgelabel = edgelabels_[edgelabel_index];

    // Get elapsed time on the edge, then add the transition cost at prior edge.
    uint32_t predidx = edgelabel.predecessor();
    if (predidx == kInvalidLabel) {
      cost += edgelabel.cost();
    } else {
      cost += edgelabel.cost() - edgelabels_[predidx].cost();
    }
    // PathInfo expects, looking forward along the route, the cost to be the transition
    // at the start of the edge plus the cost of the edge.  In the reverse search,
    // EdgeLabels contain the cost of the edge plus the transition at the _end_ of the edge,
    // looking forward.  Here, we subtract the transition at the end, and add the transition
    // at the start (which is taken from the previous edge, as we're walking from the origin
    // to the destination here)
    cost -= edgelabel.transition_cost();
    cost += previous_transition_cost;

    path.emplace_back(edgelabel.mode(), cost, edgelabel.opp_edgeid(), 0, edgelabel.path_distance(),
                      edgelabel.restriction_idx(), previous_transition_cost);

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

template <const ExpansionType expansion_direction, const bool FORWARD>
std::vector<std::vector<PathInfo>> UnidirectionalAStar<expansion_direction, FORWARD>::GetBestPath(
    valhalla::Location& origin,
    valhalla::Location& destination,
    GraphReader& graphreader,
    const sif::mode_costing_t& mode_costing,
    const travel_mode_t mode,
    const Options& /*options*/) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();
  access_mode_ = costing_->access_mode();

  if (!FORWARD) {
    // date_time must be set on the destination. Log an error but allow routes for now.
    if (destination.date_time().empty()) {
      LOG_ERROR("TimeDepReverse called without time set on the destination location");
      // return {};
    }
  }
  // Initialize - create adjacency list, edgestatus support, A*, etc.
  // Note: because we can correlate to more than one place for a given PathLocation
  // using edges.front here means we are only setting the heuristics to one of them
  // alternate paths using the other correlated points to may be harder to find
  midgard::PointLL origin_new(origin.correlation().edges(0).ll().lng(),
                              origin.correlation().edges(0).ll().lat());
  midgard::PointLL destination_new(destination.correlation().edges(0).ll().lng(),
                                   destination.correlation().edges(0).ll().lat());
  Init(origin_new, destination_new);
  float mindist = astarheuristic_.GetDistance(origin_new);

  auto& startpoint = FORWARD ? origin : destination;
  auto& endpoint = FORWARD ? destination : origin;

  // Get time information for forward
  auto time_info = TimeInfo::make(startpoint, graphreader, &tz_cache_);

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  uint32_t density = SetDestination(graphreader, endpoint);
  SetOrigin(graphreader, startpoint, endpoint, time_info);

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
    BDEdgeLabel pred = edgelabels_[predindex];
    auto maybe_dest = destinations_percent_along_.find(pred.edgeid());
    if (maybe_dest != destinations_percent_along_.end()) {
      // Check if a trivial path. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        if (IsTrivial(FORWARD ? pred.edgeid() : pred.opp_edgeid(), origin, destination)) {
          // so this is the the portion of the edge from the origin but doesnt account for how
          // far along the destination is along the edge so we need to subtract off the part after
          // the dest
          auto edge_length = graphreader.directededge(pred.edgeid())->length();
          auto path_distance = static_cast<uint32_t>(
              std::max(0.f, pred.path_distance() - (1.f - maybe_dest->second) * edge_length) + .5f);
          pred.Update(pred.predecessor(), pred.cost(), pred.sortcost(), pred.transition_cost(),
                      path_distance, pred.restriction_idx());
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

    // Get the opposing predecessor directed edge. Need to make sure we get
    // the correct one if a transition occurred
    const DirectedEdge* opp_pred_edge =
        FORWARD ? nullptr
                : graphreader.GetGraphTile(pred.opp_edgeid())->directededge(pred.opp_edgeid());

    // Expand forward from the end node of the predecessor edge.
    Expand(graphreader, pred.endnode(), pred, predindex, opp_pred_edge, time_info, destination,
           best_path);
  }
  return {}; // Should never get here
}

template std::vector<std::vector<PathInfo>>
UnidirectionalAStar<ExpansionType::forward>::GetBestPath(valhalla::Location& origin,
                                                         valhalla::Location& destination,
                                                         GraphReader& graphreader,
                                                         const sif::mode_costing_t& mode_costing,
                                                         const travel_mode_t mode,
                                                         const Options& /*options*/);
template std::vector<std::vector<PathInfo>>
UnidirectionalAStar<ExpansionType::reverse>::GetBestPath(valhalla::Location& origin,
                                                         valhalla::Location& destination,
                                                         GraphReader& graphreader,
                                                         const sif::mode_costing_t& mode_costing,
                                                         const travel_mode_t mode,
                                                         const Options& /*options*/);
// Set the mode and costing
// Initialize prior to finding best path
template <const ExpansionType expansion_direction, const bool FORWARD>
void UnidirectionalAStar<expansion_direction, FORWARD>::Init(const midgard::PointLL& origll,
                                                             const midgard::PointLL& destll) {

  float mincost = 0;
  if (FORWARD) {
    astarheuristic_.Init(destll, costing_->AStarCostFactor());
    mincost = astarheuristic_.Get(origll);
  } else {
    astarheuristic_.Init(origll, costing_->AStarCostFactor());
    mincost = astarheuristic_.Get(destll);
  }
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
template <const ExpansionType expansion_direction, const bool FORWARD>
void UnidirectionalAStar<expansion_direction, FORWARD>::ModifyHierarchyLimits(
    const float dist,
    const uint32_t /*density*/) {
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
template <const ExpansionType expansion_direction, const bool FORWARD>
void UnidirectionalAStar<expansion_direction, FORWARD>::SetOrigin(
    GraphReader& graphreader,
    const valhalla::Location& origin,
    const valhalla::Location& destination,
    const TimeInfo& time_info) {
  // Only skip inbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.correlation().edges().begin(), origin.correlation().edges().end(),
                [&has_other_edges](const valhalla::PathEdge& e) {
                  has_other_edges = has_other_edges || (FORWARD ? !e.end_node() : !e.begin_node());
                });

  /**
   * Consider the route from 1 to 2 on the following graph:
   *
   * 1          2
   * A----------B
   *
   * Here both locations get both directions of the edge AB, but it doesnt make sense to use all of
   * these candidates in practice. The edge candidate for the origin which are at 100% along the edge
   * amount to no distance traveled. Similarly the edge candidate for the destination at 0% along
   * would result in an edge with no distance traveled. In other words when its a node snap, you want
   * to use only those edge candidates that are leaving the origin and those that are arriving at the
   * destination. In addition to the node snapping, this route is trivial because it consists of one
   * edge.
   *
   * Now consider the route from 1 to 2 on the following graph:
   *            2
   * A----------B
   *            1
   * In this case the edge candidates for both locations are node snapped to B meaning both locations
   * get both directions of AB again but this time they are on the same node. We can call this route
   * super-trivial because its not just a single edge its actually a single node. In both SetOrigin
   * and in SetDestination we try to remove edge candidates that are superfluous as described above.
   * SetDestination happens first and trivially removes the edge candidate for 2 that travels from B
   * back to A. This means the destination now can only be reached via the edge traveling from A to B.
   * However that edge candidate of 1 is 100% along that edge and would be trivially removed, since
   * its a node snap that is arriving at the origin rather than leaving it. So here in SetOrigin we
   * add a bit more complicated logic to say that if this case occurs, allow that edge to be used
   */

  // its super trivial if both are node snapped to the same end of the same edge
  // note the check for node snapping is in the if below and not in this lambda
  auto super_trivial = [this](const valhalla::PathEdge& edge) {
    auto p = destinations_percent_along_.find(edge.graph_id());
    return p != destinations_percent_along_.end() && edge.percent_along() == p->second;
  };

  // Iterate through edges and add to adjacency list
  for (const auto& edge : origin.correlation().edges()) {
    // If this is a node snap and we have other candidates that we can skip this unless its the one we
    // need for super trivial route
    if ((FORWARD ? edge.end_node() : edge.begin_node()) && has_other_edges && !super_trivial(edge)) {
      continue;
    }

    GraphId edgeid(edge.graph_id());

    // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
    if (FORWARD) {
      if (costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
        continue;
      }
    }

    // Get the directed edge
    const auto tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the tile at the end node. Skip if tile not found as we won't be
    // able to expand from this origin edge.
    uint8_t flow_sources;
    Cost cost;
    float dist;
    GraphId opp_edge_id;
    const DirectedEdge* opp_dir_edge;
    if (FORWARD) {
      const auto endtile = graphreader.GetGraphTile(directededge->endnode());
      if (endtile == nullptr) {
        continue;
      }
      cost = costing_->EdgeCost(directededge, tile, time_info, flow_sources) *
             (1.0f - edge.percent_along());
      dist = astarheuristic_.GetDistance(endtile->get_node_ll(directededge->endnode()));
    } else {
      // Get the opposing directed edge, continue if we cannot get it
      opp_edge_id = graphreader.GetOpposingEdgeId(edgeid);
      if (!opp_edge_id.Is_Valid()) {
        continue;
      }
      opp_dir_edge = graphreader.GetOpposingEdge(edgeid);
      cost = costing_->EdgeCost(directededge, tile, time_info, flow_sources) * edge.percent_along();
      dist = astarheuristic_.GetDistance(tile->get_node_ll(opp_dir_edge->endnode()));
    }

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
    auto settled_dest_edge = FORWARD ? destinations_percent_along_.find(edgeid)
                                     : destinations_percent_along_.find(opp_edge_id);
    if (settled_dest_edge != destinations_percent_along_.end()) {
      bool trivial =
          FORWARD ? IsTrivial(edgeid, origin, destination) : IsTrivial(edgeid, destination, origin);
      if (trivial) {
        // Find the destination edge and update cost.
        for (const auto& dest_path_edge : destination.correlation().edges()) {
          if (dest_path_edge.graph_id() == edgeid) {
            // a trivial route passes along a single edge, meaning that the
            // destination point must be on this edge, and so the distance
            // remaining must be zero.
            GraphId id(dest_path_edge.graph_id());
            const DirectedEdge* dest_edge = tile->directededge(id);
            Cost remainder_cost = FORWARD
                                      ? costing_->EdgeCost(dest_edge, tile, time_info, flow_sources) *
                                            (1.0f - dest_path_edge.percent_along())
                                      : costing_->EdgeCost(dest_edge, tile, time_info, flow_sources) *
                                            (dest_path_edge.percent_along());
            // Remove the cost of the final "unused" part of the destination edge
            cost -= remainder_cost;
            // Add back in the edge score/penalty to account for destination edges
            // farther from the input location lat,lon.
            cost.cost += dest_path_edge.distance();
            cost.cost = std::max(0.0f, cost.cost);
            dist = 0.0;
            // Search complete if this is the forward search
            if (FORWARD)
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

    // Add EdgeLabel to the adjacency list
    uint32_t idx = edgelabels_.size();
    if (FORWARD) {
      uint32_t path_distance =
          static_cast<uint32_t>(directededge->length() * (1.0f - edge.percent_along()) + .5f);
      BDEdgeLabel edge_label(kInvalidLabel, edgeid, {}, directededge, cost, sortcost, dist, mode_,
                             Cost{}, false, !(costing_->IsClosed(directededge, tile)),
                             static_cast<bool>(flow_sources & kDefaultFlowMask),
                             sif::InternalTurn::kNoTurn, kInvalidRestriction);
      /* BDEdgeLabel doesn't have a constructor that allows you to set dist and path_distance at the
       * same time - so we need to update immediately after to set path_distance */
      edge_label.Update(kInvalidLabel, cost, sortcost, {}, path_distance, kInvalidRestriction);
      // Set the origin flag
      edgelabels_.emplace_back(std::move(edge_label));
    } else {
      edgelabels_.emplace_back(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, sortcost, dist,
                               mode_, Cost{}, false, !(costing_->IsClosed(directededge, tile)),
                               static_cast<bool>(flow_sources & kDefaultFlowMask),
                               sif::InternalTurn::kNoTurn, kInvalidRestriction);
      // Set the initial not_thru flag to false. There is an issue with not_thru
      // flags on small loops. Set this to false here to override this for now.
      edgelabels_.back().set_not_thru(false);
    }
    // Set the origin flag
    edgelabels_.back().set_origin();
    adjacencylist_.add(idx);

    // DO NOT SET EdgeStatus - it messes up trivial paths with oneways
  }
}

// Add a destination edge
template <const ExpansionType expansion_direction, const bool FORWARD>
uint32_t
UnidirectionalAStar<expansion_direction, FORWARD>::SetDestination(GraphReader& graphreader,
                                                                  const valhalla::Location& dest) {
  // Only skip outbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(dest.correlation().edges().begin(), dest.correlation().edges().end(),
                [&has_other_edges](const valhalla::PathEdge& e) {
                  has_other_edges = has_other_edges || !(FORWARD ? e.begin_node() : e.end_node());
                });

  // For each edge
  uint32_t density = 0;
  for (const auto& edge : dest.correlation().edges()) {
    // If destination is at a node skip any outbound edges
    if (has_other_edges && (FORWARD ? edge.begin_node() : edge.end_node())) {
      continue;
    }

    GraphId edgeid(edge.graph_id());
    graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
    if (tile == nullptr) {
      continue;
    }
    if (FORWARD) {
      // Disallow any user avoided edges if the avoid location is behind the destination along the
      // edge
      if (costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Keep the cost to traverse the partial distance for the remainder of the edge. This cost
      // is subtracted from the total cost up to the end of the destination edge.
      destinations_percent_along_[edge.graph_id()] = edge.percent_along();
    } else {
      // Keep the id and the cost to traverse the partial distance for the
      // remainder of the edge. This cost is subtracted from the total cost
      // up to the end of the destination edge.
      const DirectedEdge* directededge = tile->directededge(edgeid);

      // The opposing edge Id is added as a destination since the search
      // is done in reverse direction.
      auto t2 =
          directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (!t2) {
        continue;
      }
      GraphId oppedge = t2->GetOpposingEdgeId(directededge);
      destinations_percent_along_[oppedge] = edge.percent_along();
    }

    // Edge score (penalty) is handled within GetPath. Do not add score here.

    // Get the tile relative density
    density = tile->header()->density();
  }
  return density;
}

} // namespace thor
} // namespace valhalla
