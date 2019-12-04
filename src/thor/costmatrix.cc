#include <algorithm>
#include <cmath>
#include <vector>

#include "midgard/logging.h"
#include "thor/costmatrix.h"
#include "worker.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

constexpr uint32_t kMaxMatrixIterations = 2000000;

// Find a threshold to continue the search - should be based on
// the max edge cost in the adjacency set?
int GetThreshold(const TravelMode mode, const int n) {
  return (mode == TravelMode::kDrive) ? std::min(2700, std::max(100, n / 3)) : 500;
}

bool equals(const valhalla::LatLng& a, const valhalla::LatLng& b) {
  return a.has_lat() == b.has_lat() && a.has_lng() == b.has_lng() &&
         (!a.has_lat() || a.lat() == b.lat()) && (!a.has_lng() || a.lng() == b.lng());
}

} // namespace

namespace valhalla {
namespace thor {

// Constructor with cost threshold.
CostMatrix::CostMatrix()
    : mode_(TravelMode::kDrive), access_mode_(kAutoAccess), source_count_(0), remaining_sources_(0),
      target_count_(0), remaining_targets_(0), current_cost_threshold_(0) {
}

float CostMatrix::GetCostThreshold(const float max_matrix_distance) {
  float cost_threshold;
  switch (mode_) {
    case TravelMode::kBicycle:
      cost_threshold = max_matrix_distance / kCostThresholdBicycleDivisor;
      break;
    case TravelMode::kPedestrian:
    case TravelMode::kPublicTransit:
      cost_threshold = max_matrix_distance / kCostThresholdPedestrianDivisor;
      break;
    case TravelMode::kDrive:
    default:
      cost_threshold = max_matrix_distance / kCostThresholdAutoDivisor;
  }

  // Increase the cost threshold to make sure requests near the max distance succeed.
  // Some costing models and locations require higher thresholds to succeed.
  return cost_threshold * 2.0f;
}

// Clear the temporary information generated during time + distance matrix
// construction.
void CostMatrix::Clear() {
  // Clear the target edge markings
  targets_.clear();

  // Clear all source adjacency lists, edge labels, and edge status
  for (auto& adj : source_adjacency_) {
    adj.reset();
  }
  source_adjacency_.clear();

  for (auto& el : source_edgelabel_) {
    el.clear();
  }
  source_edgelabel_.clear();

  for (auto& es : source_edgestatus_) {
    es.clear();
  }
  source_edgestatus_.clear();

  // Clear all target adjacency lists, edge labels, and edge status
  for (auto& adj : target_adjacency_) {
    adj.reset();
  }
  target_adjacency_.clear();

  for (auto& el : target_edgelabel_) {
    el.clear();
  }
  target_edgelabel_.clear();

  for (auto& es : target_edgestatus_) {
    es.clear();
  }
  target_edgestatus_.clear();

  source_hierarchy_limits_.clear();
  target_hierarchy_limits_.clear();
  source_status_.clear();
  target_status_.clear();
}

// Form a time distance matrix from the set of source locations
// to the set of target locations.
std::vector<TimeDistance> CostMatrix::SourceToTarget(
    const google::protobuf::RepeatedPtrField<valhalla::Location>& source_location_list,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& target_location_list,
    GraphReader& graphreader,
    const std::shared_ptr<DynamicCost>* mode_costing,
    const TravelMode mode,
    const float max_matrix_distance) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  access_mode_ = costing_->access_mode();

  current_cost_threshold_ = GetCostThreshold(max_matrix_distance);

  // Set the source and target locations
  Clear();
  SetSources(graphreader, source_location_list);
  SetTargets(graphreader, target_location_list);

  // Initialize best connections and status. Any locations that are the
  // same get set to 0 time, distance and are not added to the remaining
  // location set.
  Initialize(source_location_list, target_location_list);

  // Perform backward search from all target locations. Perform forward
  // search from all source locations. Connections between the 2 search
  // spaces is checked during the forward search.
  int n = 0;
  while (true) {
    // Iterate all target locations in a backwards search
    for (uint32_t i = 0; i < target_count_; i++) {
      if (target_status_[i].threshold > 0) {
        target_status_[i].threshold--;
        BackwardSearch(i, graphreader);
        if (target_status_[i].threshold == 0) {
          target_status_[i].threshold = -1;
          if (remaining_targets_ > 0) {
            remaining_targets_--;
          }
        }
      }
    }

    // Iterate all source locations in a forward search
    for (uint32_t i = 0; i < source_count_; i++) {
      if (source_status_[i].threshold > 0) {
        source_status_[i].threshold--;
        ForwardSearch(i, n, graphreader);
        if (source_status_[i].threshold == 0) {
          source_status_[i].threshold = -1;
          if (remaining_sources_ > 0) {
            remaining_sources_--;
          }
        }
      }
    }

    // Break out when remaining sources and targets to expand are both 0
    if (remaining_sources_ == 0 && remaining_targets_ == 0) {
      LOG_DEBUG("SourceToTarget iterations: n = " + std::to_string(n));
      break;
    }

    // Protect against edge cases that may lead to never breaking out of
    // this loop. This should never occur but lets make sure.
    if (n >= kMaxMatrixIterations) {
      throw valhalla_exception_t{430};
    }
    n++;
  }

  // Form the time, distance matrix from the destinations list
  uint32_t idx = 0;
  std::vector<TimeDistance> td;
  for (const auto& connection : best_connection_) {
    td.emplace_back(std::round(connection.cost.secs), std::round(connection.distance));
    idx++;
  }
  return td;
}

// Initialize all time distance to "not found". Any locations that
// are the same get set to 0 time, distance and do not add to the
// remaining locations set.
void CostMatrix::Initialize(
    const google::protobuf::RepeatedPtrField<valhalla::Location>& source_locations,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& target_locations) {
  // Add initial status
  const uint32_t kMaxThreshold = std::numeric_limits<int>::max();
  for (uint32_t i = 0; i < source_count_; i++) {
    source_status_.emplace_back(kMaxThreshold);
  }
  for (uint32_t i = 0; i < target_count_; i++) {
    target_status_.emplace_back(kMaxThreshold);
  }

  // Initialize best connection
  bool all_the_same = true;
  GraphId empty;
  Cost trivial_cost(0.0f, 0.0f);
  Cost max_cost(kMaxCost, kMaxCost);
  for (uint32_t i = 0; i < source_count_; i++) {
    for (uint32_t j = 0; j < target_count_; j++) {
      if (equals(source_locations.Get(i).ll(), target_locations.Get(j).ll())) {
        best_connection_.emplace_back(empty, empty, trivial_cost, 0.0f);
        best_connection_.back().found = true;
      } else {
        best_connection_.emplace_back(empty, empty, max_cost, kMaxCost);
        source_status_[i].remaining_locations.insert(j);
        target_status_[j].remaining_locations.insert(i);
        all_the_same = false;
      }
    }
  }

  // Set the remaining number of sources and targets
  remaining_sources_ = 0;
  for (const auto& s : source_status_) {
    if (!s.remaining_locations.empty()) {
      remaining_sources_++;
    }
  }
  remaining_targets_ = 0;
  for (const auto& t : target_status_) {
    if (!t.remaining_locations.empty()) {
      remaining_targets_++;
    }
  }
}

// Iterate the forward search from the source/origin location.
void CostMatrix::ForwardSearch(const uint32_t index, const uint32_t n, GraphReader& graphreader) {
  // Get the next edge from the adjacency list for this source location
  auto& adj = source_adjacency_[index];
  auto& edgelabels = source_edgelabel_[index];
  uint32_t pred_idx = adj->pop();
  if (pred_idx == kInvalidLabel) {
    // Forward search is exhausted - mark this and update so we don't
    // extend searches more than we need to
    for (uint32_t target = 0; target < target_count_; target++) {
      UpdateStatus(index, target);
    }
    source_status_[index].threshold = 0;
    return;
  }

  // Get edge label and check cost threshold
  BDEdgeLabel pred = edgelabels[pred_idx];
  if (pred.cost().secs > current_cost_threshold_) {
    source_status_[index].threshold = 0;
    return;
  }

  // Settle this edge
  auto& edgestate = source_edgestatus_[index];
  edgestate.Update(pred.edgeid(), EdgeSet::kPermanent);

  // Check for connections to backwards search.
  CheckForwardConnections(index, pred, n);

  // Prune path if predecessor is not a through edge
  if (pred.not_thru() && pred.not_thru_pruning()) {
    return;
  }

  // Get the end node of the prior directed edge. Do not expand on this
  // hierarchy level if the maximum number of upward transitions has
  // been exceeded.
  GraphId node = pred.endnode();
  auto& hierarchy_limits = source_hierarchy_limits_[index];
  if (hierarchy_limits[node.level()].StopExpanding()) {
    return;
  }

  // lambda to expand search forward from the end node
  std::function<void(const GraphTile*, const GraphId&, const NodeInfo*, BDEdgeLabel&, const uint32_t,
                     const bool)>
      expand;
  expand = [&](const GraphTile* tile, const GraphId& node, const NodeInfo* nodeinfo,
               BDEdgeLabel& pred, const uint32_t pred_idx, const bool from_transition) {
    uint32_t shortcuts = 0;
    GraphId edgeid = {node.tileid(), node.level(), nodeinfo->edge_index()};
    EdgeStatusInfo* es = edgestate.GetPtr(edgeid, tile);
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, ++edgeid, ++es) {
      // Skip shortcut edges until we have stopped expanding on the next level. Use regular
      // edges while still expanding on the next level since we can still transition down to
      // that level. If using a shortcut, set the shortcuts mask. Skip if this is a regular
      // edge superseded by a shortcut.
      if (directededge->is_shortcut()) {
        if (hierarchy_limits[edgeid.level() + 1].StopExpanding()) {
          shortcuts |= directededge->shortcut();
        } else {
          continue;
        }
      } else if (shortcuts & directededge->superseded()) {
        continue;
      }

      // Skip this edge if permanently labeled (best path already found to this
      // directed edge) or if no access for this mode.
      if (es->set() == EdgeSet::kPermanent || !(directededge->forwardaccess() & access_mode_)) {
        continue;
      }

      // Skip this edge if no access is allowed (based on costing method)
      // or if a complex restriction prevents transition onto this edge.
      bool has_time_restrictions = false;
      if (!costing_->Allowed(directededge, pred, tile, edgeid, 0, 0, has_time_restrictions) ||
          costing_->Restricted(directededge, pred, edgelabels, tile, edgeid, true)) {
        continue;
      }

      // Get cost. Separate out transition cost.
      Cost tc = costing_->TransitionCost(directededge, nodeinfo, pred);
      Cost newcost = pred.cost() + tc + costing_->EdgeCost(directededge, tile);

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated along with new cost and distance.
      if (es->set() == EdgeSet::kTemporary) {
        BDEdgeLabel& lab = edgelabels[es->index()];
        if (newcost.cost < lab.cost().cost) {
          adj->decrease(es->index(), newcost.cost);
          lab.Update(pred_idx, newcost, newcost.cost, tc,
                     pred.path_distance() + directededge->length(), has_time_restrictions);
        }
        continue;
      }

      // Get end node tile (skip if tile is not found) and opposing edge Id
      const GraphTile* t2 =
          directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (t2 == nullptr) {
        continue;
      }
      GraphId oppedge = t2->GetOpposingEdgeId(directededge);

      // Add edge label, add to the adjacency list and set edge status
      uint32_t idx = edgelabels.size();
      *es = {EdgeSet::kTemporary, idx};
      edgelabels.emplace_back(pred_idx, edgeid, oppedge, directededge, newcost, mode_, tc,
                              pred.path_distance() + directededge->length(),
                              (pred.not_thru_pruning() || !directededge->not_thru()),
                              has_time_restrictions);
      adj->add(idx);
    }

    // Handle transitions - expand from the end node of the transition
    if (!from_transition && nodeinfo->transition_count() > 0) {
      const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
      for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
        if (trans->up()) {
          hierarchy_limits[node.level()].up_transition_count++;
        } else if (hierarchy_limits[trans->endnode().level()].StopExpanding()) {
          continue;
        }

        // Expand from end node of this transition.
        GraphId node = trans->endnode();
        const GraphTile* endtile = graphreader.GetGraphTile(node);
        if (endtile != nullptr) {
          expand(endtile, node, endtile->node(node), pred, pred_idx, true);
        }
      }
    }
  };

  // Expand from node in forward search path. Get the tile and the node info.
  // Skip if tile is null (can happen with regional data sets) or if no access
  // at the node.
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile != nullptr) {
    const NodeInfo* nodeinfo = tile->node(node);
    if (costing_->Allowed(nodeinfo)) {
      expand(tile, node, nodeinfo, pred, pred_idx, false);
    }
  }
}

// Check if the edge on the forward search connects to a reached edge
// on the reverse search trees.
void CostMatrix::CheckForwardConnections(const uint32_t source,
                                         const BDEdgeLabel& pred,
                                         const uint32_t n) {
  // Disallow connections that are part of a complex restriction.
  // TODO - validate that we do not need to "walk" the paths forward
  // and backward to see if they match a restriction.
  if (pred.on_complex_rest()) {
    return;
  }

  // Get the opposing edge. Get a list of target locations whose reverse
  // search has reached this edge.
  GraphId oppedge = pred.opp_edgeid();
  auto targets = targets_.find(oppedge);
  if (targets == targets_.end()) {
    return;
  }

  // Iterate through the targets
  for (auto target : targets->second) {
    uint32_t idx = source * target_count_ + target;
    if (best_connection_[idx].found) {
      continue;
    }

    // Update any targets whose threshold has been reached
    if (best_connection_[idx].threshold > 0 && n > best_connection_[idx].threshold) {
      best_connection_[idx].found = true;
      continue;
    }

    const auto& edgestate = target_edgestatus_[target];

    // If this edge has been reached then a shortest path has been found
    // to the end node of this directed edge.
    EdgeStatusInfo oppedgestatus = edgestate.Get(oppedge);
    if (oppedgestatus.set() != EdgeSet::kUnreached) {
      const auto& edgelabels = target_edgelabel_[target];
      uint32_t predidx = edgelabels[oppedgestatus.index()].predecessor();
      const BDEdgeLabel& opp_el = edgelabels[oppedgestatus.index()];

      // Special case - common edge for source and target are both initial edges
      if (pred.predecessor() == kInvalidLabel && predidx == kInvalidLabel) {
        float s = std::abs(pred.cost().secs + opp_el.cost().secs - opp_el.transition_cost());

        // Update best connection and set found = true.
        // distance computation only works with the casts.
        uint32_t d = std::abs(static_cast<int>(pred.path_distance()) +
                              static_cast<int>(opp_el.path_distance()) -
                              static_cast<int>(opp_el.transition_secs()));
        best_connection_[idx].Update(pred.edgeid(), oppedge, Cost(s, s), d);
        best_connection_[idx].found = true;

        // Update status and update threshold if this is the last location
        // to find for this source or target
        UpdateStatus(source, target);
      } else {
        float oppcost = (predidx == kInvalidLabel) ? 0 : edgelabels[predidx].cost().cost;
        float c = pred.cost().cost + oppcost + opp_el.transition_cost();

        // Check if best connection
        if (c < best_connection_[idx].cost.cost) {
          float oppsec = (predidx == kInvalidLabel) ? 0 : edgelabels[predidx].cost().secs;
          uint32_t oppdist = (predidx == kInvalidLabel) ? 0 : edgelabels[predidx].path_distance();
          float s = pred.cost().secs + oppsec + opp_el.transition_secs();
          uint32_t d = pred.path_distance() + oppdist;

          // Update best connection and set a threshold
          best_connection_[idx].Update(pred.edgeid(), oppedge, Cost(c, s), d);
          if (best_connection_[idx].threshold == 0) {
            best_connection_[idx].threshold =
                n + GetThreshold(mode_,
                                 source_edgelabel_[source].size() + target_edgelabel_[target].size());
          }

          // Update status and update threshold if this is the last location
          // to find for this source or target
          UpdateStatus(source, target);
        }
      }
    }
  }
}

// Update status when a connection is found.
void CostMatrix::UpdateStatus(const uint32_t source, const uint32_t target) {
  // Remove the target from the source status
  auto& s = source_status_[source].remaining_locations;
  auto it = s.find(target);
  if (it != s.end()) {
    s.erase(it);
    if (s.empty() && source_status_[source].threshold > 0) {
      // At least 1 connection has been found to each target for this source.
      // Set a threshold to continue search for a limited number of times.
      source_status_[source].threshold =
          GetThreshold(mode_, source_edgelabel_[source].size() + target_edgelabel_[target].size());
    }
  }

  // Remove the source from the target status
  auto& t = target_status_[target].remaining_locations;
  it = t.find(source);
  if (it != t.end()) {
    t.erase(it);
    if (t.empty() && target_status_[target].threshold > 0) {
      // At least 1 connection has been found to each source for this target.
      // Set a threshold to continue search for a limited number of times.
      target_status_[target].threshold =
          GetThreshold(mode_, source_edgelabel_[source].size() + target_edgelabel_[target].size());
    }
  }
}

// Expand the backwards search trees.
void CostMatrix::BackwardSearch(const uint32_t index, GraphReader& graphreader) {
  // Get the next edge from the adjacency list for this target location
  auto& adj = target_adjacency_[index];
  auto& edgelabels = target_edgelabel_[index];
  uint32_t pred_idx = adj->pop();
  if (pred_idx == kInvalidLabel) {
    // Backward search is exhausted - mark this and update so we don't
    // extend searches more than we need to
    for (uint32_t source = 0; source < source_count_; source++) {
      UpdateStatus(source, index);
    }
    target_status_[index].threshold = 0;
    return;
  }

  // Copy predecessor, check cost threshold
  BDEdgeLabel pred = edgelabels[pred_idx];
  if (pred.cost().secs > current_cost_threshold_) {
    target_status_[index].threshold = 0;
    return;
  }

  // Settle this edge
  auto& edgestate = target_edgestatus_[index];
  edgestate.Update(pred.edgeid(), EdgeSet::kPermanent);

  // Prune path if predecessor is not a through edge
  if (pred.not_thru() && pred.not_thru_pruning()) {
    return;
  }

  // Get the end node of the prior directed edge. Do not expand on this
  // hierarchy level if the maximum number of upward transitions has
  // been exceeded.
  GraphId node = pred.endnode();
  auto& hierarchy_limits = target_hierarchy_limits_[index];
  if (hierarchy_limits[node.level()].StopExpanding()) {
    return;
  }

  // Expand from node in reverse direction.
  std::function<void(const GraphTile*, const GraphId&, const NodeInfo*, const uint32_t, BDEdgeLabel&,
                     const uint32_t, const DirectedEdge*, const bool)>
      expand;
  expand = [&](const GraphTile* tile, const GraphId& node, const NodeInfo* nodeinfo,
               const uint32_t index, BDEdgeLabel& pred, const uint32_t pred_idx,
               const DirectedEdge* opp_pred_edge, const bool from_transition) {
    uint32_t shortcuts = 0;
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    EdgeStatusInfo* es = edgestate.GetPtr(edgeid, tile);
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, ++edgeid, ++es) {
      // Skip shortcut edges until we have stopped expanding on the next level. Use regular
      // edges while still expanding on the next level since we can still transition down to
      // that level. If using a shortcut, set the shortcuts mask. Skip if this is a regular
      // edge superseded by a shortcut.
      if (directededge->is_shortcut()) {
        if (hierarchy_limits[edgeid.level() + 1].StopExpanding()) {
          shortcuts |= directededge->shortcut();
        } else {
          continue;
        }
      } else if (shortcuts & directededge->superseded()) {
        continue;
      }

      // Skip edges not allowed by the access mode. Do this here to avoid having
      // to get opposing edge. Also skip edges that are permanently labeled (
      // best path already found to this directed edge).
      if (!(directededge->reverseaccess() & access_mode_) || es->set() == EdgeSet::kPermanent) {
        continue;
      }

      // Get opposing edge Id and end node tile
      const GraphTile* t2 =
          directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (t2 == nullptr) {
        continue;
      }
      GraphId oppedge = t2->GetOpposingEdgeId(directededge);

      // Skip this edge if no access is allowed (based on costing method)
      // or if a complex restriction prevents transition onto this edge.
      const DirectedEdge* opp_edge = t2->directededge(oppedge);
      bool has_time_restrictions = false;
      if (!costing_->AllowedReverse(directededge, pred, opp_edge, t2, oppedge, 0, 0,
                                    has_time_restrictions) ||
          costing_->Restricted(directededge, pred, edgelabels, tile, edgeid, false)) {
        continue;
      }

      // Get cost. Use opposing edge for EdgeCost. Separate the transition seconds so
      // we can properly recover elapsed time on the reverse path.
      Cost tc = costing_->TransitionCostReverse(directededge->localedgeidx(), nodeinfo, opp_edge,
                                                opp_pred_edge);
      Cost newcost = pred.cost() + tc + costing_->EdgeCost(opp_edge, tile);

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated along with new cost and distance.
      if (es->set() == EdgeSet::kTemporary) {
        BDEdgeLabel& lab = edgelabels[es->index()];
        if (newcost.cost < lab.cost().cost) {
          adj->decrease(es->index(), newcost.cost);
          lab.Update(pred_idx, newcost, newcost.cost, tc,
                     pred.path_distance() + directededge->length(), has_time_restrictions);
        }
        continue;
      }

      // Add edge label, add to the adjacency list and set edge status
      uint32_t idx = edgelabels.size();
      *es = {EdgeSet::kTemporary, idx};
      edgelabels.emplace_back(pred_idx, edgeid, oppedge, directededge, newcost, mode_, tc,
                              pred.path_distance() + directededge->length(),
                              (pred.not_thru_pruning() || !directededge->not_thru()),
                              has_time_restrictions);
      adj->add(idx);

      // Add to the list of targets that have reached this edge
      targets_[edgeid].push_back(index);
    }

    // Handle transitions - expand from the end node of the transition
    if (!from_transition && nodeinfo->transition_count() > 0) {
      const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
      for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
        if (trans->up()) {
          hierarchy_limits[node.level()].up_transition_count++;
        } else if (hierarchy_limits[trans->endnode().level()].StopExpanding()) {
          continue;
        }

        // Expand from end node of this transition edge.
        GraphId node = trans->endnode();
        const GraphTile* endtile = graphreader.GetGraphTile(node);
        if (endtile != nullptr) {
          expand(endtile, node, endtile->node(node), index, pred, pred_idx, opp_pred_edge, true);
        }
        continue;
      }
    }
  };

  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile != nullptr) {
    const NodeInfo* nodeinfo = tile->node(node);
    if (costing_->Allowed(nodeinfo)) {
      // Get the opposing predecessor directed edge. Need to make sure we get
      // the correct one if a transition occurred
      const DirectedEdge* opp_pred_edge;
      if (pred.opp_edgeid().Tile_Base() == tile->id().Tile_Base()) {
        opp_pred_edge = tile->directededge(pred.opp_edgeid().id());
      } else {
        opp_pred_edge =
            graphreader.GetGraphTile(pred.opp_edgeid().Tile_Base())->directededge(pred.opp_edgeid());
      }
      expand(tile, node, nodeinfo, index, pred, pred_idx, opp_pred_edge, false);
    }
  }
}

// Sets the source/origin locations. Search expands forward from these
// locations.
void CostMatrix::SetSources(GraphReader& graphreader,
                            const google::protobuf::RepeatedPtrField<valhalla::Location>& sources) {
  // Allocate edge labels and edge status
  source_count_ = sources.size();
  source_edgelabel_.resize(source_count_);
  source_edgestatus_.resize(source_count_);
  source_adjacency_.resize(source_count_);
  source_hierarchy_limits_.resize(source_count_);

  // Go through each source location
  uint32_t index = 0;
  Cost empty_cost;
  for (const auto& origin : sources) {
    // Set up lambda to get sort costs
    const auto edgecost = [this, index](const uint32_t label) -> float {
      return source_edgelabel_[index][label].sortcost();
    };

    // Allocate the adjacency list and hierarchy limits for this source.
    // Use the cost threshold to size the adjacency list.
    source_adjacency_[index].reset(
        new DoubleBucketQueue(0, current_cost_threshold_, costing_->UnitSize(), edgecost));
    source_hierarchy_limits_[index] = costing_->GetHierarchyLimits();

    // Iterate through edges and add to adjacency list
    for (const auto& edge : origin.path_edges()) {
      // If origin is at a node - skip any inbound edge (dist = 1)
      if (edge.end_node()) {
        continue;
      }

      // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
      GraphId edgeid(edge.graph_id());
      if (costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Get the directed edge and the opposing edge Id
      const GraphTile* tile = graphreader.GetGraphTile(edgeid);
      const DirectedEdge* directededge = tile->directededge(edgeid);
      GraphId oppedge = graphreader.GetOpposingEdgeId(edgeid);

      // Get cost. Get distance along the remainder of this edge.
      Cost edgecost = costing_->EdgeCost(directededge, tile);
      Cost cost = edgecost * (1.0f - edge.percent_along());
      uint32_t d = std::round(directededge->length() * (1.0f - edge.percent_along()));

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
      cost.cost += edge.distance();

      // Store the edge cost and length in the transition cost (so we can
      // recover the full length and cost for cases where origin and
      // destination are on the same edge
      Cost ec(std::round(edgecost.secs), static_cast<uint32_t>(directededge->length()));

      // Set the initial not_thru flag to false. There is an issue with not_thru
      // flags on small loops. Set this to false here to override this for now.
      BDEdgeLabel edge_label(kInvalidLabel, edgeid, oppedge, directededge, cost, mode_, ec, d, false,
                             false);
      edge_label.set_not_thru(false);

      // Add EdgeLabel to the adjacency list (but do not set its status).
      // Set the predecessor edge index to invalid to indicate the origin
      // of the path.
      uint32_t idx = source_edgelabel_[index].size();
      source_edgelabel_[index].push_back(std::move(edge_label));
      source_adjacency_[index]->add(idx);
      source_edgestatus_[index].Set(edgeid, EdgeSet::kUnreached, idx, tile);
    }
    index++;
  }
}

// Set the target/destination locations. Search expands backwards from
// these locations.
void CostMatrix::SetTargets(baldr::GraphReader& graphreader,
                            const google::protobuf::RepeatedPtrField<valhalla::Location>& targets) {
  // Allocate target edge labels and edge status
  target_count_ = targets.size();
  target_edgelabel_.resize(targets.size());
  target_edgestatus_.resize(targets.size());
  target_adjacency_.resize(targets.size());
  target_hierarchy_limits_.resize(targets.size());

  // Go through each target location
  uint32_t index = 0;
  Cost empty_cost;
  for (const auto& dest : targets) {
    // Set up lambda to get sort costs
    const auto edgecost = [this, index](const uint32_t label) {
      return target_edgelabel_[index][label].sortcost();
    };

    // Allocate the adjacency list and hierarchy limits for target location.
    // Use the cost threshold to size the adjacency list.
    target_adjacency_[index].reset(
        new DoubleBucketQueue(0, current_cost_threshold_, costing_->UnitSize(), edgecost));
    target_hierarchy_limits_[index] = costing_->GetHierarchyLimits();

    // Iterate through edges and add to adjacency list
    for (const auto& edge : dest.path_edges()) {
      // If the destination is at a node, skip any outbound edges (so any
      // opposing inbound edges are not considered)
      if (edge.begin_node()) {
        continue;
      }

      // Disallow any user avoided edges if the avoid location is behind the destination along the
      // edge
      GraphId edgeid(edge.graph_id());
      if (costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Get the directed edge
      const GraphTile* tile = graphreader.GetGraphTile(edgeid);
      const DirectedEdge* directededge = tile->directededge(edgeid);

      // Get the opposing directed edge, continue if we cannot get it
      GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid);
      if (!opp_edge_id.Is_Valid()) {
        continue;
      }
      const DirectedEdge* opp_dir_edge = graphreader.GetOpposingEdge(edgeid);

      // Get cost. Get distance along the remainder of this edge.
      // Use the directed edge for costing, as this is the forward direction
      // along the destination edge.
      Cost edgecost = costing_->EdgeCost(directededge, tile);
      Cost cost = edgecost * edge.percent_along();
      uint32_t d = std::round(directededge->length() * edge.percent_along());

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
      cost.cost += edge.distance();

      // Store the edge cost and length in the transition cost (so we can
      // recover the full length and cost for cases where origin and
      // destination are on the same edge
      Cost ec(std::round(edgecost.secs), static_cast<uint32_t>(directededge->length()));

      // Set the initial not_thru flag to false. There is an issue with not_thru
      // flags on small loops. Set this to false here to override this for now.
      BDEdgeLabel edge_label(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, mode_, ec, d,
                             false, false);
      edge_label.set_not_thru(false);

      // Add EdgeLabel to the adjacency list (but do not set its status).
      // Set the predecessor edge index to invalid to indicate the origin
      // of the path. Set the origin flag
      uint32_t idx = target_edgelabel_[index].size();
      target_edgelabel_[index].push_back(std::move(edge_label));
      target_adjacency_[index]->add(idx);
      target_edgestatus_[index].Set(opp_edge_id, EdgeSet::kUnreached, idx,
                                    graphreader.GetGraphTile(opp_edge_id));
      targets_[opp_edge_id].push_back(index);
    }
    index++;
  }
}

} // namespace thor
} // namespace valhalla
