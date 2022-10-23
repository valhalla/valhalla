#include "thor/timedistancebssmatrix.h"
#include "midgard/logging.h"
#include <algorithm>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {
static bool IsTrivial(const uint64_t& edgeid,
                      const valhalla::Location& origin,
                      const valhalla::Location& destination) {
  for (const auto& destination_edge : destination.correlation().edges()) {
    if (destination_edge.graph_id() == edgeid) {
      for (const auto& origin_edge : origin.correlation().edges()) {
        if (origin_edge.graph_id() == edgeid &&
            origin_edge.percent_along() <= destination_edge.percent_along()) {
          return true;
        }
      }
    }
  }
  return false;
}

static travel_mode_t get_other_travel_mode(const travel_mode_t current_mode) {
  static const auto bss_modes =
      std::vector<travel_mode_t>{travel_mode_t::kPedestrian, travel_mode_t::kBicycle};
  return bss_modes[static_cast<size_t>(current_mode == travel_mode_t::kPedestrian)];
}

} // namespace
namespace valhalla {
namespace thor {

// Constructor with cost threshold.
TimeDistanceBSSMatrix::TimeDistanceBSSMatrix() : settled_count_(0), current_cost_threshold_(0) {
}

float TimeDistanceBSSMatrix::GetCostThreshold(const float max_matrix_distance) const {
  // The threshold should be the time consumed by the pedestrian, because in the worst case,
  // the route may be pure pedestrian. Use a conservative walking speed of 2 MPH.
  return max_matrix_distance / (2.0f * kMPHtoMetersPerSec);
}

// Clear the temporary information generated during time + distance matrix
// construction.
void TimeDistanceBSSMatrix::clear() {
  // Clear the edge labels and destination list
  edgelabels_.clear();
  destinations_.clear();
  dest_edges_.clear();

  // Clear elements from the adjacency list
  adjacencylist_.clear();

  // Clear the edge status flags
  pedestrian_edgestatus_.clear();
  bicycle_edgestatus_.clear();
}

// Expand from a node in the forward direction
template <const ExpansionType expansion_direction, const bool FORWARD>
void TimeDistanceBSSMatrix::Expand(GraphReader& graphreader,
                                   const GraphId& node,
                                   const EdgeLabel& pred,
                                   const uint32_t pred_idx,
                                   const bool from_transition,
                                   const bool from_bss,
                                   const sif::travel_mode_t mode) {

  const auto& current_costing =
      (mode == travel_mode_t::kPedestrian ? pedestrian_costing_ : bicycle_costing_);

  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  graph_tile_ptr tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }
  const NodeInfo* nodeinfo = tile->node(node);
  if (!current_costing->Allowed(nodeinfo)) {
    return;
  }

  const DirectedEdge* opp_pred_edge = nullptr;
  if (!FORWARD) {
    opp_pred_edge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, opp_pred_edge++) {
      if (opp_pred_edge->localedgeidx() == pred.opp_local_idx()) {
        break;
      }
    }
  }

  // Expand from end node.
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  EdgeStatusInfo* es =
      (mode == travel_mode_t::kPedestrian ? pedestrian_edgestatus_ : bicycle_edgestatus_)
          .GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());

  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, ++edgeid, ++es) {
    // Skip shortcut edges
    if (directededge->is_shortcut() || es->set() == EdgeSet::kPermanent) {
      continue;
    }

    graph_tile_ptr t2 = nullptr;
    GraphId opp_edge_id;
    const DirectedEdge* opp_edge = nullptr;
    if (!FORWARD) {
      // Get opposing edge Id and end node tile
      graph_tile_ptr t2 =
          directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (t2 == nullptr) {
        continue;
      }
      GraphId opp_edge_id = t2->GetOpposingEdgeId(directededge);
      opp_edge = t2->directededge(opp_edge_id);
    }

    // Skip this edge if permanently labeled (best path already found to this
    // directed edge), if no access is allowed to this edge (based on costing
    // method), or if a complex restriction prevents this path.
    uint8_t restriction_idx = -1;
    const bool is_dest = dest_edges_.find(edgeid.value) != dest_edges_.cend();
    if (FORWARD) {
      if (!current_costing->Allowed(directededge, is_dest, pred, tile, edgeid, 0, 0,
                                    restriction_idx) ||
          current_costing->Restricted(directededge, pred, edgelabels_, tile, edgeid, true)) {
        continue;
      }
    } else {
      if (!current_costing->AllowedReverse(directededge, pred, opp_edge, t2, opp_edge_id, 0, 0,
                                           restriction_idx) ||
          (current_costing->Restricted(directededge, pred, edgelabels_, tile, edgeid, FORWARD))) {
        continue;
      }
    }

    // Get cost and update distance
    auto edge_cost = FORWARD ? current_costing->EdgeCost(directededge, tile)
                             : current_costing->EdgeCost(opp_edge, t2);
    auto transition_cost =
        FORWARD ? current_costing->TransitionCost(directededge, nodeinfo, pred)
                : current_costing->TransitionCostReverse(directededge->localedgeidx(), nodeinfo,
                                                         opp_edge, opp_pred_edge);

    Cost normalized_edge_cost = {edge_cost.cost * current_costing->GetModeFactor(), edge_cost.secs};
    // Compute the cost to the end of this edge
    Cost newcost = pred.cost() + normalized_edge_cost + transition_cost;

    uint32_t distance = pred.path_distance() + directededge->length();

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (es->set() == EdgeSet::kTemporary) {
      EdgeLabel& lab = edgelabels_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_.decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, transition_cost, restriction_idx);
      }
      continue;
    }

    // Add to the adjacency list and edge labels.
    uint32_t idx = edgelabels_.size();
    edgelabels_.emplace_back(pred_idx, edgeid, directededge, newcost, newcost.cost, 0.0f, mode,
                             distance, transition_cost, restriction_idx, true, false,
                             InternalTurn::kNoTurn);
    *es = {EdgeSet::kTemporary, idx};
    adjacencylist_.add(idx);
  }

  if (!from_bss && nodeinfo->type() == NodeType::kBikeShare) {
    auto other_mode = get_other_travel_mode(pred.mode());
    auto from_bss = true;
    Expand<expansion_direction>(graphreader, node, pred, pred_idx, from_transition, from_bss,
                                other_mode);
  }
  // Handle transitions - expand from the end node of each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      Expand<expansion_direction>(graphreader, trans->endnode(), pred, pred_idx, true, from_bss,
                                  mode);
    }
  }
}

// Calculate time and distance from one origin location to many destination
// locations.
template <const ExpansionType expansion_direction, const bool FORWARD>
std::vector<TimeDistance> TimeDistanceBSSMatrix::ComputeMatrix(
    const google::protobuf::RepeatedPtrField<valhalla::Location>& source_location_list,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& target_location_list,
    baldr::GraphReader& graphreader,
    const float max_matrix_distance,
    const uint32_t matrix_locations) {
  // Run a series of one to many calls and concatenate the results.
  const auto& origins = FORWARD ? source_location_list : target_location_list;
  const auto& destinations = FORWARD ? target_location_list : source_location_list;

  // Construct adjacency list, edge status, and done set. Set bucket size and
  // cost range based on DynamicCost.
  uint32_t bucketsize = std::max(pedestrian_costing_->UnitSize(), bicycle_costing_->UnitSize());

  std::vector<TimeDistance> many_to_many(origins.size() * destinations.size());
  for (size_t origin_index = 0; origin_index < origins.size(); ++origin_index) {
    const auto& origin = origins.Get(origin_index);
    std::vector<TimeDistance> one_to_many;

    current_cost_threshold_ = GetCostThreshold(max_matrix_distance);
    adjacencylist_.reuse(0.0f, current_cost_threshold_, bucketsize, &edgelabels_);

    // Initialize the origin and destination locations
    settled_count_ = 0;
    SetOrigin<expansion_direction>(graphreader, origin);
    // TODO:
    SetDestinations<expansion_direction>(graphreader, destinations);

    // Find shortest path
    graph_tile_ptr tile;
    while (true) {
      // Get next element from adjacency list. Check that it is valid. An
      // invalid label indicates there are no edges that can be expanded.
      uint32_t predindex = adjacencylist_.pop();
      if (predindex == kInvalidLabel) {
        // Can not expand any further...
        one_to_many = FormTimeDistanceMatrix();
        break;
      }

      // Remove label from adjacency list, mark it as permanently labeled.
      // Copy the EdgeLabel for use in costing
      EdgeLabel pred = edgelabels_[predindex];

      // Mark the edge as permanently labeled. Do not do this for an origin
      // edge (this will allow loops/around the block cases)
      if (!pred.origin() && pred.mode() == travel_mode_t::kPedestrian) {
        pedestrian_edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent);
      }

      if (!pred.origin() && pred.mode() == travel_mode_t::kBicycle) {
        bicycle_edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent);
      }

      // Identify any destinations on this edge
      auto destedge = dest_edges_.find(pred.edgeid());
      if (destedge != dest_edges_.end() && pred.mode() == travel_mode_t::kPedestrian) {
        // Update any destinations along this edge. Return if all destinations
        // have been settled.
        tile = graphreader.GetGraphTile(pred.edgeid());
        const DirectedEdge* edge = tile->directededge(pred.edgeid());
        if (UpdateDestinations(origin, destinations, destedge->second, edge, tile, pred,
                               matrix_locations)) {
          one_to_many = FormTimeDistanceMatrix();
          break;
        }
      }

      // Terminate when we are beyond the cost threshold
      if (pred.cost().cost > current_cost_threshold_) {
        one_to_many = FormTimeDistanceMatrix();
        break;
      }

      // Expand forward from the end node of the predecessor edge.
      Expand<expansion_direction>(graphreader, pred.endnode(), pred, predindex, false, false,
                                  pred.mode());
    }

    // Insert one-to-many into many-to-many
    if (FORWARD) {
      for (size_t target_index = 0; target_index < destinations.size(); target_index++) {
        size_t index = origin_index * origins.size() + target_index;
        many_to_many[index] = one_to_many[target_index];
      }
    } else {
      for (size_t source_index = 0; source_index < destinations.size(); source_index++) {
        size_t index = source_index * origins.size() + origin_index;
        many_to_many[index] = one_to_many[source_index];
      }
    }
    clear();
  }
  return many_to_many;
}

template std::vector<TimeDistance> TimeDistanceBSSMatrix::ComputeMatrix<ExpansionType::forward, true>(
    const google::protobuf::RepeatedPtrField<valhalla::Location>& source_location_list,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& target_location_list,
    baldr::GraphReader& graphreader,
    const float max_matrix_distance,
    const uint32_t matrix_locations);
template std::vector<TimeDistance>
TimeDistanceBSSMatrix::ComputeMatrix<ExpansionType::reverse, false>(
    const google::protobuf::RepeatedPtrField<valhalla::Location>& source_location_list,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& target_location_list,
    baldr::GraphReader& graphreader,
    const float max_matrix_distance,
    const uint32_t matrix_locations);

// Add edges at the origin to the adjacency list
template <const ExpansionType expansion_direction, const bool FORWARD>
void TimeDistanceBSSMatrix::SetOrigin(GraphReader& graphreader, const valhalla::Location& origin) {
  // Only skip inbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.correlation().edges().begin(), origin.correlation().edges().end(),
                [&has_other_edges](const valhalla::PathEdge& e) {
                  has_other_edges = has_other_edges || (FORWARD ? !e.end_node() : !e.begin_node());
                });

  // Iterate through edges and add to adjacency list
  for (const auto& edge : origin.correlation().edges()) {
    // If origin is at a node - skip any inbound edge (dist = 1)
    if ((FORWARD ? edge.end_node() : edge.begin_node()) && has_other_edges) {
      continue;
    }

    // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
    GraphId edgeid(edge.graph_id());
    if (FORWARD ? pedestrian_costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())
                : pedestrian_costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
      continue;
    }

    // Get the directed edge
    graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the tile at the end node. Skip if tile not found as we won't be
    // able to expand from this origin edge.
    graph_tile_ptr endtile = graphreader.GetGraphTile(directededge->endnode());
    if (endtile == nullptr) {
      continue;
    }

    uint8_t flow_sources;
    // Cost is also sortcost, since this is Dijsktra
    Cost cost;
    float dist;
    GraphId opp_edge_id;
    const DirectedEdge* opp_dir_edge;
    const auto time_info = TimeInfo::invalid();
    if (FORWARD) {
      const auto percent_along = 1.0f - edge.percent_along();
      cost =
          pedestrian_costing_->EdgeCost(directededge, tile, time_info, flow_sources) * percent_along;
      dist = static_cast<uint32_t>(directededge->length() * percent_along);

    } else {
      opp_edge_id = graphreader.GetOpposingEdgeId(edgeid);
      if (!opp_edge_id.Is_Valid()) {
        continue;
      }
      opp_dir_edge = graphreader.GetOpposingEdge(edgeid);
      cost = pedestrian_costing_->EdgeCost(opp_dir_edge, endtile, time_info, flow_sources) *
             edge.percent_along();
      dist = static_cast<uint32_t>(directededge->length() * edge.percent_along());
    }

    // We need to penalize this location based on its score (distance in meters from input)
    // We assume the slowest speed you could travel to cover that distance to start/end the route
    // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
    cost.cost += edge.distance();

    // Add EdgeLabel to the adjacency list (but do not set its status).
    // Set the predecessor edge index to invalid to indicate the origin
    // of the path. Set the origin flag
    if (FORWARD) {
      edgelabels_.emplace_back(kInvalidLabel, edgeid, directededge, cost, cost.cost, 0.0f,
                               travel_mode_t::kPedestrian, dist, Cost{}, baldr::kInvalidRestriction,
                               true, false, InternalTurn::kNoTurn);
    } else {
      edgelabels_.emplace_back(kInvalidLabel, opp_edge_id, opp_dir_edge, cost, cost.cost, 0.0f,
                               travel_mode_t::kPedestrian, dist, Cost{}, baldr::kInvalidRestriction,
                               true, false, InternalTurn::kNoTurn);
    }
    edgelabels_.back().set_origin();
    adjacencylist_.add(edgelabels_.size() - 1);
  }
}

// Set destinations
template <const ExpansionType expansion_direction, const bool FORWARD>
void TimeDistanceBSSMatrix::SetDestinations(
    GraphReader& graphreader,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& locations) {
  // For each destination
  uint32_t idx = 0;
  for (const auto& loc : locations) {
    // Set up the destination - consider each possible location edge.
    bool added = false;

    // Only skip outbound edges if we have other options
    bool has_other_edges = false;
    std::for_each(loc.correlation().edges().begin(), loc.correlation().edges().end(),
                  [&has_other_edges](const valhalla::PathEdge& e) {
                    has_other_edges = has_other_edges || (FORWARD ? !e.begin_node() : !e.end_node());
                  });
    for (const auto& edge : loc.correlation().edges()) {
      // If destination is at a node skip any outbound edges
      if (has_other_edges && (FORWARD ? edge.begin_node() : edge.end_node())) {
        continue;
      }

      // Disallow any user avoided edges if the avoid location is behind the destination along the
      // edge
      GraphId edgeid(edge.graph_id());
      GraphId opp_edge_id;
      if (FORWARD ? pedestrian_costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())
                  : pedestrian_costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Add a destination if this is the first allowed edge for the location
      if (!added) {
        destinations_.emplace_back();
        added = true;
      }

      // Form a threshold cost (the total cost to traverse the edge)
      graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
      const DirectedEdge* directededge = tile->directededge(edgeid);
      float c = pedestrian_costing_->EdgeCost(directededge, tile).cost;

      // Keep the id and the partial distance for the remainder of the edge.
      Destination& d = destinations_.back();
      edgeid = FORWARD ? edgeid : graphreader.GetOpposingEdgeId(edgeid);
      auto percent_along = FORWARD ? (1.0f - edge.percent_along()) : edge.percent_along();

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
      c += edge.distance();
      if (c > d.threshold) {
        d.threshold = c;
      }

      // Mark the edge as having a destination on it and add the
      // destination index
      d.dest_edges[edgeid] = percent_along;
      dest_edges_[edgeid].push_back(idx);
    }
    idx++;
  }
}

// Update any destinations along the edge. Returns true if all destinations
// have be settled.
bool TimeDistanceBSSMatrix::UpdateDestinations(
    const valhalla::Location& origin,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
    std::vector<uint32_t>& destinations,
    const DirectedEdge* edge,
    const graph_tile_ptr& tile,
    const EdgeLabel& pred,
    const uint32_t matrix_locations) {
  // For each destination along this edge
  for (auto dest_idx : destinations) {
    Destination& dest = destinations_[dest_idx];

    // Skip if destination has already been settled. This can happen since we
    // do not remove remaining destination edges for this destination from
    // dest_edges.
    if (dest.settled) {
      continue;
    }

    // See if this edge is part of the destination
    // TODO - it should always be, but protect against not finding it
    auto dest_edge = dest.dest_edges.find(pred.edgeid());
    if (dest_edge == dest.dest_edges.end()) {
      // If the edge isn't there but the path is trivial, then that means the edge
      // was removed towards the beginning which is not an error.
      if (!IsTrivial(pred.edgeid(), origin, locations.Get(dest_idx))) {
        LOG_ERROR("Could not find the destination edge");
      }
      continue;
    }

    // Skip case where destination is along the origin edge, there is no
    // predecessor, and the destination cannot be reached via trivial path.
    if (pred.predecessor() == kInvalidLabel &&
        !IsTrivial(pred.edgeid(), origin, locations.Get(dest_idx))) {
      continue;
    }

    // Get the cost. The predecessor cost is cost to the end of the edge.
    // Subtract the partial remaining cost and distance along the edge.
    float remainder = dest_edge->second;
    Cost newcost = pred.cost() - (pedestrian_costing_->EdgeCost(edge, tile) * remainder);
    if (newcost.cost < dest.best_cost.cost) {
      dest.best_cost = newcost;
      dest.distance = pred.path_distance() - (edge->length() * remainder);
    }

    // Erase this edge from further consideration. Mark this destination as
    // settled if all edges have been found
    dest.dest_edges.erase(dest_edge);
    if (dest.dest_edges.empty()) {
      dest.settled = true;
      settled_count_++;
    }
  }

  // Settle any destinations where current cost is above the destination's
  // best cost + threshold. This helps remove destinations where one edge
  // cannot be reached (e.g. on a cul-de-sac or where turn restrictions apply).
  // Update the cost threshold if at least one path to all destinations has
  // been found.
  bool allfound = true;
  float maxcost = 0.0f;
  for (auto& d : destinations_) {
    // Skip any settled destinations
    if (d.settled) {
      continue;
    }

    // Do not update cost threshold if no path to this destination
    // has been found
    if (d.best_cost.cost == kMaxCost) {
      allfound = false;
    } else {
      // Settle any destinations above their threshold and update maxcost
      if ((d.best_cost.cost + d.threshold) < pred.cost().cost) {
        d.settled = true;
        settled_count_++;
      }
      maxcost = std::max(maxcost, d.best_cost.cost + d.threshold);
    }
  }

  // Update cost threshold for early termination if at least one path has
  // been found to each destination
  if (allfound) {
    current_cost_threshold_ = maxcost;
  }

  // Return true if the settled count equals the number of destinations or
  // exceeds the matrix location count provided.
  return settled_count_ == destinations_.size() || settled_count_ >= matrix_locations;
}

// Form the time, distance matrix from the destinations list
std::vector<TimeDistance> TimeDistanceBSSMatrix::FormTimeDistanceMatrix() {
  std::vector<TimeDistance> td;
  for (auto& dest : destinations_) {
    td.emplace_back(dest.best_cost.secs, dest.distance);
  }
  return td;
}

} // namespace thor
} // namespace valhalla
