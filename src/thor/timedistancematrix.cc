#include "thor/timedistancematrix.h"
#include "midgard/logging.h"
#include <algorithm>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {
static bool IsTrivial(const uint64_t& edgeid,
                      const valhalla::Location& origin,
                      const valhalla::Location& destination) {
  for (const auto& destination_edge : destination.path_edges()) {
    if (destination_edge.graph_id() == edgeid) {
      for (const auto& origin_edge : origin.path_edges()) {
        if (origin_edge.graph_id() == edgeid &&
            origin_edge.percent_along() <= destination_edge.percent_along()) {
          return true;
        }
      }
    }
  }
  return false;
}
} // namespace
namespace valhalla {
namespace thor {

// Constructor with cost threshold.
TimeDistanceMatrix::TimeDistanceMatrix()
    : mode_(TravelMode::kDrive), settled_count_(0), current_cost_threshold_(0) {
}

float TimeDistanceMatrix::GetCostThreshold(const float max_matrix_distance) const {
  float cost_threshold;
  switch (mode_) {
    case TravelMode::kBicycle:
      cost_threshold = max_matrix_distance / kTimeDistCostThresholdBicycleDivisor;
      break;
    case TravelMode::kPedestrian:
    case TravelMode::kPublicTransit:
      cost_threshold = max_matrix_distance / kTimeDistCostThresholdPedestrianDivisor;
      break;
    case TravelMode::kDrive:
    default:
      cost_threshold = max_matrix_distance / kTimeDistCostThresholdAutoDivisor;
  }
  return cost_threshold;
}

// Clear the temporary information generated during time + distance matrix
// construction.
void TimeDistanceMatrix::Clear() {
  // Clear the edge labels and destination list
  edgelabels_.clear();
  destinations_.clear();
  dest_edges_.clear();

  // Clear elements from the adjacency list
  adjacencylist_.reset();

  // Clear the edge status flags
  edgestatus_.clear();
}

// Expand from a node in the forward direction
void TimeDistanceMatrix::ExpandForward(GraphReader& graphreader,
                                       const GraphId& node,
                                       const EdgeLabel& pred,
                                       const uint32_t pred_idx,
                                       const bool from_transition) {
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

  // Expand from end node.
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  EdgeStatusInfo* es = edgestatus_.GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, ++edgeid, ++es) {
    // Skip shortcut edges
    if (directededge->is_shortcut()) {
      continue;
    }

    // Skip this edge if permanently labeled (best path already found to this
    // directed edge), if no access is allowed to this edge (based on costing
    // method), or if a complex restriction prevents this path.
    bool has_time_restrictions = false;
    if (es->set() == EdgeSet::kPermanent ||
        !costing_->Allowed(directededge, pred, tile, edgeid, 0, 0, has_time_restrictions) ||
        costing_->Restricted(directededge, pred, edgelabels_, tile, edgeid, true)) {
      continue;
    }

    // Get cost and update distance
    Cost newcost = pred.cost() + costing_->EdgeCost(directededge, tile) +
                   costing_->TransitionCost(directededge, nodeinfo, pred);
    uint32_t distance = pred.path_distance() + directededge->length();

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (es->set() == EdgeSet::kTemporary) {
      EdgeLabel& lab = edgelabels_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_->decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, distance, has_time_restrictions);
      }
      continue;
    }

    // Add to the adjacency list and edge labels.
    uint32_t idx = edgelabels_.size();
    edgelabels_.emplace_back(pred_idx, edgeid, directededge, newcost, newcost.cost, 0.0f, mode_,
                             distance, has_time_restrictions);
    *es = {EdgeSet::kTemporary, idx};
    adjacencylist_->add(idx);
  }

  // Handle transitions - expand from the end node each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true);
    }
  }
}

// Calculate time and distance from one origin location to many destination
// locations.
std::vector<TimeDistance>
TimeDistanceMatrix::OneToMany(const valhalla::Location& origin,
                              const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                              GraphReader& graphreader,
                              const std::shared_ptr<DynamicCost>* mode_costing,
                              const TravelMode mode,
                              const float max_matrix_distance) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  current_cost_threshold_ = GetCostThreshold(max_matrix_distance);

  // Construct adjacency list, edge status, and done set. Set bucket size and
  // cost range based on DynamicCost. Initialize A* heuristic with 0 cost
  // factor (needed for setting the origin).
  astarheuristic_.Init({origin.ll().lng(), origin.ll().lat()}, 0.0f);
  uint32_t bucketsize = costing_->UnitSize();
  // Set up lambda to get sort costs
  const auto edgecost = [this](const uint32_t label) { return edgelabels_[label].sortcost(); };
  adjacencylist_.reset(new DoubleBucketQueue(0.0f, current_cost_threshold_, bucketsize, edgecost));
  edgestatus_.clear();

  // Initialize the origin and destination locations
  settled_count_ = 0;
  SetOriginOneToMany(graphreader, origin);
  SetDestinations(graphreader, locations);

  // Find shortest path
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->pop();
    if (predindex == kInvalidLabel) {
      // Can not expand any further...
      return FormTimeDistanceMatrix();
    }

    // Remove label from adjacency list, mark it as permanently labeled.
    // Copy the EdgeLabel for use in costing
    EdgeLabel pred = edgelabels_[predindex];

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge. Otherwise loops/around the block cases will not work
    if (!pred.origin()) {
      edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Identify any destinations on this edge
    auto destedge = dest_edges_.find(pred.edgeid());
    if (destedge != dest_edges_.end()) {
      // Update any destinations along this edge. Return if all destinations
      // have been settled.
      tile = graphreader.GetGraphTile(pred.edgeid());
      const DirectedEdge* edge = tile->directededge(pred.edgeid());
      if (UpdateDestinations(origin, locations, destedge->second, edge, tile, pred, predindex)) {
        return FormTimeDistanceMatrix();
      }
    }

    // Terminate when we are beyond the cost threshold
    if (pred.cost().cost > current_cost_threshold_) {
      return FormTimeDistanceMatrix();
    }

    // Expand forward from the end node of the predecessor edge.
    ExpandForward(graphreader, pred.endnode(), pred, predindex, false);
  }
  return {}; // Should never get here
}

// Expand from the node along the reverse search path.
void TimeDistanceMatrix::ExpandReverse(GraphReader& graphreader,
                                       const GraphId& node,
                                       const EdgeLabel& pred,
                                       const uint32_t pred_idx,
                                       const bool from_transition) {
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

  // Get the opposing predecessor directed edge
  const DirectedEdge* opp_pred_edge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, opp_pred_edge++) {
    if (opp_pred_edge->localedgeidx() == pred.opp_local_idx()) {
      break;
    }
  }

  // Expand from end node.
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  EdgeStatusInfo* es = edgestatus_.GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n; i++, directededge++, ++edgeid, ++es) {
    // Skip shortcut edges and edges permanently labeled (best
    // path already found to this directed edge).
    if (directededge->is_shortcut() || es->set() == EdgeSet::kPermanent) {
      continue;
    }

    // Get opposing edge Id and end node tile
    const GraphTile* t2 =
        directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
    if (t2 == nullptr) {
      continue;
    }
    GraphId oppedge = t2->GetOpposingEdgeId(directededge);

    // Get opposing directed edge and check if allowed.
    const DirectedEdge* opp_edge = t2->directededge(oppedge);
    bool has_time_restrictions = false;
    if (opp_edge == nullptr || !costing_->AllowedReverse(directededge, pred, opp_edge, t2, oppedge, 0,
                                                         0, has_time_restrictions)) {
      continue;
    }

    // Get cost. Use the opposing edge for EdgeCost.
    Cost newcost = pred.cost() + costing_->EdgeCost(opp_edge, t2) +
                   costing_->TransitionCostReverse(directededge->localedgeidx(), nodeinfo, opp_edge,
                                                   opp_pred_edge);
    uint32_t distance = pred.path_distance() + directededge->length();

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (es->set() == EdgeSet::kTemporary) {
      EdgeLabel& lab = edgelabels_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_->decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, distance);
        lab.set_has_time_restriction(has_time_restrictions);
      }
      continue;
    }

    // Add to the adjacency list and edge labels.
    uint32_t idx = edgelabels_.size();
    edgelabels_.emplace_back(pred_idx, edgeid, directededge, newcost, newcost.cost, 0.0f, mode_,
                             distance, has_time_restrictions);
    *es = {EdgeSet::kTemporary, idx};
    adjacencylist_->add(idx);
  }

  // Handle transitions - expand from the end node each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      ExpandReverse(graphreader, trans->endnode(), pred, pred_idx, true);
    }
  }
}

// Many to one time and distance cost matrix. Computes time and distance
// from many locations to one location.
std::vector<TimeDistance>
TimeDistanceMatrix::ManyToOne(const valhalla::Location& dest,
                              const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                              GraphReader& graphreader,
                              const std::shared_ptr<DynamicCost>* mode_costing,
                              const TravelMode mode,
                              const float max_matrix_distance) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  current_cost_threshold_ = GetCostThreshold(max_matrix_distance);

  // Construct adjacency list, edge status, and done set. Set bucket size and
  // cost range based on DynamicCost. Initialize A* heuristic with 0 cost
  // factor (needed for setting the origin).
  astarheuristic_.Init({dest.ll().lng(), dest.ll().lat()}, 0.0f);
  uint32_t bucketsize = costing_->UnitSize();
  const auto edgecost = [this](const uint32_t label) { return edgelabels_[label].sortcost(); };
  adjacencylist_.reset(new DoubleBucketQueue(0.0f, current_cost_threshold_, bucketsize, edgecost));
  edgestatus_.clear();

  // Initialize the origin and destination locations
  settled_count_ = 0;
  SetOriginManyToOne(graphreader, dest);
  SetDestinationsManyToOne(graphreader, locations);

  // Find shortest path
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->pop();
    if (predindex == kInvalidLabel) {
      // Can not expand any further...
      return FormTimeDistanceMatrix();
    }

    // Remove label from adjacency list, mark it as permanently labeled.
    // Copy the EdgeLabel for use in costing
    EdgeLabel pred = edgelabels_[predindex];

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge (this will allow loops/around the block cases)
    if (!pred.origin()) {
      edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Identify any destinations on this edge
    auto destedge = dest_edges_.find(pred.edgeid());
    if (destedge != dest_edges_.end()) {
      // Update any destinations along this edge. Return if all destinations
      // have been settled.
      tile = graphreader.GetGraphTile(pred.edgeid());
      const DirectedEdge* edge = tile->directededge(pred.edgeid());
      if (UpdateDestinations(dest, locations, destedge->second, edge, tile, pred, predindex)) {
        return FormTimeDistanceMatrix();
      }
    }

    // Terminate when we are beyond the cost threshold
    if (pred.cost().cost > current_cost_threshold_) {
      return FormTimeDistanceMatrix();
    }

    // Expand forward from the end node of the predecessor edge.
    ExpandReverse(graphreader, pred.endnode(), pred, predindex, false);
  }
  return {}; // Should never get here
}

// Many to one time and distance cost matrix. Computes time and distance
// from many locations to many locations.
std::vector<TimeDistance> TimeDistanceMatrix::ManyToMany(
    const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
    GraphReader& graphreader,
    const std::shared_ptr<DynamicCost>* mode_costing,
    const sif::TravelMode mode,
    const float max_matrix_distance) {
  return SourceToTarget(locations, locations, graphreader, mode_costing, mode, max_matrix_distance);
}

std::vector<TimeDistance> TimeDistanceMatrix::SourceToTarget(
    const google::protobuf::RepeatedPtrField<valhalla::Location>& source_location_list,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& target_location_list,
    baldr::GraphReader& graphreader,
    const std::shared_ptr<sif::DynamicCost>* mode_costing,
    const sif::TravelMode mode,
    const float max_matrix_distance) {
  // Run a series of one to many calls and concatenate the results.
  std::vector<TimeDistance> many_to_many;
  if (source_location_list.size() <= target_location_list.size()) {
    for (const auto& origin : source_location_list) {
      std::vector<TimeDistance> td = OneToMany(origin, target_location_list, graphreader,
                                               mode_costing, mode, max_matrix_distance);
      many_to_many.insert(many_to_many.end(), td.begin(), td.end());
      Clear();
    }
  } else {
    for (const auto& destination : target_location_list) {
      std::vector<TimeDistance> td = ManyToOne(destination, source_location_list, graphreader,
                                               mode_costing, mode, max_matrix_distance);
      many_to_many.insert(many_to_many.end(), td.begin(), td.end());
      Clear();
    }
  }
  return many_to_many;
}

// Add edges at the origin to the adjacency list
void TimeDistanceMatrix::SetOriginOneToMany(GraphReader& graphreader,
                                            const valhalla::Location& origin) {
  // Only skip inbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.path_edges().begin(), origin.path_edges().end(),
                [&has_other_edges](const valhalla::Location::PathEdge& e) {
                  has_other_edges = has_other_edges || !e.end_node();
                });

  // Iterate through edges and add to adjacency list
  for (const auto& edge : origin.path_edges()) {
    // If origin is at a node - skip any inbound edge (dist = 1)
    if (has_other_edges && edge.end_node()) {
      continue;
    }

    // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
    GraphId edgeid(edge.graph_id());
    if (costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
      continue;
    }

    // Get the directed edge
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the tile at the end node. Skip if tile not found as we won't be
    // able to expand from this origin edge.
    const GraphTile* endtile = graphreader.GetGraphTile(directededge->endnode());
    if (endtile == nullptr) {
      continue;
    }

    // Get cost. Use this as sortcost since A* is not used for time+distance
    // matrix computations. . Get distance along the remainder of this edge.
    Cost cost = costing_->EdgeCost(directededge, tile) * (1.0f - edge.percent_along());
    uint32_t d = static_cast<uint32_t>(directededge->length() * (1.0f - edge.percent_along()));

    // We need to penalize this location based on its score (distance in meters from input)
    // We assume the slowest speed you could travel to cover that distance to start/end the route
    // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
    cost.cost += edge.distance();

    // Add EdgeLabel to the adjacency list (but do not set its status).
    // Set the predecessor edge index to invalid to indicate the origin
    // of the path. Set the origin flag
    EdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost, cost.cost, 0.0f, mode_, d);
    edge_label.set_origin();
    edgelabels_.push_back(std::move(edge_label));
    adjacencylist_->add(edgelabels_.size() - 1);
  }
}

// Add origin for a many to one time distance matrix.
void TimeDistanceMatrix::SetOriginManyToOne(GraphReader& graphreader,
                                            const valhalla::Location& dest) {
  // Iterate through edges and add opposing edges to adjacency list
  for (const auto& edge : dest.path_edges()) {
    // Disallow any user avoided edges if the avoid location is behind the destination along the edge
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

    // Get the tile at the end node. Skip if tile not found as we won't be
    // able to expand from this origin edge.
    const GraphTile* endtile = graphreader.GetGraphTile(directededge->endnode());
    if (endtile == nullptr) {
      continue;
    }

    // Get cost. Use this as sortcost since A* is not used for time
    // distance matrix computations. Get the distance along the edge.
    Cost cost = costing_->EdgeCost(opp_dir_edge, endtile) * edge.percent_along();
    uint32_t d = static_cast<uint32_t>(directededge->length() * edge.percent_along());

    // We need to penalize this location based on its score (distance in meters from input)
    // We assume the slowest speed you could travel to cover that distance to start/end the route
    // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
    cost.cost += edge.distance();

    // Add EdgeLabel to the adjacency list (but do not set its status).
    // Set the predecessor edge index to invalid to indicate the origin
    // of the path. Set the origin flag.
    // TODO - restrictions?
    EdgeLabel edge_label(kInvalidLabel, opp_edge_id, opp_dir_edge, cost, cost.cost, 0.0f, mode_, d);
    edge_label.set_origin();
    edgelabels_.push_back(std::move(edge_label));
    adjacencylist_->add(edgelabels_.size() - 1);
  }
}

// Set destinations
void TimeDistanceMatrix::SetDestinations(
    GraphReader& graphreader,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& locations) {
  // For each destination
  uint32_t idx = 0;
  for (const auto& loc : locations) {
    // Set up the destination - consider each possible location edge.
    bool added = false;
    for (const auto& edge : loc.path_edges()) {
      // Disallow any user avoided edges if the avoid location is behind the destination along the
      // edge
      GraphId edgeid(edge.graph_id());
      if (costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Add a destination if this is the first allowed edge for the location
      if (!added) {
        destinations_.emplace_back();
        added = true;
      }

      // Keep the id and the partial distance for the remainder of the edge.
      Destination& d = destinations_.back();
      d.dest_edges[edge.graph_id()] = (1.0f - edge.percent_along());

      // Form a threshold cost (the total cost to traverse the edge)
      GraphId id(static_cast<GraphId>(edge.graph_id()));
      const GraphTile* tile = graphreader.GetGraphTile(id);
      const DirectedEdge* directededge = tile->directededge(id);
      float c = costing_->EdgeCost(directededge, tile).cost;

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
      c += edge.distance();
      if (c > d.threshold) {
        d.threshold = c;
      }

      // Mark the edge as having a destination on it and add the
      // destination index
      dest_edges_[edge.graph_id()].push_back(idx);
    }
    idx++;
  }
}

// Set destinations for the many to one case.
void TimeDistanceMatrix::SetDestinationsManyToOne(
    GraphReader& graphreader,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& locations) {
  // For each destination
  uint32_t idx = 0;
  for (const auto& loc : locations) {
    // Set up the destination - consider each possible location edge.
    bool added = false;
    for (const auto& edge : loc.path_edges()) {
      // Get the opposing directed edge Id - this is the edge marked as the "destination",
      // but the cost is based on the forward path along the initial edge.
      GraphId opp_edge_id = graphreader.GetOpposingEdgeId(static_cast<GraphId>(edge.graph_id()));

      // Add a destination if this is the first allowed edge for the location
      if (!added) {
        destinations_.emplace_back();
        added = true;
      }

      // Keep the id and the partial distance for the remainder of the edge.
      Destination& d = destinations_.back();
      d.dest_edges[opp_edge_id] = edge.percent_along();

      // Form a threshold cost (the total cost to traverse the edge)
      GraphId id(static_cast<GraphId>(edge.graph_id()));
      const GraphTile* tile = graphreader.GetGraphTile(id);
      const DirectedEdge* directededge = tile->directededge(id);
      float c = costing_->EdgeCost(directededge, tile).cost;

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
      c += edge.distance();
      if (c > d.threshold) {
        d.threshold = c;
      }

      // Mark the edge as having a destination on it and add the
      // destination index
      dest_edges_[opp_edge_id].push_back(idx);
    }
    idx++;
  }
}

// Update any destinations along the edge. Returns true if all destinations
// have be settled.
bool TimeDistanceMatrix::UpdateDestinations(
    const valhalla::Location& origin,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
    std::vector<uint32_t>& destinations,
    const DirectedEdge* edge,
    const GraphTile* tile,
    const EdgeLabel& pred,
    const uint32_t predindex) {
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
    Cost newcost = pred.cost() - (costing_->EdgeCost(edge, tile) * remainder);
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
  return settled_count_ == destinations_.size();
}

// Form the time, distance matrix from the destinations list
std::vector<TimeDistance> TimeDistanceMatrix::FormTimeDistanceMatrix() {
  std::vector<TimeDistance> td;
  for (auto& dest : destinations_) {
    td.emplace_back(dest.best_cost.secs, dest.distance);
  }
  return td;
}

} // namespace thor
} // namespace valhalla
