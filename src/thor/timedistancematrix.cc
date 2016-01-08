#include <vector>
#include <algorithm>
#include "thor/timedistancematrix.h"
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

// Constructor with cost threshold.
TimeDistanceMatrix::TimeDistanceMatrix(float initial_cost_threshold)
    : settled_count_(0),
      initial_cost_threshold_(initial_cost_threshold),
      cost_threshold_(initial_cost_threshold) {
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
  edgestatus_.reset();
}

// Calculate time and distance from one origin location to many destination
// locations.
std::vector<TimeDistance> TimeDistanceMatrix::OneToMany(
            const uint32_t origin_index,
            const std::vector<PathLocation>& locations,
            GraphReader& graphreader,
            const std::shared_ptr<DynamicCost>* mode_costing,
            const TravelMode mode) {
  cost_threshold_ = initial_cost_threshold_;

  // Set the mode and costing
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode_)];

  // Construct adjacency list, edge status, and done set. Set bucket size and
  // cost range based on DynamicCost. Initialize A* heuristic with 0 cost
  // factor (needed for setting the origin).
  astarheuristic_.Init(locations[origin_index].latlng_, 0.0f);
  uint32_t bucketsize = costing->UnitSize();
  adjacencylist_.reset(new AdjacencyList(0.0f, initial_cost_threshold_,
                                         bucketsize));
  edgestatus_.reset(new EdgeStatus());

  // Initialize the origin and destination locations
  settled_count_ = 0;
  SetOriginOneToMany(graphreader, locations[origin_index], costing);
  SetDestinations(graphreader, origin_index, locations, costing);

  // Find shortest path
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->Remove(edgelabels_);
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
      edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Identify any destinations on this edge
    auto destedge = dest_edges_.find(pred.edgeid());
    if (destedge != dest_edges_.end()) {
      // Update any destinations along this edge. Return if all destinations
      // have been settled.
      tile = graphreader.GetGraphTile(pred.edgeid());
      const DirectedEdge* edge = tile->directededge(pred.edgeid());
      if (UpdateDestinations(origin_index, locations, destedge->second, edge,
                             pred, predindex, costing)) {
        return FormTimeDistanceMatrix();
      }
    }

    // Terminate when we are beyond the cost threshold
    if (pred.cost().cost > cost_threshold_) {
      return FormTimeDistanceMatrix();
    }

    // Get the end node of the prior directed edge. Skip if tile not found
    // (can happen with regional data sets).
    GraphId node = pred.endnode();
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Check access at the node
    const NodeInfo* nodeinfo = tile->node(node);
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    // Expand from end node.
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                i++, directededge++, edgeid++) {
      // Do not allow transitions so should never see downward transitions
      // or shortcuts (always remain on local hierarchy)
      if (directededge->trans_up()) {
        continue;
      }

      // Skip if no access is allowed to this edge (based on costing method)
      if (!costing->Allowed(directededge, pred, tile, edgeid)) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.set() == EdgeSet::kPermanent) {
        continue;
      }

      // Get cost and update distance
      Cost newcost = pred.cost() +
                     costing->EdgeCost(directededge, nodeinfo->density()) +
                     costing->TransitionCost(directededge, nodeinfo, pred);
      uint32_t distance = pred.walking_distance() + directededge->length();

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change)
      if (edgestatus.set() == EdgeSet::kTemporary) {
        uint32_t idx = edgestatus.status.index;
        float dc = edgelabels_[idx].cost().cost - newcost.cost;
        if (dc > 0) {
          float oldsortcost = edgelabels_[idx].sortcost();
          float newsortcost = oldsortcost - dc;
          edgelabels_[idx].Update(predindex, newcost, newsortcost,
                                  distance, 0, 0);
          adjacencylist_->DecreaseCost(idx, newsortcost, oldsortcost);
        }
        continue;
      }

      // Add to the adjacency list and edge labels.
      AddToAdjacencyList(edgeid, newcost.cost);
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                    newcost, newcost.cost, 0.0f, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_, distance,
                    0, 0, 0, false);
    }
  }
  return {};      // Should never get here
}

// Many to one time and distance cost matrix. Computes time and distance
// from many locations to one location.
std::vector<TimeDistance> TimeDistanceMatrix::ManyToOne(
            const uint32_t dest_index,
            const std::vector<PathLocation>& locations,
            GraphReader& graphreader,
            const std::shared_ptr<DynamicCost>* mode_costing,
            const TravelMode mode) {
  cost_threshold_ = initial_cost_threshold_;

  // Set the mode and costing
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode_)];

  // Construct adjacency list, edge status, and done set. Set bucket size and
  // cost range based on DynamicCost. Initialize A* heuristic with 0 cost
  // factor (needed for setting the origin).
  astarheuristic_.Init(locations[dest_index].latlng_, 0.0f);
  uint32_t bucketsize = costing->UnitSize();
  adjacencylist_.reset(new AdjacencyList(0.0f, initial_cost_threshold_,
                                         bucketsize));
  edgestatus_.reset(new EdgeStatus());

  // Initialize the origin and destination locations
  settled_count_ = 0;
  SetOriginManyToOne(graphreader, locations[dest_index], costing);
  SetDestinationsManyToOne(graphreader, dest_index, locations, costing);

  // Find shortest path
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->Remove(edgelabels_);
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
      edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Identify any destinations on this edge
    auto destedge = dest_edges_.find(pred.edgeid());
    if (destedge != dest_edges_.end()) {
      // Update any destinations along this edge. Return if all destinations
      // have been settled.
      tile = graphreader.GetGraphTile(pred.edgeid());
      const DirectedEdge* edge = tile->directededge(pred.edgeid());
      if (UpdateDestinations(dest_index, locations, destedge->second, edge,
                             pred, predindex, costing)) {
        return FormTimeDistanceMatrix();
      }
    }

    // Terminate when we are beyond the cost threshold
    if (pred.cost().cost > cost_threshold_) {
      return FormTimeDistanceMatrix();
    }

    // Get the end node of the prior directed edge. Skip if tile not found
    // (can happen with regional data sets).
    GraphId node = pred.endnode();
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Check access at the node
    const NodeInfo* nodeinfo = tile->node(node);
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    // Get the opposing predecessor directed edge
    const DirectedEdge* opp_pred_edge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, opp_pred_edge++) {
      if (opp_pred_edge->localedgeidx() == pred.opp_local_idx())
        break;
    }

    // Expand from end node.
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                i++, directededge++, edgeid++) {
      // Do not allow transitions so should never see downward transitions
      // or shortcuts (always remain on local hierarchy)
      if (directededge->trans_up()) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.set() == EdgeSet::kPermanent) {
        continue;
      }

      // Get opposing edge and check if allowed.
      const DirectedEdge* opp_edge = graphreader.GetOpposingEdge(edgeid);
      if (opp_edge == nullptr ||
         !costing->AllowedReverse(directededge, pred, opp_edge, opp_pred_edge,
                                  tile, edgeid)) {
        continue;
      }

      // Get cost. Use the opposing edge for EdgeCost.
      Cost newcost = pred.cost() +
                    costing->EdgeCost(opp_edge, nodeinfo->density()) +
                    costing->TransitionCostReverse(directededge->localedgeidx(),
                                        nodeinfo, opp_edge, opp_pred_edge);
      uint32_t distance = pred.walking_distance() + directededge->length();

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change)
      if (edgestatus.set() == EdgeSet::kTemporary) {
        uint32_t idx = edgestatus.status.index;
        float dc = edgelabels_[idx].cost().cost - newcost.cost;
        if (dc > 0) {
          float oldsortcost = edgelabels_[idx].sortcost();
          float newsortcost = oldsortcost - dc;
          edgelabels_[idx].Update(predindex, newcost, newsortcost,
                                  distance, 0, 0);
          adjacencylist_->DecreaseCost(idx, newsortcost, oldsortcost);
        }
        continue;
      }

      // Add to the adjacency list and edge labels.
      AddToAdjacencyList(edgeid, newcost.cost);
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                    newcost, newcost.cost, 0.0f, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_, distance,
                    0, 0, 0, false);
    }
  }
  return {};      // Should never get here
}

// Many to one time and distance cost matrix. Computes time and distance
// from many locations to many locations.
std::vector<TimeDistance> TimeDistanceMatrix::ManyToMany(
           const std::vector<PathLocation>& locations,
           GraphReader& graphreader,
           const std::shared_ptr<DynamicCost>* mode_costing,
           const sif::TravelMode mode) {
  // Run a series of one to many calls and concatenate the results.
  std::vector<TimeDistance> many_to_many;
  for (uint32_t origin_idx = 0; origin_idx < locations.size(); origin_idx++) {
    std::vector<TimeDistance> td = OneToMany(origin_idx, locations,
                          graphreader, mode_costing, mode);
    many_to_many.insert(many_to_many.end(), td.begin(), td.end());
    Clear();
  }
  return many_to_many;
}

// Add an edge at the origin to the adjacency list
void TimeDistanceMatrix::SetOriginOneToMany(GraphReader& graphreader,
                 const PathLocation& origin,
                 const std::shared_ptr<DynamicCost>& costing) {
  // Iterate through edges and add to adjacency list
  for (const auto& edge : (origin.edges())) {
    // If origin is at a node - skip any inbound edge (dist = 1)
    if (origin.IsNode() && edge.dist == 1) {
      continue;
    }

    // Get the directed edge
    GraphId edgeid = edge.id;
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
    Cost cost = costing->EdgeCost(directededge,
                  graphreader.GetEdgeDensity(edgeid)) * (1.0f - edge.dist);
    uint32_t d = static_cast<uint32_t>(directededge->length() *
                             (1.0f - edge.dist));

    // Add EdgeLabel to the adjacency list (but do not set its status).
    // Set the predecessor edge index to invalid to indicate the origin
    // of the path. Set the origin flag
    adjacencylist_->Add(edgelabels_.size(), cost.cost);
    EdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost,
            cost.cost, 0.0f, directededge->restrictions(),
            directededge->opp_local_idx(), mode_, d, 0, 0, 0, false);
    edge_label.set_origin();
    edgelabels_.push_back(std::move(edge_label));
  }
}

// Add origin for a many to one time distance matrix.
void TimeDistanceMatrix::SetOriginManyToOne(GraphReader& graphreader,
                      const PathLocation& dest,
                      const std::shared_ptr<DynamicCost>& costing) {
  // Iterate through edges and add opposing edges to adjacency list
  for (const auto& edge : dest.edges()) {
    // Get the directed edge
    GraphId edgeid = edge.id;
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
    Cost cost = costing->EdgeCost(opp_dir_edge,
                         graphreader.GetEdgeDensity(opp_edge_id)) * edge.dist;
    uint32_t d = static_cast<uint32_t>(directededge->length() * edge.dist);

    // Add EdgeLabel to the adjacency list (but do not set its status).
    // Set the predecessor edge index to invalid to indicate the origin
    // of the path. Set the origin flag.
    // TODO - restrictions?
    adjacencylist_->Add(edgelabels_.size(), cost.cost);
    EdgeLabel edge_label(kInvalidLabel, opp_edge_id, opp_dir_edge, cost,
            cost.cost, 0.0f, 0, opp_dir_edge->opp_local_idx(), mode_, d,
            0, 0, 0, false);
    edge_label.set_origin();
    edgelabels_.push_back(std::move(edge_label));
  }
}

// Set destinations
void TimeDistanceMatrix::SetDestinations(GraphReader& graphreader,
          const uint32_t origin_index,
          const std::vector<PathLocation>& locations,
          const std::shared_ptr<DynamicCost>& costing) {
  // For each destination
  uint32_t idx = 0;
  for (const auto& loc : locations) {
    // Add a destination and get a reference to it
    destinations_.emplace_back();
    Destination& d = destinations_.back();

    // If this is the origin, mark the destination as complete with
    // 0 cost and 0 distance
    if (idx == origin_index) {
      d.best_cost = { 0.0f, 0.0f };
      d.distance  = 0;
      d.settled = true;
      settled_count_++;
    } else {
      // Set up the destination - consider each possible location edge.
      for (const auto& edge : loc.edges()) {
        // Keep the id and the partial distance for the
        // remainder of the edge.
        d.dest_edges[edge.id] = (1.0f - edge.dist);

        // Form a threshold cost (the total cost to traverse the edge)
        const GraphTile* tile = graphreader.GetGraphTile(edge.id);
        float c = costing->EdgeCost(tile->directededge(edge.id),
                    graphreader.GetEdgeDensity(edge.id)).cost;
        if (c > d.threshold) {
          d.threshold = c;
        }

        // Mark the edge as having a destination on it and add the
        // destination index
        dest_edges_[edge.id].push_back(idx);
      }
    }
    idx++;
  }
}

// Set destinations for the many to one case.
void TimeDistanceMatrix::SetDestinationsManyToOne(GraphReader& graphreader,
          const uint32_t dest_index,
          const std::vector<PathLocation>& locations,
          const std::shared_ptr<DynamicCost>& costing) {
  // For each destination
  uint32_t idx = 0;
  for (const auto& loc : locations) {
    // Add a destination and get a reference to it
    destinations_.emplace_back();
    Destination& d = destinations_.back();

    // If this is the origin, mark the destination as complete with
    // 0 cost and 0 distance
    if (idx == dest_index) {
      d.best_cost = { 0.0f, 0.0f };
      d.distance  = 0;
      d.settled = true;
      settled_count_++;
    } else {
      // Set up the destination - consider each possible location edge.
      for (const auto& edge : loc.edges()) {
        // Get the opposing directed edge Id - this is the edge marked as the
        // "destination" - but the cost is based on the forward path along the
        // initial edge.
        GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edge.id);

        // Keep the id and the partial distance for the
        // remainder of the edge.
        d.dest_edges[opp_edge_id] = edge.dist;

        // Form a threshold cost (the total cost to traverse the edge)
        const GraphTile* tile = graphreader.GetGraphTile(edge.id);
        float c = costing->EdgeCost(tile->directededge(edge.id),
                    graphreader.GetEdgeDensity(edge.id)).cost;
        if (c > d.threshold) {
          d.threshold = c;
        }

        // Mark the edge as having a destination on it and add the
        // destination index
        dest_edges_[opp_edge_id].push_back(idx);
      }
    }
    idx++;
  }
}

// Update any destinations along the edge. Returns true if all destinations
// have be settled.
bool TimeDistanceMatrix::UpdateDestinations(const uint32_t origin_index,
                                const std::vector<PathLocation>& locations,
                                std::vector<uint32_t>& destinations,
                                const DirectedEdge* edge,
                                const EdgeLabel& pred,
                                const uint32_t predindex,
                                const std::shared_ptr<DynamicCost>& costing) {
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
      LOG_ERROR("Could not find the destination edge");
      continue;
    }

    // Skip case where destination is along the origin edge, there is no
    // predecessor, and the destination cannot be reached via trival path.
    if (pred.predecessor() == kInvalidLabel &&
        !IsTrivial(pred.edgeid(), locations[origin_index],
                   locations[dest_idx])) {
      continue;
    }

// TODO - need density for edgecost method...
    // Get the cost. The predecessor cost is cost to the end of the edge.
    // Subtract the partial remaining cost and distance along the edge.
    float remainder = dest_edge->second;
    Cost newcost = pred.cost() - (costing->EdgeCost(edge, 0.0f) * remainder);
    if (newcost.cost < dest.best_cost.cost) {
      dest.best_cost = newcost;
      dest.distance = pred.walking_distance() - (edge->length() * remainder);
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
    if (d.distance == 0) {
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
    cost_threshold_ = maxcost;
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

}
}
