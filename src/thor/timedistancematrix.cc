#include <vector>
#include <algorithm>
#include "thor/timedistancematrix.h"
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

// If we end up with locations where there is a trivial path but you would
// have to traverse the edge in reverse to do it we need to mark this as a
// loop so that the initial edge can be considered on the path at some later
// point in time
// TODO - test and validate loop cases.
GraphId loop(const uint32_t origin_index, const std::vector<PathLocation>& locations) {
  const PathLocation& origin = locations[origin_index];
  for (const auto& origin_edge : origin.edges()) {
    uint32_t index = 0;
    for (const auto& destination : locations) {
      if (index == origin_index) {
        index++;
        continue;
      }
      for (const auto& destination_edge : destination.edges()) {
        // same id and the origin shows up at the end of the edge
        // while the destination shows up at the beginning of the edge
        if (origin_edge.id == destination_edge.id &&
            origin_edge.dist > destination_edge.dist) {
          //something is wrong if we have more than one option here
          if (origin.edges().size() != 1 || destination.edges().size() != 1)
            throw std::runtime_error("Found oneway loop but multiple ins and outs!");

          return origin_edge.id;
        }
      }
      index++;
    }
  }
  return {};
}

// Calculate time and distance from one origin location to many destination
// locations.
std::vector<TimeDistance> TimeDistanceMatrix::OneToMany(
            const uint32_t origin_index,
            const std::vector<PathLocation>& locations,
            GraphReader& graphreader,
            const std::shared_ptr<DynamicCost>* mode_costing,
            const TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode_)];

  // Alter the destination edges if at a node - loki always gives edges
  // leaving a node, but when a destination we want edges entering the node
//  PathLocation dest = update_destinations(graphreader, destination,
//                                          costing->GetFilter());

  // Check for loop path (on any of the destination edges)
  PathInfo loop_edge_info(mode_, 0.0f, loop(origin_index, locations), 0);

  // Construct adjacency list, edge status, and done set. Set bucket size and
  // cost range based on DynamicCost. Initialize A* heuristic with 0 cost
  // factor (needed for setting the origin).
  astarheuristic_.Init(locations[origin_index].latlng_, 0.0f);
  uint32_t bucketsize = costing->UnitSize();
  adjacencylist_.reset(new AdjacencyList(0.0f, kBucketCount * bucketsize,
                                         bucketsize));
  edgestatus_.reset(new EdgeStatus());

  // Initialize the origin and destination locations
  settled_count_ = 0;
  SetOrigin(graphreader, locations[origin_index], costing, loop_edge_info);
  SetDestinations(graphreader, origin_index, locations, costing);

  // Set the initial cost threshold
  float cost_threshold = DEFAULT_COST_THRESHOLD;

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
    edgestatus_->Update(pred.edgeid(), kPermanent);

    // Identify any destinations on this edge
    auto destedge = dest_edges_.find(pred.edgeid());
    if (destedge != dest_edges_.end()) {
      // Update any destinations along this edge. Return if all destinations
      // have been settled.
      tile = graphreader.GetGraphTile(pred.edgeid());
      const DirectedEdge* edge = tile->directededge(pred.edgeid());
      if (UpdateDestinations(destedge->second, edge, pred,
                             predindex, costing)) {
        return FormTimeDistanceMatrix();
      }
    }

    // Terminate when we are beyond the cost threshold
    if (pred.cost().cost > cost_threshold) {
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
      if (!costing->Allowed(directededge, pred)) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.status.set == kPermanent) {
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
      if (edgestatus.status.set == kTemporary) {
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
        d.threshold = (costing->EdgeCost(tile->directededge(edge.id), 0.0f)).cost;

        // Mark the edge as having a destination on it and add the
        // destination index
        dest_edges_[edge.id].push_back(idx);
      }
    }
    idx++;
  }
}

// Update any destinations along the edge. Returns true if all destinations
// have be settled.
bool TimeDistanceMatrix::UpdateDestinations(std::vector<uint32_t>& destinations,
                                const DirectedEdge* edge,
                                const EdgeLabel& pred,
                                const uint32_t predindex,
                                const std::shared_ptr<DynamicCost>& costing) {
  // For each destination along this edge
  for (auto dest_idx : destinations) {
    Destination& dest = destinations_[dest_idx];

    // Skip if destination has already been settled.
    // TODO - remove once the threshold logic is fully validated
    if (dest.settled) {
      LOG_ERROR("Destination alread settled, threshold might be too low");
      continue;
    }

    // See if this edge is part of the destination
    // TODO - it should always be, but protect against not finding it
    auto dest_edge = dest.dest_edges.find(pred.edgeid());
    if (dest_edge == dest.dest_edges.end()) {
      LOG_ERROR("Could not find the destination edge");
      continue;
    }

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
  // cannot be reached (e.g. on a cul-de-sac or where turn restrictions apply)
  for (auto& d : destinations_) {
    if (!d.settled &&
        (d.best_cost.cost + d.threshold) < pred.cost().cost) {
      d.settled = true;
      settled_count_++;
    }
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

// Many to one time and distance cost matrix. Computes time and distance
// from many locations to one location. The last location in the locations
// vector is assumed to be the destination.
std::vector<TimeDistance> TimeDistanceMatrix::ManyToOne(
            const uint32_t dest_index,
            const std::vector<PathLocation>& locations,
            GraphReader& graphreader,
            const std::shared_ptr<DynamicCost>* mode_costing,
            const TravelMode mode) {
  LOG_ERROR("Not yet implemented");
  return { };
}

// Many to one time and distance cost matrix. Computes time and distance
// from many locations to many locations.
std::vector<TimeDistance> TimeDistanceMatrix::ManyToMany(
           const std::vector<PathLocation>& locations,
           GraphReader& graphreader,
           const std::shared_ptr<DynamicCost>* mode_costing,
           const sif::TravelMode mode) {
  LOG_ERROR("Not yet implemented");
  return { };
}

}
}
