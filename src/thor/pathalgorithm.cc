#include <iostream> // TODO remove if not needed
#include <map>
#include <algorithm>
#include "thor/pathalgorithm.h"
#include <valhalla/baldr/datetime.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;

// TODO: make a class that extends std::exception, with messages and
// error codes and return the appropriate error codes

namespace valhalla {
namespace thor {

constexpr uint64_t kInitialEdgeLabelCount = 500000;

// Default constructor
PathAlgorithm::PathAlgorithm()
    : mode_(TravelMode::kDrive),
      allow_transitions_(false),
      adjacencylist_(nullptr),
      edgestatus_(nullptr),
      tile_creation_date_(0) {
  edgelabels_.reserve(kInitialEdgeLabelCount);
}

// Destructor
PathAlgorithm::~PathAlgorithm() {
  Clear();
}

// Clear the temporary information generated during path construction.
void PathAlgorithm::Clear() {
  // Clear the edge labels and destination list
  edgelabels_.clear();
  destinations_.clear();

  // Clear elements from the adjacency list
  adjacencylist_.reset();

  // Clear the edge status flags
  edgestatus_.reset();
}

// Initialize prior to finding best path
void PathAlgorithm::Init(const PointLL& origll, const PointLL& destll,
                         const std::shared_ptr<DynamicCost>& costing) {
  LOG_TRACE("Orig LL = " + std::to_string(origll.lat()) + "," + std::to_string(origll.lng()));
  LOG_TRACE("Dest LL = " + std::to_string(destll.lat()) + "," + std::to_string(destll.lng()));

  // Set the destination and cost factor in the A* heuristic
  astarheuristic_.Init(destll, costing->AStarCostFactor());

  // Get the initial cost based on A* heuristic from origin
  float mincost = astarheuristic_.Get(origll);

  // Construct adjacency list, edge status, and done set
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_.reset(new AdjacencyList(mincost, range, bucketsize));
  edgestatus_.reset(new EdgeStatus());

  // Get hierarchy limits from the costing. Get a copy since we increment
  // transition counts (i.e., this is not a const reference).
  allow_transitions_ = costing->AllowTransitions();
  hierarchy_limits_  = costing->GetHierarchyLimits();
}

// Modulate the hierarchy expansion within distance based on density at
// the destination (increase distance for lower densities and decrease
// for higher densities) and the distance between origin and destination
// (increase for shorter distances).
void PathAlgorithm::ModifyHierarchyLimits(const float dist,
                                          const uint32_t density) {
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

// Calculate best path. This method is single mode, not time-dependent.
std::vector<PathInfo> PathAlgorithm::GetBestPath(PathLocation& origin,
             PathLocation& destination, GraphReader& graphreader,
             const std::shared_ptr<DynamicCost>* mode_costing,
             const TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode_)];

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  Init(origin.vertex(), destination.vertex(), costing);
  float mindist = astarheuristic_.GetDistance(origin.vertex());

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  uint32_t density = SetDestination(graphreader, destination, costing);
  SetOrigin(graphreader, origin, destination, costing);

  // Update hierarchy limits
  if (allow_transitions_) {
    ModifyHierarchyLimits(mindist, density);
  }

  // Find shortest path
  uint32_t nc = 0;       // Count of iterations with no convergence
                         // towards destination
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->Remove(edgelabels_);
    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " +
                     std::to_string(edgelabels_.size()));
      return { };
    }

    // Copy the EdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    EdgeLabel pred = edgelabels_[predindex];
    if (destinations_.find(pred.edgeid()) != destinations_.end()) {
      // Check if a trivial path. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        if (IsTrivial(pred.edgeid(), origin, destination)) {
          return FormPath(predindex);
        }
      } else {
        return FormPath(predindex);
      }
    }

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge (this will allow loops/around the block cases)
    if (!pred.origin()) {
      edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Check that distance is converging towards the destination. Return route
    // failure if no convergence for TODO iterations
    float dist2dest = pred.distance();
    if (dist2dest < mindist) {
      mindist = dist2dest;
      nc = 0;
    } else if (nc++ > 500000) {
      LOG_ERROR("No convergence to destination after = " +
                           std::to_string(edgelabels_.size()));
      return {};
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

    // Check hierarchy. Count upward transitions (counted on the level
    // transitioned from). Do not expand based on hierarchy level based on
    // number of upward transitions and distance to the destination
    uint32_t level = node.level();
    if (pred.trans_up()) {
      hierarchy_limits_[level+1].up_transition_count++;
    }
    if (hierarchy_limits_[level].StopExpanding(dist2dest)) {
      continue;
    }

    // Expand from end node.
    uint32_t shortcuts = 0;
    uint32_t max_shortcut_length = static_cast<uint32_t>(dist2dest * 0.5f);
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count();
                i++, directededge++, edgeid++) {
      // Handle transition edges they either get skipped or added to the
      // adjacency list using the predecessor info
      if (directededge->trans_up() || directededge->trans_down()) {
        HandleTransitionEdge(level, edgeid, directededge, pred,
                             predindex, dist2dest);
        continue;
      }

      // Skip shortcut edges when near the destination. Always skip within
      // 10km but also reject long shortcut edges outside this distance.
      // TODO - configure this distance based on density?
      if (directededge->is_shortcut() && (dist2dest < 10000.0f ||
          directededge->length() > max_shortcut_length)) {
        continue;
      }

      // Skip any superseded edges that match the shortcut mask. Also skip
      // if no access is allowed to this edge (based on costing method)
      if ((shortcuts & directededge->superseded()) ||
          !costing->Allowed(directededge, pred, tile, edgeid)) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.set() == EdgeSet::kPermanent) {
        continue;
      }

      // Update the_shortcuts mask
      shortcuts |= directededge->shortcut();

      // Compute the cost to the end of this edge
      Cost newcost = pred.cost() +
			     costing->EdgeCost(directededge, nodeinfo->density()) +
			     costing->TransitionCost(directededge, nodeinfo, pred);

      // If this edge is a destination, subtract the partial/remainder cost
      // (cost from the dest. location to the end of the edge).
      auto p = destinations_.find(edgeid);
      if (p != destinations_.end()) {
        newcost -= p->second;
      }

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change)
      if (edgestatus.set() == EdgeSet::kTemporary) {
        CheckIfLowerCostPath(edgestatus.status.index, predindex, newcost);
        continue;
      }

      // If this is a destination edge the A* heuristic is 0. Otherwise the
      // sort cost (with A* heuristic) is found using the lat,lng at the
      // end node of the directed edge.
      float dist = 0.0f;
      float sortcost = newcost.cost;
      if (p == destinations_.end()) {
        const GraphTile* t2 = directededge->leaves_tile() ?
            graphreader.GetGraphTile(directededge->endnode()) : tile;
        if (t2 == nullptr) {
          continue;
        }
        sortcost += astarheuristic_.Get(
                    t2->node(directededge->endnode())->latlng(), dist);
      }

      // Add to the adjacency list and edge labels.
      AddToAdjacencyList(edgeid, sortcost);
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                    newcost, sortcost, dist, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_, 0);
    }
  }
  return {};      // Should never get here
}

// Convenience method to add an edge to the adjacency list and temporarily
// label it.
void PathAlgorithm::AddToAdjacencyList(const GraphId& edgeid,
                                       const float sortcost) {
  uint32_t idx = edgelabels_.size();
  adjacencylist_->Add(idx, sortcost);
  edgestatus_->Set(edgeid, EdgeSet::kTemporary, idx);
}

// Check if edge is temporarily labeled and this path has less cost. If
// less cost the predecessor is updated and the sort cost is decremented
// by the difference in real cost (A* heuristic doesn't change)
void PathAlgorithm::CheckIfLowerCostPath(const uint32_t idx,
                                         const uint32_t predindex,
                                         const Cost& newcost) {
  float dc = edgelabels_[idx].cost().cost - newcost.cost;
  if (dc > 0) {
    float oldsortcost = edgelabels_[idx].sortcost();
    float newsortcost = oldsortcost - dc;
    edgelabels_[idx].Update(predindex, newcost, newsortcost);
    adjacencylist_->DecreaseCost(idx, newsortcost, oldsortcost);
  }
}

// Handle a transition edge between hierarchies.
void PathAlgorithm::HandleTransitionEdge(const uint32_t level,
                    const GraphId& edgeid, const DirectedEdge* edge,
                    const EdgeLabel& pred, const uint32_t predindex,
                    const float dist) {
  // Skip any transition edges that are not allowed.
  if (!allow_transitions_ ||
      (edge->trans_up() &&
       !hierarchy_limits_[level].AllowUpwardTransition(dist)) ||
      (edge->trans_down() &&
       !hierarchy_limits_[level].AllowDownwardTransition(dist))) {
    return;
  }

  // Allow the transition edge. Add it to the adjacency list and edge labels
  // using the predecessor information. Transition edges have no length.
  AddToAdjacencyList(edgeid, pred.sortcost());
  edgelabels_.emplace_back(predindex, edgeid,
                edge, pred.cost(), pred.sortcost(), dist,
                pred.restrictions(), pred.opp_local_idx(), mode_,
                pred.path_distance());
}

// Add an edge at the origin to the adjacency list
void PathAlgorithm::SetOrigin(GraphReader& graphreader,
                 PathLocation& origin,
                 const PathLocation& destination,
                 const std::shared_ptr<DynamicCost>& costing) {
  // Iterate through edges and add to adjacency list
  const NodeInfo* nodeinfo = nullptr;
  for (const auto& edge : (origin.edges())) {
    // If origin is at a node - skip any inbound edge (dist = 1)
    if (origin.IsNode() && edge.dist == 1) {
      continue;
    }

    // Get the directed edge
    GraphId edgeid = edge.id;
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Set the tile creation date
    tile_creation_date_ = tile->header()->date_created();

    // Get the tile at the end node. Skip if tile not found as we won't be
    // able to expand from this origin edge.
    const GraphTile* endtile = graphreader.GetGraphTile(directededge->endnode());
    if (endtile == nullptr) {
      continue;
    }

    // Get cost
    nodeinfo = endtile->node(directededge->endnode());
    Cost cost = costing->EdgeCost(directededge,
                    graphreader.GetEdgeDensity(edge.id)) * (1.0f - edge.dist);
    float dist = astarheuristic_.GetDistance(nodeinfo->latlng());

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge) if the
    // destination is in a forward direction along the edge
    // TODO - if this doesn't make the cost go negative!
    auto p = destinations_.find(edgeid);
    if (p != destinations_.end()) {
      if (IsTrivial(edgeid, origin, destination)) {
        // Update cost and use A* heuristic from the origin location rather
        // than the end node of this edge
        cost -= p->second;
        dist = astarheuristic_.GetDistance(origin.latlng_);
      }
    }

    // Compute sortcost
    float sortcost = cost.cost + astarheuristic_.Get(dist);

    // Add EdgeLabel to the adjacency list (but do not set its status).
    // Set the predecessor edge index to invalid to indicate the origin
    // of the path.
    uint32_t d = static_cast<uint32_t>(directededge->length() * (1.0f - edge.dist));
    adjacencylist_->Add(edgelabels_.size(), sortcost);
    EdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost,
            sortcost, dist, directededge->restrictions(),
            directededge->opp_local_idx(), mode_, d);
    edge_label.set_origin();

    // Set the origin flag
    edgelabels_.push_back(std::move(edge_label));
  }

  // Set the origin timezone
  if (nodeinfo != nullptr && origin.date_time_ &&
	  *origin.date_time_ == "current") {
    origin.date_time_= DateTime::iso_date_time(
    		DateTime::get_tz_db().from_index(nodeinfo->timezone()));
  }
}

// Add a destination edge
uint32_t PathAlgorithm::SetDestination(GraphReader& graphreader,
                     const PathLocation& dest,
                     const std::shared_ptr<DynamicCost>& costing) {
  // For each edge
  uint32_t density = 0;
  for (const auto& edge : dest.edges()) {
    // If destination is at a node skip any outbound edges
    if (dest.IsNode() && edge.dist == 0.0f) {
      continue;
    }

    // Keep the id and the cost to traverse the partial distance for the
    // remainder of the edge. This cost is subtracted from the total cost
    // up to the end of the destination edge.
    const GraphTile* tile = graphreader.GetGraphTile(edge.id);
    destinations_[edge.id] = costing->EdgeCost(tile->directededge(edge.id),
              graphreader.GetEdgeDensity(edge.id)) * (1.0f - edge.dist);

    // Get the tile relative density
    density = tile->header()->density();
  }
  return density;
}

// Check for path completion along the same edge. Edge ID in question
// is along both an origin and destination and origin shows up at the
// beginning of the edge while the destination shows up at the end of
// the edge
bool PathAlgorithm::IsTrivial(const GraphId& edgeid,
                              const PathLocation& origin,
                              const PathLocation& destination) const {
  for (const auto& destination_edge : destination.edges()) {
    if (destination_edge.id == edgeid) {
      for (const auto& origin_edge : origin.edges()) {
        if (origin_edge.id == edgeid &&
            origin_edge.dist <= destination_edge.dist) {
          return true;
        }
      }
    }
  }
  return false;
}

// Form the path from the adjacency list.
std::vector<PathInfo> PathAlgorithm::FormPath(const uint32_t dest) {
  // Metrics to track
  LOG_INFO("path_cost::" + std::to_string(edgelabels_[dest].cost().cost));
  LOG_INFO("path_iterations::" + std::to_string(edgelabels_.size()));

  // Work backwards from the destination
  std::vector<PathInfo> path;
  for(auto edgelabel_index = dest; edgelabel_index != kInvalidLabel;
      edgelabel_index = edgelabels_[edgelabel_index].predecessor()) {
    const EdgeLabel& edgelabel = edgelabels_[edgelabel_index];
    path.emplace_back(edgelabel.mode(), edgelabel.cost().secs,
                      edgelabel.edgeid(), edgelabel.tripid());
  }

  // Reverse the list and return
  std:reverse(path.begin(), path.end());
  return path;
}

}
}
