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

// If the destination is at a node we want the incoming edge Ids
// with distance = 1.0 (the full edge). This returns and updated
// destination PathLocation.
// TODO - move this logic into Loki
// TODO - fail the route if no dest edges
PathLocation update_destinations(GraphReader& graphreader,
                                 const PathLocation& destination,
                                 const EdgeFilter& filter) {
  if (destination.IsNode()) {
    // Copy the current destination info and clear the edges
    PathLocation dest = destination;
    dest.ClearEdges();

    // Get the node. Iterate through the edges and get opposing edges. Add
    // to the destination edges if it is allowed by the costing model
    GraphId destedge = destination.edges()[0].id;
    GraphId opposing_edge = graphreader.GetOpposingEdgeId(destedge);
    GraphId endnode = graphreader.GetGraphTile(opposing_edge)->directededge(opposing_edge)->endnode();
    const GraphTile* tile = graphreader.GetGraphTile(endnode);
    const NodeInfo* nodeinfo = tile->node(endnode);
    GraphId edgeid(endnode.tileid(), endnode.level(), nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, edgeid++) {
      GraphId opposing_edge = graphreader.GetOpposingEdgeId(edgeid);
      tile = graphreader.GetGraphTile(opposing_edge);
      const DirectedEdge* edge = tile->directededge(opposing_edge);
      if (!filter(edge)) {
        dest.CorrelateEdge(PathLocation::PathEdge{opposing_edge, 1.0f});
      }
    }
    return dest;
  } else {
    // No need to alter destination edges
    return destination;
  }
}

// Check if any of the pairs of origin and destination edges could be a
// trivial path. This means the origin and destination are on the same edge.
GraphId trivial(const PathLocation& origin, const PathLocation& destination) {
  // NOTE: there could be a shorter path by leaving this edge and coming
  // back in the other direction however this should be uncommon
  for(const auto& origin_edge : origin.edges()) {
    for(const auto& destination_edge : destination.edges()) {
      // same id and the origin shows up at the beginning of the edge
      // while the destination shows up at the end of the edge
      if (origin_edge.id == destination_edge.id &&
          origin_edge.dist <= destination_edge.dist) {
        return origin_edge.id;
      }
    }
  }
  return {};
}

// If we end up with locations where there is a trivial path but you would
// have to traverse the edge in reverse to do it we need to mark this as a
// loop so that the initial edge can be considered on the path at some later
// point in time
GraphId loop(const PathLocation& origin, const PathLocation& destination) {
  for(const auto& origin_edge : origin.edges()) {
    for(const auto& destination_edge : destination.edges()) {
      //same id and the origin shows up at the end of the edge
      //while the destination shows up at the beginning of the edge
      if(origin_edge.id == destination_edge.id && origin_edge.dist > destination_edge.dist) {
        //something is wrong if we have more than one option here
        if(origin.edges().size() != 1 || destination.edges().size() != 1)
          throw std::runtime_error("Found oneway loop but multiple ins and outs!");
        return origin_edge.id;
      }
    }
  }
  return {};
}

// Default constructor
PathAlgorithm::PathAlgorithm()
    : allow_transitions_(false),
      adjacencylist_(nullptr),
      edgestatus_(nullptr),
      walking_distance_(0),
      best_destination_{kInvalidLabel, Cost(std::numeric_limits<float>::max(), 0.0f)} {
  edgelabels_.reserve(kInitialEdgeLabelCount);
}

// Destructor
PathAlgorithm::~PathAlgorithm() {
  Clear();
}

// Clear the temporary information generated during path construction.
void PathAlgorithm::Clear() {
  // Clear the edge labels
  edgelabels_.clear();
  best_destination_ = std::make_pair(kInvalidLabel,
                         Cost(std::numeric_limits<float>::max(), 0.0f));
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
std::vector<PathInfo> PathAlgorithm::GetBestPath(const PathLocation& origin,
             const PathLocation& destination, GraphReader& graphreader,
             const std::shared_ptr<DynamicCost>* mode_costing,
             const sif::TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode_)];

  // Alter the destination edges if at a node - loki always gives edges
  // leaving a node, but when a destination we want edges entering the node
  PathLocation dest = update_destinations(graphreader, destination,
                                          costing->GetFilter());

  // Check for trivial path
  mode_ = costing->travelmode();
  auto trivial_id = trivial(origin, dest);
  if (trivial_id.Is_Valid()) {
    std::vector<PathInfo> trivialpath;
    trivialpath.emplace_back(mode_, 0, trivial_id, 0);
    return trivialpath;
  }

  // Check for loop path
  PathInfo loop_edge_info(mode_, 0.0f, loop(origin, dest), 0);

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  Init(origin.vertex(), dest.vertex(), costing);
  float mindist = astarheuristic_.GetDistance(origin.vertex());

  // Initialize the origin and destination locations
  SetOrigin(graphreader, origin, costing, loop_edge_info);
  uint32_t density = SetDestination(graphreader, dest, costing);

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
      // If we had a destination but we were waiting on other possible ones
      if (best_destination_.first != kInvalidLabel) {
        return FormPath(best_destination_.first, loop_edge_info);
      } else {
        // Did not find any destination edges - return empty list of edges
        LOG_ERROR("Route failed after iterations = " +
                     std::to_string(edgelabels_.size()));
        return { };
      }
    }

    // Check for completion. Form path and return if complete.
    if (IsComplete(predindex)) {
      return FormPath(best_destination_.first, loop_edge_info);
    }

    // Remove label from adjacency list, mark it as permanently labeled.
    // Copy the EdgeLabel for use in costing
    EdgeLabel pred = edgelabels_[predindex];
    edgestatus_->Update(pred.edgeid(), kPermanent);

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
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                i++, directededge++, edgeid++) {
      // Handle transition edges they either get skipped or added to the
      // adjacency list using the predecessor info
      if (directededge->trans_up() || directededge->trans_down()) {
          HandleTransitionEdge(level, edgeid, directededge, pred,  predindex);
        continue;
      }

      // Skip shortcut edges when near the destination.
      // TODO - do not think this is needed - moved this out of autocost.
      // If needed should base it on a hierarchy limit...
      if (directededge->is_shortcut() && dist2dest < 10000.0f)
        continue;

      // Skip any superseded edges that match the shortcut mask. Also skip
      // if no access is allowed to this edge (based on costing method)
      if ((shortcuts & directededge->superseded()) ||
          !costing->Allowed(directededge, pred)) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.status.set == kPermanent) {
        continue;
      }

      // Update the_shortcuts mask
      shortcuts |= directededge->shortcut();

      // Get cost
      Cost newcost = pred.cost() +
                     costing->EdgeCost(directededge, nodeinfo->density()) +
                     costing->TransitionCost(directededge, nodeinfo, pred);

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change)
      if (edgestatus.status.set == kTemporary) {
        CheckIfLowerCostPath(edgestatus.status.index, predindex, newcost);
        continue;
      }

      // Find the sort cost (with A* heuristic) using the lat,lng at the
      // end node of the directed edge. Skip if tile not found.
      if ((tile = graphreader.GetGraphTile(directededge->endnode())) == nullptr) {
        continue;
      }
      float dist = astarheuristic_.GetDistance(tile->node(
                directededge->endnode())->latlng());
      float sortcost = newcost.cost + astarheuristic_.Get(dist);

      // Add to the adjacency list and edge labels.
      AddToAdjacencyList(edgeid, sortcost);
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                    newcost, sortcost, dist, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_);
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
  edgestatus_->Set(edgeid, kTemporary, idx);
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
                    const EdgeLabel& pred, const uint32_t predindex) {
  // Skip any transition edges that are not allowed.
  if (!allow_transitions_ ||
      (edge->trans_up() &&
       !hierarchy_limits_[level].AllowUpwardTransition(pred.distance())) ||
      (edge->trans_down() &&
       !hierarchy_limits_[level].AllowDownwardTransition(pred.distance()))) {
    return;
  }

  // Allow the transition edge. Add it to the adjacency list and edge labels
  // using the predecessor information. Transition edges have no length.
  AddToAdjacencyList(edgeid, pred.sortcost());
  edgelabels_.emplace_back(predindex, edgeid,
                edge, pred.cost(), pred.sortcost(), pred.distance(),
                pred.restrictions(), pred.opp_local_idx(), mode_);
}

// Add an edge at the origin to the adjacency list
void PathAlgorithm::SetOrigin(GraphReader& graphreader,
                 const PathLocation& origin,
                 const std::shared_ptr<DynamicCost>& costing,
                 const PathInfo& loop_edge_info) {
  // Get sort heuristic based on distance from origin to destination
  float dist = astarheuristic_.GetDistance(origin.vertex());
  float heuristic = astarheuristic_.Get(dist);

  //we need to do some additional bookkeeping if this path needs to be a loop
  GraphId loop_edge_id = loop_edge_info.edgeid;
  std::vector<baldr::PathLocation::PathEdge> loop_edges;
  Cost loop_edge_cost {0.0f, 0.0f};
  if (loop_edge_id.Is_Valid()) {
    //grab some info about the edge and whats connected to the end of it
    const auto directededge = graphreader.GetGraphTile(loop_edge_id)->directededge(loop_edge_id);
    const auto node_id = directededge->endnode();
    const auto tile = graphreader.GetGraphTile(node_id);
    const auto node_info = tile->node(node_id);
    loop_edge_cost = costing->EdgeCost(directededge, node_info->density()) *
                        (1.f - origin.edges().front().dist);
    //keep information about all the edges leaving the end of this edge
    for(uint32_t edge_index = node_info->edge_index(); edge_index < node_info->edge_index() + node_info->edge_count(); ++edge_index) {
      if(!costing->GetFilter()(tile->directededge(edge_index))) {
        loop_edges.emplace_back(node_id, 0.f);
        loop_edges.back().id.fields.id = edge_index;
      }
    }
  }

  // Iterate through edges and add to adjacency list
  for (const auto& edge : (loop_edges.size() ? loop_edges : origin.edges())) {
    // Get the directed edge
    GraphId edgeid = edge.id;
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get cost and sort cost
    Cost cost = (costing->EdgeCost(directededge, 0) * (1.0f - edge.dist)) + loop_edge_cost;
    float sortcost = cost.cost + heuristic;

    // Add EdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path.
    AddToAdjacencyList(edgeid, sortcost);
    edgelabels_.emplace_back(kInvalidLabel, edgeid, directededge, cost,
            sortcost, dist, directededge->restrictions(),
            directededge->opp_local_idx(), mode_);
  }
}

// Add a destination edge
uint32_t PathAlgorithm::SetDestination(GraphReader& graphreader,
                     const PathLocation& dest,
                     const std::shared_ptr<DynamicCost>& costing) {
  // For each edge
  uint32_t density = 0;
  float seconds = 0.0f;
  for (const auto& edge : dest.edges()) {
    // Keep the id and the cost to traverse the partial distance for the
    // remainder of the edge. This cost is subtracted from the total cost
    // up to the end of the destination edge.
    const GraphTile* tile = graphreader.GetGraphTile(edge.id);
    destinations_[edge.id] = (costing->EdgeCost(tile->directededge(edge.id), 0.0f) *
                (1.0f - edge.dist));

    // Get the tile relative density
    density = tile->header()->density();
  }
  return density;
}

// Test is the shortest path has been found.
bool PathAlgorithm::IsComplete(const uint32_t edge_label_index) {
  //grab the label
  const EdgeLabel& edge_label = edgelabels_[edge_label_index];

  // if we've already found a destination and the search's current edge is
  // more costly to get to, we are done
  if (best_destination_.first != kInvalidLabel &&
      edge_label.cost() > best_destination_.second) {
    return true;
  }

  //check if its a destination
  auto p = destinations_.find(edge_label.edgeid());
  //it is indeed one of the possible destination edges
  if(p != destinations_.end()) {
    // if we didnt have another destination yet or this one is better
    // The cost at this point is to the end of the destination edge.
    // Subtract the partial cost from the end of the destination back to
    // the location to get the partial cost along the edge.
    auto cost = edge_label.cost() - p->second;
    if (best_destination_.first == kInvalidLabel || cost < best_destination_.second) {
      best_destination_.first = edge_label_index;
      best_destination_.second = cost;
    }
    destinations_.erase(p);
    //if we've found all of the destinations we are done looking
    return destinations_.size() == 0;
  }
  return false;
}

// Form the path from the adjacency list.
std::vector<PathInfo> PathAlgorithm::FormPath(const uint32_t dest,
             const PathInfo& loop_edge_info) {
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

  // We had a loop which means we end on the same edge we began
  // this special case can only be handled by adding back the start
  // edge at the end of the path finding because we need to encounter
  // the same edge twice (loop) and the algorithm doesn't allow for this
  if (loop_edge_info.edgeid.Is_Valid()) {
    // Loop edge uses the mode of the last edge found above.
    // TODO - what is the elapsed time on the loop edge?
    path.emplace_back(loop_edge_info);
  }

  // Reverse the list and return
  std:reverse(path.begin(), path.end());
  return path;
}

}
}
