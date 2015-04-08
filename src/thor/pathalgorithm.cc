#include <iostream> // TODO remove if not needed
#include <map>
#include <algorithm>
#include "thor/pathalgorithm.h"

#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;

// TODO: make a class that extends std::exception, with messages and
// error codes and return the appropriate error codes

namespace {

constexpr uint32_t kBucketCount = 20000;
constexpr uint64_t kInitialEdgeLabelCount = 500000;

GraphId trivial(const PathLocation& origin, const PathLocation& destination) {
  //check if any of the pairs of origin and destination edges could be a trivial path
  //NOTE: it is true that there could be a shorter path by leaving this edge and coming
  //back in the other direction however this should be uncommon
  for(const auto& origin_edge : origin.edges()) {
    for(const auto& destination_edge : destination.edges()) {
      //same id and the origin shows up at the beginning of the edge
      //while the destination shows up at the end of the edge
      if(origin_edge.id == destination_edge.id && origin_edge.dist <= destination_edge.dist) {
        return origin_edge.id;
      }
    }
  }
  return {};
}

GraphId loop(const PathLocation& origin, const PathLocation& destination) {
  //if we end up with locations where there is a trivial path but you would have to
  //traverse the edge in reverse to do it we need to mark this as a loop so that
  //the initial edge can be considered on the path at some later point in time
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

}

namespace valhalla {
namespace thor {

// Default constructor
PathAlgorithm::PathAlgorithm()
    : allow_transitions_(false),
      edgelabel_index_(0),
      adjacencylist_(nullptr),
      edgestatus_(nullptr),
      best_destination_{kInvalidLabel, std::numeric_limits<float>::max()}{
  edgelabels_.reserve(kInitialEdgeLabelCount);
}

// Destructor
PathAlgorithm::~PathAlgorithm() {
  Clear();
}

// Clear the temporary information generated during path construction.
void PathAlgorithm::Clear() {
  // Set the edge label index back to 0
  edgelabel_index_ = 0;
  edgelabels_.clear();
  best_destination_ = std::make_pair(kInvalidLabel, std::numeric_limits<float>::max());

  // Clear elements from the adjacency list
  if(adjacencylist_ != nullptr) {
    adjacencylist_->Clear();
    adjacencylist_ = nullptr;
  }

  // Clear the edge status flags
  if (edgestatus_ != nullptr) {
    delete edgestatus_;
    edgestatus_ = nullptr;
  }

  // Clear the map of adjacency list edges
  adjlistedges_.clear();
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
  adjacencylist_ = new AdjacencyList(mincost, range, bucketsize);
  edgestatus_ = new EdgeStatus();

  // Get hierarchy limits from the costing. Get a copy since we increment
  // transition counts (i.e., this is not a const reference).
  allow_transitions_ = costing->AllowTransitions();
  hierarchy_limits_  = costing->GetHierarchyLimits();
}

// Calculate best path.
std::vector<GraphId> PathAlgorithm::GetBestPath(const PathLocation& origin,
             const PathLocation& dest, GraphReader& graphreader,
             const std::shared_ptr<DynamicCost>& costing) {
  // Check for trivial path
  auto trivial_id = trivial(origin, dest);
  if(trivial_id.Is_Valid())
    return {trivial_id};

  // Check for loop path
  auto loop_edge = loop(origin, dest);

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  Init(origin.vertex(), dest.vertex(), costing);
  float mindist = astarheuristic_.GetDistance(origin.vertex());

  // Initialize the origin and destination locations
  SetOrigin(graphreader, origin, costing, loop_edge);
  SetDestination(graphreader, dest, costing);

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
      if(best_destination_.first != kInvalidLabel)
        return FormPath(best_destination_.first, graphreader, loop_edge);

      // We didn't find any destination edge - return empty list of edges
      LOG_ERROR("Route failed after iterations = " +
                   std::to_string(edgelabel_index_));
 //     throw std::runtime_error("No path could be found for input");
      return { };
    }

    // Remove label from adjacency list, mark it as done - copy the EdgeLabel
    // for use in costing
    EdgeLabel pred = edgelabels_[predindex];
    RemoveFromAdjMap(pred.edgeid());
    edgestatus_->Set(pred.edgeid(), kPermanent);

    // Check for completion. Form path and return if complete.
    if (IsComplete(predindex)) {
      return FormPath(best_destination_.first, graphreader, loop_edge);
    }

    // Get the end node of the prior directed edge and the current distance
    // to the destination.
    GraphId node    = pred.endnode();
    float dist2dest = pred.distance();
    uint32_t level  = node.level();

    // Check that distance is converging towards the destination. Return route
    // failure if no convergence for TODO iterations
    if (dist2dest < mindist) {
      mindist = dist2dest;
      nc = 0;
    } else if (nc++ > 500000) {
      return {};
    }

    // Check hierarchy. Count upward transitions (counted on the level
    // transitioned from). Do not expand based on hierarchy level based on
    // number of upward transitions and distance to the destination
    if (pred.trans_up()) {
      hierarchy_limits_[level+1].up_transition_count++;
    }
    if (hierarchy_limits_[level].StopExpanding(dist2dest)) {
      continue;
    }

    // Skip if tile not found (can happen with regional data sets).
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Check access at the node
    const NodeInfo* nodeinfo = tile->node(node);
    if (!costing->Allowed(nodeinfo)) {
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
      EdgeStatusType edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus == kPermanent) {
        continue;
      }

      // Update the_shortcuts mask
      shortcuts |= directededge->shortcut();

      // Get cost
      Cost edgecost = costing->EdgeCost(directededge, nodeinfo->density()) +
                      costing->TransitionCost(directededge, nodeinfo, pred, i);
      float cost = pred.cost() + edgecost.cost;

      // Check if already in adjacency list
      if (edgestatus == kTemporary) {
        // If cost is less than current cost to this edge then we
        // update the predecessor information and decrement the sort cost by
        // the difference in the real costs (the A* heuristic doesn't change)
        uint32_t prior_label_index = GetPriorEdgeLabel(edgeid);
        if (prior_label_index != kInvalidLabel) {
          // Reduce if cost difference is outside the costing unit size
          if (cost < edgelabels_[prior_label_index].cost()) {
            float prior_sort_cost = edgelabels_[prior_label_index].sortcost();
            float newsortcost = prior_sort_cost -
                    (edgelabels_[prior_label_index].cost() - cost);
            // TODO - do we need to update trans_up/trans_down?
            edgelabels_[prior_label_index].Update(predindex, cost, newsortcost);
            adjacencylist_->DecreaseCost(prior_label_index, newsortcost,
                    prior_sort_cost);
          }
        }
        continue;
      }

      // Find the sort cost (with A* heuristic) using the lat,lng at the
      // end node of the directed edge. Skip if tile not found.
      if ((tile = graphreader.GetGraphTile(directededge->endnode())) == nullptr) {
        continue;
      }
      float dist = astarheuristic_.GetDistance(tile->node(
                directededge->endnode())->latlng());
      float sortcost = cost + astarheuristic_.Get(dist);

      // Add edge label
      edgelabels_.emplace_back(EdgeLabel(predindex, edgeid, directededge, cost,
                        sortcost, dist, directededge->restrictions(),
                        directededge->opp_local_idx()));

      // Add to the adjacency list, add to the map of edges in the adj. list
      adjacencylist_->Add(edgelabel_index_, sortcost);
      adjlistedges_[edgeid] = edgelabel_index_;
      edgestatus_->Set(edgeid, kTemporary);
      edgelabel_index_++;
    }
  }

  // Failure! Return empty list of edges
  return {};
}

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

  // Allow the transition edge. Add it to the adjacency list using the
  // predecessor information. Transition edges have no length.
  edgelabels_.emplace_back(EdgeLabel(predindex, edgeid,
                edge, pred.cost(), pred.sortcost(), pred.distance(),
                pred.restrictions(), pred.opp_local_idx()));

  // Add to the adjacency list, add to the map of edges in the adj. list
  adjacencylist_->Add(edgelabel_index_, pred.sortcost());
  adjlistedges_[edgeid] = edgelabel_index_;
  edgestatus_->Set(edgeid, kTemporary);
  edgelabel_index_++;
}

// Add an edge at the origin to the adjacency list
void PathAlgorithm::SetOrigin(GraphReader& graphreader,
          const PathLocation& origin, const std::shared_ptr<DynamicCost>& costing, const GraphId& loop_edge_id) {
  // Get sort heuristic based on distance from origin to destination
  float dist = astarheuristic_.GetDistance(origin.vertex());
  float heuristic = astarheuristic_.Get(dist);

  //we need to do some additional bookkeeping if this path needs to be a loop
  std::vector<baldr::PathLocation::PathEdge> loop_edges;
  float loop_edge_cost = 0.f;
  if(loop_edge_id.Is_Valid()) {
    //grab some info about the edge and whats connected to the end of it
    const auto node_id = graphreader.GetGraphTile(loop_edge_id)->directededge(loop_edge_id)->endnode();
    const auto tile = graphreader.GetGraphTile(node_id);
    const auto node_info = tile->node(node_id);
    loop_edge_cost = costing->EdgeCost(tile->directededge(loop_edge_id), node_info->density()).cost *
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
    float cost = costing->EdgeCost(directededge, 0).cost * (1.f - edge.dist) + loop_edge_cost;
    float sortcost = cost + heuristic;

    // Add EdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path.
    edgelabels_.emplace_back(EdgeLabel(kInvalidLabel, edgeid,
            directededge, cost, sortcost, dist, 0,
            directededge->opp_local_idx()));
    adjacencylist_->Add(edgelabel_index_, sortcost);
    edgelabel_index_++;
  }
}

// Add a destination edge
void PathAlgorithm::SetDestination(GraphReader& graphreader, const PathLocation& dest, const std::shared_ptr<DynamicCost>& costing) {
  // For each edge
  float seconds = 0.0f;
  for (const auto& edge : dest.edges()) {
    // Keep the id and the cost to traverse the partial distance
    const GraphTile* tile = graphreader.GetGraphTile(edge.id);
    destinations_[edge.id] = costing->EdgeCost(tile->directededge(edge.id), 0).cost * edge.dist;
  }
}

// Test is the shortest path has been found.
bool PathAlgorithm::IsComplete(const uint32_t edge_label_index) {
  //grab the label
  const EdgeLabel& edge_label = edgelabels_[edge_label_index];
  //if we've already found a destination and the search's current edge is more costly to get to, we are done
  if(best_destination_.first != kInvalidLabel && edge_label.cost() > best_destination_.second)
    return true;
  //check if its a destination
  auto p = destinations_.find(edge_label.edgeid());
  //it is indeed one of the possible destination edges
  if(p != destinations_.end()) {
    //if we didnt have another destination yet or this one is better
    auto cost = edge_label.cost() + p->second;
    if(best_destination_.first == kInvalidLabel || cost < best_destination_.second){
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
// TODO - support partial distances at origin/destination
std::vector<baldr::GraphId> PathAlgorithm::FormPath(const uint32_t dest,
                     GraphReader& graphreader, const GraphId& loop_edge) {
  // TODO - leave in for now!
  LOG_INFO("PathCost = " + std::to_string(edgelabels_[dest].cost()) +
           "  Iterations = " + std::to_string(edgelabel_index_));

  // Return path on local level...
/**  if (true) {
    return FormLocalPath(dest, graphreader);
  } **/

  // Add the destination edge
  std::vector<GraphId> edges;
  edges.reserve(edgelabels_.size());
  // Work backwards from the destination
  for(auto edgelabel_index = dest; edgelabel_index != kInvalidLabel; edgelabel_index = edgelabels_[edgelabel_index].predecessor()) {
    edges.push_back(edgelabels_[edgelabel_index].edgeid());
  }
  // We had a loop which means we end on the same edge we began
  // this special case can only be handled by adding back the start
  // edge at the end of the path finding because we need to encounter
  // the same edge twice (loop) and the algorithm doesn't allow for this
  if(loop_edge.Is_Valid())
    edges.push_back(loop_edge);

  // Reverse the list and return
  std:reverse(edges.begin(), edges.end());
  return edges;
}

// Gets the edge label for an edge that is in the adjacency list.
uint32_t PathAlgorithm::GetPriorEdgeLabel(const GraphId& edgeid) const {
  const auto& p = adjlistedges_.find(edgeid);
  return (p == adjlistedges_.end()) ? kInvalidLabel : p->second;
}

// Remove the edge label from the map of edges in the adjacency list
void PathAlgorithm::RemoveFromAdjMap(const GraphId& edgeid) {
  auto p = adjlistedges_.find(edgeid);
  if (p != adjlistedges_.end()) {
    adjlistedges_.erase(p);
  }
}

}
}
