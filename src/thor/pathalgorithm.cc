#include <map>
#include <algorithm>
#include "thor/pathalgorithm.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// Default constructor
PathAlgorithm::PathAlgorithm()
    : adjacencylist_(nullptr),
      edgestatus_(nullptr) {
}

// Destructor
PathAlgorithm::~PathAlgorithm() {
  // Need to clean up all the allocated edge labels!
  // Is there a better way to manage?

  // Delete all edgelabels from the done set
  for (auto label : doneset_) {
    delete label;
  }

  if (adjacencylist_ != nullptr) {
    delete adjacencylist_;
  }
  if (edgestatus_ != nullptr) {
    delete edgestatus_;
  }
}

// Clear the temporary information generated during path construction.
void PathAlgorithm::Clear() {
  // Delete all edgelabels from the done set
  for (auto label : doneset_) {
    delete label;
  }

  // Clear elements from the adjacency list
  adjacencylist_->Clear();
}

void PathAlgorithm::Init(const PointLL& origll, const PointLL& destll,
                EdgeCost* edgecost) {
  // Set the destination and cost factor in the A* heuristic
  astarheuristic_.Init(destll, edgecost->AStarCostFactor());

  // Get the initial cost based on A* heuristic from origin
  float mincost = astarheuristic_.Get(origll);

  // Construct adjacency list, edge status, and done set
  // TODO - adjust cost range and bucket size - perhaps from EdgeCost
  // virtual methods
  // Pedestrian - based on distance (km)
  // Use 2m buckets to a range of 10km (5000 buckets)
  float bucketsize = 0.002;
  float range = 10.0f;
  adjacencylist_ = new AdjacencyList(mincost, range, bucketsize);

  edgestatus_ = new EdgeStatus();
}

// Form path.
// TODO - need to pass in origin and destination directed edges
std::vector<GraphId> PathAlgorithm::GetBestPath(GraphReader& graphreader,
                     EdgeCost* edgecost) {
  // Initialize - create adjacency list, edgestatus support, A*, etc.
  PointLL origll, destll; // TODO - get from location info
  Init(origll, destll, edgecost);

  // Add initial directed edges to adjacency list
  std::vector<GraphId> origin_edges;  // TODO (input variable)
  for (auto origin : origin_edges) {
    AddOriginEdge(graphreader, origin, edgecost);
  }

  // Set up destination(s)
  std::vector<GraphId> dest_edges;  // TODO (input variable)
  for (auto dest : dest_edges) {
    AddDestinationEdge(dest);
  }

  // Find shortest path
  float cost, sortcost;
  GraphId node;
  EdgeLabel* next;
  EdgeLabel* edgelabel;
  EdgeLabel* prior_edge_label;
  const NodeInfo* endnode;
  const GraphTile* tile;
  const DirectedEdge* directededge;
  EdgeStatusType edgestatus;
  GraphId edgeid;
  while (true) {
    // Get next element from adjacency list
    next = adjacencylist_->Remove();
    RemoveFromAdjMap(next->edgeid());

    // Check for completion
    if (IsComplete(next->edgeid())) {
      // Form path and return
      return FormPath(next);
    }

    // TODO - do we need to terminate fruitless searches

    // Expand from end node
    node = next->endnode();
    tile = graphreader.GetGraphTile(node);
    if (tile == nullptr)
      continue;
    endnode = tile->node(node);
    edgeid.Set(node.tileid(), node.level(), endnode->edge_index());
    directededge = tile->directededge(endnode->edge_index());
    for (unsigned int i = 0, n = endnode->edge_count(); i < n;
                i++, directededge++, edgeid++) {
      // Get the current set. Skip this edge if permanently labeled (shortest
      // path already found to this directed edge).
      edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus == kPermanent)
        continue;

      // TODO - Check access (method of the costing???
      // Turn costs/restrictions...
      // Transitions between hierarchy levels...

      // Get cost
      cost = next->truecost() + edgecost->Get(directededge);

      if (edgestatus == kTemporary) {
        // If cost is less than current cost to this edge then we
        // update the predecessor information and decrement the sort cost by
        // the difference in the real costs (the A* heuristic doesn't change)
        prior_edge_label = GetPriorEdgeLabel(next->edgeid());
        if (cost < prior_edge_label->truecost()) {
          float prior_sort_cost = prior_edge_label->sortcost();
          prior_edge_label->Update(next, cost, sortcost);
          adjacencylist_->DecreaseCost(prior_edge_label, prior_sort_cost);
        }
        continue;
      }

      // Find the sort cost (with A* heuristic) and add this directed
      // edge to the adjacency list. Set the edge status to temporary
      // (also stores a pointer to edge label)
      sortcost = cost + astarheuristic_.Get(endnode->latlng());
      edgelabel = new EdgeLabel(next, edgeid, directededge->endnode(),
                          cost, sortcost);
      adjacencylist_->Add(edgelabel);
      adjlistedges_.emplace(edgeid, edgelabel);
      edgestatus_->Set(edgeid, kTemporary);
    }
  }

  // Failure! Return empty list of edges
  std::vector<GraphId> empty;
  return empty;
}

// Add an edge at the origin to the adjacency list
void PathAlgorithm::AddOriginEdge(baldr::GraphReader& graphreader,
          const GraphId& edgeid, EdgeCost* edgecost) {

  // Get the directed edge
  GraphTile* tile = graphreader.GetGraphTile(edgeid);
  const DirectedEdge* edge = tile->directededge(edgeid);

  // Add A* heuristic
  PointLL ll;   // TODO - lat,lng of the end node of the edge?
  float cost = edgecost->Get(edge);
  float sortcost = cost + astarheuristic_.Get(ll);

  // Add EdgeLabel to the adjacency list. Set the predecessor edge to null
  // to indicate the origin of the path.
  adjacencylist_->Add(new EdgeLabel(nullptr, edgeid, edge->endnode(),
                                    cost, sortcost));
}

// Add a destination edge
void PathAlgorithm::AddDestinationEdge(const GraphId& edgeid) {
  // TODO - add partial distances
  destinations_.emplace(edgeid, 1.0f);
}

// Test is the shortest path has been found.
bool PathAlgorithm::IsComplete(const baldr::GraphId& edgeid) {
  // TODO - if destination is along an edge and the edge allows
  // travel in both directions we need to make sure both directions
  // are found or some further cost is encountered to rule out the
  // other direction
  auto p = destinations_.find(edgeid);
  return (p == destinations_.end()) ? false : true;
}

// Form the path from the adjacency list.
// TODO - support partial distances at origin/destination
std::vector<baldr::GraphId> PathAlgorithm::FormPath(const EdgeLabel* dest) {
  // Add the destination edge
  std::vector<GraphId> edgesonpath;
  edgesonpath.push_back(dest->edgeid());
  const EdgeLabel* edgelabel = dest;
  while ((edgelabel = edgelabel->predecessor()) != nullptr) {
    edgesonpath.push_back(edgelabel->edgeid());
  }

  // Reverse the list and return
  std::reverse(edgesonpath.begin(), edgesonpath.end());
  return edgesonpath;
}

// Gets the edge label for an edge that is in the adjacency list.
EdgeLabel* PathAlgorithm::GetPriorEdgeLabel(const GraphId& edgeid) const {
  auto p = adjlistedges_.find(edgeid);
  return (p == adjlistedges_.end()) ? nullptr : p->second;
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
