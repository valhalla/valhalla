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

void PathAlgorithm::Init(const PointLL& destll, EdgeCost* edgecost) {
  // Set the destination and cost factor in the A* heuristic
  astarheuristic_.Init(destll, edgecost->AStarCostFactor());

  // Construct adjacency list, edge status, and done set
  // TODO - adjust cost range and bucket size
  unsigned int mincost = 0;
  unsigned int range = 100000;
  unsigned int bucketsize = 4;
  adjacencylist_ = new AdjacencyList(mincost, range, bucketsize);
  edgestatus_ = new EdgeStatus();
}

// Form path.
// TODO - need to pass in origin and destination directed edges
bool PathAlgorithm::GetBestPath(GraphReader& graphreader, EdgeCost* edgecost) {
  // Initialize - create adjacency list, edgestatus support, A*, etc.
  PointLL destll; // TODO - get from location info
  Init(destll, edgecost);

  // Add initial directed edges to adjacency list
  std::vector<DirectedEdge> origin_edges;  // TODO (input variable)
  for (auto origin_edge : origin_edges) {
    AddOriginEdge(origin_edge, edgecost);
  }

  // Set up destination(s)

  // Find shortest path
  float cost, sortcost;
  GraphId node;
  EdgeLabel* next;
  const NodeInfo* endnode;
  const GraphTile* tile;
  const DirectedEdge* directededge;
  GraphId edgeid;
  while (true) {
    // Get next element from adjacency list
    next = adjacencylist_->Remove();

    // Check for completion
    if (IsComplete()) {
      // Form path and return
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
      EdgeStatusType status = edgestatus_->Get(edgeid);
      if (status == kPermanent)
        continue;

      // Check access

      // Turn costs/restrictions...

      // Transitions between hierarchy levels...

      // Get cost plus heuristic. TODO - turn costs
      cost = next->truecost() + edgecost->Get(directededge);

      if (status == kTemporary) {
        // If cost is less than current cost to this edge then we
        // update the predecessor information
        // TODO

        continue;
      }

      // Find the sort cost (with A* heuristic) and add this directed
      // edge to the adjacency list.
      sortcost = cost + astarheuristic_.Get(endnode->latlng());
      EdgeLabel* edgelabel = new EdgeLabel();
      // TODO - set fields in the edge label.
      adjacencylist_->Add(edgelabel);
    }
  }
  return true;
}

// Add an edge at the origin to the adjacency list
void PathAlgorithm::AddOriginEdge(const DirectedEdge& edge, EdgeCost* edgecost) {
  // Get a new edge label. TODO - do we want any sort of memory pool?
  EdgeLabel* edgelabel = new EdgeLabel();
  float cost = edgecost->Get(&edge);
  edgelabel->SetTrueCost(cost);

  // Add A* heuristic
  PointLL ll;   // TODO - lat,lng of the end node of the edge
  float sortcost = cost + astarheuristic_.Get(ll);
  edgelabel->SetSortCost(sortcost);
  edgelabel->SetEndNode(edge.endnode());

  // TODO - set predecessor to nullptr to indicate origin edge
  // TODO - do we need sortcost separate from the edgelabel??
  adjacencylist_->Add(edgelabel);
}

// Test is the shortest path has been found.
bool PathAlgorithm::IsComplete() {
  return false;
}

}
}
