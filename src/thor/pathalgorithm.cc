#include <iostream> // TODO remove if not needed
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
std::cout << "Orig LL = " << origll.lat() << "," << origll.lng() << std::endl;
std::cout << "Dest LL = " << destll.lat() << "," << destll.lng() << std::endl;
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
std::cout << "AdjList - mincost= " << mincost << std::endl;
  adjacencylist_ = new AdjacencyList(mincost, range, bucketsize);
  edgestatus_ = new EdgeStatus();
}

// Form path.
// TODO - need to pass in origin and destination directed edges
std::vector<GraphId> PathAlgorithm::GetBestPath(const PathLocation& origin,
             const PathLocation& dest, GraphReader& graphreader,
             EdgeCost* edgecost) {
  // Initialize - create adjacency list, edgestatus support, A*, etc.
  Init(origin.location_.latlng_, dest.location_.latlng_, edgecost);

  // Initialize the origin and destination locations
  SetOrigin(graphreader, origin, edgecost);
  SetDestination(dest);

  std::cout << "Add dest edge: " << dest.edges_[0].id_.tileid() << "," << dest.edges_[0].id_.id() << std::endl;
  GraphTile* t = graphreader.GetGraphTile(dest.edges_[0].id_);
  const DirectedEdge* d = t->directededge(dest.edges_[0].id_);
  const NodeInfo* en  = t->node(d->endnode());
  std::cout << "   Length = " << d->length() << " EndNode LL = " << en->latlng().lat() << "," << en->latlng().lng() << std::endl;


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

    //TODO: make a class that extends std::exception, with messages and error codes and return
    //the appropriate one here
    if(next == nullptr)
      throw std::runtime_error("No path could be found for input");

    RemoveFromAdjMap(next->edgeid());
    edgestatus_->Set(next->edgeid(), kPermanent);
//std::cout << "Next from adj list: " << next->edgeid().tileid() << "," << next->edgeid().id() <<
//    " cost = " << next->truecost() << " sortcost = " << next->sortcost()  << std::endl;

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
        prior_edge_label = GetPriorEdgeLabel(edgeid);
        if (prior_edge_label != nullptr) {
          if (cost < prior_edge_label->truecost()) {
            float prior_sort_cost = prior_edge_label->sortcost();
            prior_edge_label->Update(next, cost, sortcost);
            adjacencylist_->DecreaseCost(prior_edge_label, prior_sort_cost);
          }
        }
        continue;
      }

      // Find the sort cost (with A* heuristic) and add this directed
      // edge to the adjacency list. Set the edge status to temporary
      // (also stores a pointer to edge label)
      endnode  = graphreader.GetGraphTile(directededge->endnode())->node(directededge->endnode());
      sortcost = cost + astarheuristic_.Get(endnode->latlng());
      edgelabel = new EdgeLabel(next, edgeid, directededge->endnode(),
                          cost, sortcost);
      adjacencylist_->Add(edgelabel);
      adjlistedges_[edgeid.value()] = edgelabel;

//std::cout << "     Length = " << directededge->length() << " EndNode LL = " << endnode->latlng().lat() << "," << endnode->latlng().lng() << std::endl;
//std::cout << "     Add to adj list: " << edgeid.tileid() << "," << edgeid.id() << " cost = " << cost << " sortcost = " << sortcost << std::endl;

      edgestatus_->Set(edgeid, kTemporary);
    }
  }

  // Failure! Return empty list of edges
  std::vector<GraphId> empty;
  return empty;
}

// Add an edge at the origin to the adjacency list
void PathAlgorithm::SetOrigin(baldr::GraphReader& graphreader,
          const PathLocation& origin, EdgeCost* edgecost) {
std::cout << "In SetOrigin" << std::endl;
  // Get sort heuristic based on distance from origin to destination
  float heuristic = astarheuristic_.Get(origin.location_.latlng_);

  // Iterate through edges and add to adjacency list
  for (auto edge : origin.edges_) {
    // Get the directed edge
    GraphId edgeid = edge.id_;
    GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get cost and sort cost
    float cost = edgecost->Get(directededge);
    float sortcost = cost + heuristic;
/**
std::cout << "Add origin edge: " << edgeid.tileid() << "," << edgeid.id() << " cost = " << cost << " sortcost = " << sortcost << std::endl;
std::cout << "   End Node ID = " << directededge->endnode().tileid() << "," << directededge->endnode().id() << std::endl;
tile = graphreader.GetGraphTile(directededge->endnode());
const NodeInfo* endnode  = tile->node(directededge->endnode());
std::cout << "   Length = " << directededge->length() << " EndNode LL = " << endnode->latlng().lat() << "," << endnode->latlng().lng() << std::endl;
**/
    // Add EdgeLabel to the adjacency list. Set the predecessor edge to null
    // to indicate the origin of the path.
    adjacencylist_->Add(new EdgeLabel(nullptr, edgeid,
            directededge->endnode(), cost, sortcost));
  }
}

// Add a destination edge
void PathAlgorithm::SetDestination(const PathLocation& dest) {
  // TODO - add partial distances
  for (auto edge : dest.edges_) {
    destinations_[edge.id_.value()] = 1.0f;
  }
}

// Test is the shortest path has been found.
bool PathAlgorithm::IsComplete(const baldr::GraphId& edgeid) {
  // TODO - if destination is along an edge and the edge allows
  // travel in both directions we need to make sure both directions
  // are found or some further cost is encountered to rule out the
  // other direction
  auto p = destinations_.find(edgeid.value());
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
  auto p = adjlistedges_.find(edgeid.value());
  return (p == adjlistedges_.end()) ? nullptr : p->second;
}

// Remove the edge label from the map of edges in the adjacency list
void PathAlgorithm::RemoveFromAdjMap(const GraphId& edgeid) {
  auto p = adjlistedges_.find(edgeid.value());
  if (p != adjlistedges_.end()) {
    adjlistedges_.erase(p);
  }
}

}
}
