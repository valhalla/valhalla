#include <iostream> // TODO remove if not needed
#include <map>
#include <algorithm>
#include "thor/pathalgorithm.h"

#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

constexpr uint32_t kBucketCount = 20000;
constexpr uint64_t kInitialEdgeLabelCount = 500000;

// Default constructor
PathAlgorithm::PathAlgorithm()
    : edgelabel_index_(0),
      adjacencylist_(nullptr),
      edgestatus_(nullptr) {
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

  // Clear elements from the adjacency list
  adjacencylist_->Clear();

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
  float bucketsize = costing->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_ = new AdjacencyList(mincost, range, bucketsize);
  edgestatus_ = new EdgeStatus();
}

// Calculate best path.
std::vector<GraphId> PathAlgorithm::GetBestPath(const PathLocation& origin,
             const PathLocation& dest, GraphReader& graphreader,
             std::shared_ptr<DynamicCost> costing) {
  // Initialize - create adjacency list, edgestatus support, A*, etc.
  Init(origin.vertex(), dest.vertex(), costing);

  // Initialize the origin and destination locations
  SetOrigin(graphreader, origin, costing);
  SetDestination(dest);

  // Counts of transitions to upper levels (TEST - TODO better design!)
  uint32_t upto1count = 0;
  uint32_t upto0count = 0;

  // Find shortest path
  uint32_t uturn_index, next_label_index;
  uint32_t prior_label_index;
  float dist2dest, dist;
  float cost, sortcost, currentcost;
  GraphId node;
  const NodeInfo* nodeinfo;
  const GraphTile* tile;
  const DirectedEdge* directededge;
  EdgeStatusType edgestatus;
  GraphId edgeid;
  while (true) {
    // Get next element from adjacency list. Check that it is valid.
    // TODO: make a class that extends std::exception, with messages and
    // error codes and return the appropriate one here
    next_label_index = adjacencylist_->Remove(edgelabels_);
    if (next_label_index == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " + std::to_string(edgelabel_index_));
      throw std::runtime_error("No path could be found for input");
    }

    // Remove label from adjacency list, mark it as done
    const EdgeLabel& nextlabel = edgelabels_[next_label_index];
    RemoveFromAdjMap(nextlabel.edgeid());
    edgestatus_->Set(nextlabel.edgeid(), kPermanent);

    // Check for completion. Form path and return if complete.
    if (IsComplete(nextlabel.edgeid())) {
      return FormPath(next_label_index, graphreader);
    }

    // TODO - do we need to terminate fruitless searches?

    // Get the end node of the prior directed edge and the current distance
    // to the destination.
    node      = nextlabel.endnode();
    dist2dest = nextlabel.distance();

    // Do not expand based on hierarchy level?
    // TODO - come up with rule sets/options/config
    if (nextlabel.trans_up()) {
      if (node.level() == 0) {
        upto0count++;
      } else if (node.level() == 1) {
        upto1count++;
      }
    }

    // Stop expanding the local level once 50 transitions have been made
    if (dist2dest > 10000.0f && node.level() == 2 && upto1count > 50) {
      continue;
    }

    // Stop expanding arterial level once 250 transitions have been made
    if (dist2dest > 50000.0f && node.level() == 1 && upto0count > 250) {
      continue;
    }

    // Skip if tile not found (can happen with regional data sets).
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Set temp variables here. EdgeLabel is not good inside the loop
    // below since the edgelabel list is modified
    nodeinfo =    tile->node(node);
    currentcost = nextlabel.truecost();
    uturn_index = nextlabel.uturn_index();


    // Check access at the node
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    // Expand from end node. Identify if this node has shortcut edges
    // (they occur first so just check the first directed edge).
    edgeid.Set(node.tileid(), node.level(), nodeinfo->edge_index());
    directededge = tile->directededge(nodeinfo->edge_index());
    bool has_shortcuts = false;
    for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                i++, directededge++, edgeid++) {
      // Skip any superseded edges if edges include shortcuts. Also skip
      // if no access is allowed to this edge (based on costing method)
      if ((has_shortcuts && directededge->superseded()) ||
          !costing->Allowed(directededge, (i == uturn_index), dist2dest)) {
        continue;
      }

      // Set the has_shortcuts flag if a shortcut was taken
      has_shortcuts |= directededge->shortcut();

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus == kPermanent) {
        continue;
      }

      // TODO
      // Turn costs/restrictions...
      // Transitions between hierarchy levels...

      // Get cost
      cost = currentcost + costing->Get(directededge);

      // Check if already in adjacency list
      if (edgestatus == kTemporary) {
        // If cost is less than current cost to this edge then we
        // update the predecessor information and decrement the sort cost by
        // the difference in the real costs (the A* heuristic doesn't change)
        prior_label_index = GetPriorEdgeLabel(edgeid);
        if (prior_label_index != kInvalidLabel) {
          if (cost < edgelabels_[prior_label_index].truecost()) {
            float prior_sort_cost = edgelabels_[prior_label_index].sortcost();
            float newsortcost = prior_sort_cost -
                    (edgelabels_[prior_label_index].truecost() - cost);
            // TODO - do we need to update trans_up/trans_down?
            edgelabels_[prior_label_index].Update(next_label_index, cost,
                    newsortcost);
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
      dist = astarheuristic_.GetDistance(tile->node(
                directededge->endnode())->latlng());
      sortcost = cost + astarheuristic_.Get(dist);

      // Add edge label
      edgelabels_.emplace_back(EdgeLabel(next_label_index, edgeid,
               directededge, cost, sortcost, dist));

      // Add to the adjacency list, add to the map of edges in the adj. list
      adjacencylist_->Add(edgelabel_index_, sortcost);
      adjlistedges_[edgeid.value] = edgelabel_index_;
      edgestatus_->Set(edgeid, kTemporary);
      edgelabel_index_++;
    }
  }

  // Failure! Return empty list of edges
  std::vector<GraphId> empty;
  return empty;
}

// Add an edge at the origin to the adjacency list
void PathAlgorithm::SetOrigin(baldr::GraphReader& graphreader,
          const PathLocation& origin, const std::shared_ptr<DynamicCost>& costing) {
  // Get sort heuristic based on distance from origin to destination
  float dist = astarheuristic_.GetDistance(origin.vertex());
  float heuristic = astarheuristic_.Get(dist);

  // Iterate through edges and add to adjacency list
  for (const auto& edge : origin.edges()) {
    // Get the directed edge
    GraphId edgeid = edge.id;
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get cost and sort cost
    float cost = costing->Get(directededge);
    float sortcost = cost + heuristic;

    // Add EdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path.
    edgelabels_.emplace_back(EdgeLabel(kInvalidLabel, edgeid,
            directededge, cost, sortcost, dist));
    adjacencylist_->Add(edgelabel_index_, sortcost);
    edgelabel_index_++;
  }
}

// Add a destination edge
void PathAlgorithm::SetDestination(const PathLocation& dest) {
  // TODO - add partial distances
  for (const auto& edge : dest.edges()) {
    destinations_[edge.id.value] = 1.0f;
  }
}

// Test is the shortest path has been found.
bool PathAlgorithm::IsComplete(const baldr::GraphId& edgeid) {
  // TODO - if destination is along an edge and the edge allows
  // travel in both directions we need to make sure both directions
  // are found or some further cost is encountered to rule out the
  // other direction
  const auto& p = destinations_.find(edgeid.value);
  return (p == destinations_.end()) ? false : true;
}

// Form the path from the adjacency list.
// TODO - support partial distances at origin/destination
std::vector<baldr::GraphId> PathAlgorithm::FormPath(const uint32_t dest,
                     GraphReader& graphreader) {
  // TODO - leave in for now!
  LOG_INFO("PathCost = " + std::to_string(edgelabels_[dest].truecost()) +
           "  Iterations = " + std::to_string(edgelabel_index_));

  // Return path on local level...
/**  if (true) {
    return FormLocalPath(dest, graphreader);
  } **/

  // Add the destination edge
  std::vector<GraphId> edgesonpath;
  edgesonpath.push_back(edgelabels_[dest].edgeid());
  uint32_t edgelabel_index = dest;
  while ((edgelabel_index = edgelabels_[edgelabel_index].predecessor()) !=
              kInvalidLabel) {
    edgesonpath.emplace_back(edgelabels_[edgelabel_index].edgeid());
  }

  // Reverse the list and return
  std::reverse(edgesonpath.begin(), edgesonpath.end());
  return edgesonpath;
}

// Gets the edge label for an edge that is in the adjacency list.
uint32_t PathAlgorithm::GetPriorEdgeLabel(const GraphId& edgeid) const {
  const auto& p = adjlistedges_.find(edgeid.value);
  return (p == adjlistedges_.end()) ? kInvalidLabel : p->second;
}

// Remove the edge label from the map of edges in the adjacency list
void PathAlgorithm::RemoveFromAdjMap(const GraphId& edgeid) {
  auto p = adjlistedges_.find(edgeid.value);
  if (p != adjlistedges_.end()) {
    adjlistedges_.erase(p);
  }
}

}
}
