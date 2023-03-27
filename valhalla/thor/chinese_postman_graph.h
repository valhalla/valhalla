#ifndef VALHALLA_THOR_CHINESES_POSTMAN_GRAPH_H_
#define VALHALLA_THOR_CHINESES_POSTMAN_GRAPH_H_

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>

#include "baldr/graphid.h"
#include "sif/costconstants.h"

namespace valhalla {
namespace thor {

struct CPEdge {
  sif::Cost cost;
  baldr::GraphId graph_id;
};

typedef std::vector<std::pair<int, int>> ExtraPaths;

// Define the graph
using CPGraph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, baldr::GraphId, CPEdge>;
using Vertex = boost::graph_traits<CPGraph>::vertex_descriptor;              // Define Vertex
using Edge = std::pair<boost::graph_traits<CPGraph>::edge_descriptor, bool>; // Define Edge

/**
 * Graph representation for solving chinese postman problem
 */

class ChinesePostmanGraph {
private:
  CPGraph G;
  std::unordered_map<uint64_t, Vertex> vertices;

  // store the indegree and outdegree for each node, updated when an edge is added.
  std::unordered_map<uint64_t, int> indegrees;
  std::unordered_map<uint64_t, int> outdegrees;

  // Used for Hierholzer's algorithm
  std::vector<int> reversedEulerPath;    // Storing euler path, but reversed
  std::unordered_map<int, int> outEdges; // Storing the number of outedges from a node index
  std::unordered_map<int, std::vector<int>>
      expandedAdjacencyList; // Storing the adjacency list after being expanded

public:
  ChinesePostmanGraph();
  ~ChinesePostmanGraph();

  // Add cpvertex to the graph
  void addVertex(baldr::GraphId cpvertex);
  // Add cpEdge to the graph that connect cpStartVertex to cpEndVertex
  void addEdge(baldr::GraphId cpStartVertex, baldr::GraphId cpEndVertex, CPEdge cpEdge);

  // Get the vertex index in the graph from a CPVertex, return -1 if not found
  int getVertexIndex(baldr::GraphId cpvertex);

  // Check if a CPVertex exists
  bool isVertexExist(baldr::GraphId cpvertex);

  // Return the number of edges in the graph
  int numEdges();

  // Return map of graph id (as string) as the key and the difference between indegree and outdegree
  std::unordered_map<uint64_t, int> getUnbalancedVerticesMap();

  // Return a sorted vector of unbalanced vertices (GraphID)
  std::vector<baldr::GraphId> getUnbalancedVertices();

  // Helper to get the adjacency list from the graph. It returns a map with
  // node-index as the key and a vector of node-indexes from that node in the
  // key as the value
  std::unordered_map<int, std::vector<int>> getAdjacencyList(ExtraPaths extraPaths = ExtraPaths());

  // Compute euler cycle of the graph. Graph is ideal after added by extraPaths.
  // It returns a vector of reversed Euler path, or this->reversedEulerPath
  std::vector<int> computeIdealEulerCycle(const baldr::GraphId& start_vertex,
                                          ExtraPaths extraPaths = ExtraPaths());

  // Setup the DFS for computing the Euler cycle
  void setupDFSEulerCycle(ExtraPaths extraPaths = ExtraPaths());

  // The DFS part of the Euler cycle computation.
  void dfsEulerCycle(int startNodeIndex);

  // Helper function to get CPedge based on the index of the start vertex and
  // end vertex
  CPEdge* getCPEdge(int i, int j);

  // Helper function to get CPVertex based on the index of the vertex
  baldr::GraphId* getCPVertex(int i);

  // Check whether the graph is ideal or not, based on the start and end vertex
  // Graph is ideal if all the nodes are balances for start = end vertex
  // For start != vertex, all nodes are balances and the start vertex has +1
  // outdegree, and end vertex has +1 indegree
  bool isIdealGraph(baldr::GraphId start_vertex, baldr::GraphId end_vertex);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CHINESES_POSTMAN_GRAPH_H_
