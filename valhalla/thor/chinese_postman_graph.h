#ifndef VALHALLA_THOR_CHINESES_POSTMAN_GRAPH_H_
#define VALHALLA_THOR_CHINESES_POSTMAN_GRAPH_H_

#include "midgard/util.h"
#include "sif/costconstants.h"
#include "thor/worker.h"
#include "tyr/serializers.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <stack>

using namespace valhalla::sif;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {

constexpr double NOT_CONNECTED{9999.0};

struct CPVertex {
  std::string vertex_id;
  GraphId graph_id;
  CPVertex(GraphId graph_id = GraphId()) : graph_id(graph_id) {
    vertex_id = std::to_string(graph_id);
  }
};

struct CPEdge {
  Cost cost;
  GraphId graph_id;
  CPEdge(Cost cost = Cost(0, 0), GraphId graph_id = GraphId()) : cost(cost), graph_id(graph_id) {
  }
};

typedef std::vector<std::pair<int, int>> ExtraPaths;

// Define the graph with the vertex as a mytuple and the vertices container as a vector
using CPGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, CPVertex, CPEdge>;
using Vertex = boost::graph_traits<CPGraph>::vertex_descriptor;              // Define Vertex
using VertexItr = boost::graph_traits<CPGraph>::vertex_iterator;             // Define Vertex iterator
using Edge = std::pair<boost::graph_traits<CPGraph>::edge_descriptor, bool>; // Define Edge
using EdgeItr = boost::graph_traits<CPGraph>::edge_iterator;                 // Define Edge Iterator
using DegreeSizeType = boost::graph_traits<CPGraph>::degree_size_type; // Define Degree Size Type

/**
 * Graph representation for solving chinese postman problem
 */

class ChinesePostmanGraph {
private:
  /* data */
  CPGraph G;
  std::map<std::string, Vertex> vertices;
  // store the indegree and outdegree for each node, updated when an edge is added.
  std::map<std::string, int> indegrees;
  std::map<std::string, int> outdegrees;

  // Used for Hiezholer's algorithm
  std::vector<int> reversedEulerPath; // Storing euler path
  std::map<int, int> outEdges;        // Storing the number of outedges from a node index
  std::map<int, std::vector<int>>
      expandedAdjacencyList; // Storing the adjacency list after being expanded

public:
  ChinesePostmanGraph(/* args */);
  ~ChinesePostmanGraph();
  void addVertex(CPVertex cpvertex);
  VertexItr findVertex(CPVertex cpvertex);
  int getVertexIndex(CPVertex cpvertex);
  int getVertexIndex(GraphId graphID);
  bool isVertexExist(CPVertex cpvertex);
  int numVertices();
  int numEdges();
  void addEdge(CPVertex cpStartVertex, CPVertex cpEndVertex, CPEdge cpEdge);
  // Return map of graph id (as string) as the key and the difference between indegree and outdegree
  std::map<std::string, int> getUnbalancedVertices();
  std::vector<GraphId> computeIdealEulerCycle(const CPVertex start_vertex,
                                              ExtraPaths extraPaths = ExtraPaths());
  void setupDFSEulerCycle(ExtraPaths extraPaths = ExtraPaths());
  std::map<int, std::vector<int>> getAdjacencyList(ExtraPaths extraPaths = ExtraPaths());
  void dfsEulerCycle(int startNodeIndex);
  CPEdge* getCPEdge(int i, int j);
  bool isIdealGraph(CPVertex start_vertex, CPVertex end_vertex);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CHINESES_POSTMAN_GRAPH_H_
