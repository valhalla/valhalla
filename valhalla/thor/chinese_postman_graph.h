#ifndef VALHALLA_THOR_CHINESES_POSTMAN_GRAPH_H_
#define VALHALLA_THOR_CHINESES_POSTMAN_GRAPH_H_

#include "midgard/util.h"
#include "sif/costconstants.h"
#include "thor/worker.h"
#include "tyr/serializers.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>

using namespace valhalla::sif;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {

struct CPVertex {
  std::string vertex_id;
  GraphId graph_id;
  CPVertex(GraphId graph_id = GraphId()) : graph_id(graph_id) {
    vertex_id = std::to_string(graph_id);
  }
};

struct CPEdge {
  float cost;
  CPEdge(float cost = 0) : cost(cost) {
  }
};

// Define the graph with the vertex as a mytuple and the vertices container as a vector
using CPGraph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, CPVertex, valhalla::sif::Cost>;
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

public:
  ChinesePostmanGraph(/* args */);
  ~ChinesePostmanGraph();
  void addVertex(CPVertex cpvertex);
  VertexItr findVertex(CPVertex cpvertex);
  bool isVertexExist(CPVertex cpvertex);
  int numVertices();
  int numEdges();
  void addEdge(CPVertex cpStartVertex, CPVertex cpEndVertex, Cost edge_cost);
  std::map<std::string, int> getUnbalancedVertices();
  std::vector<CPVertex> computeIdealEulerCycle(const CPVertex start_vertex);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_CHINESES_POSTMAN_GRAPH_H_
