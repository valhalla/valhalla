#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>

#include "thor/chinese_postman_graph.h"
#include "thor/worker.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {

ChinesePostmanGraph::ChinesePostmanGraph() {
}

ChinesePostmanGraph::~ChinesePostmanGraph() {
}

bool ChinesePostmanGraph::isVertexExist(baldr::GraphId cpvertex) {
  return vertices.count(cpvertex);
}

int ChinesePostmanGraph::getVertexIndex(baldr::GraphId cpvertex) {
  for (int i = 0; i < boost::num_vertices(G); i++)
    if (G[i] == cpvertex) {
      return i;
    }
  return -1;
}

void ChinesePostmanGraph::addVertex(baldr::GraphId cpvertex) {
  if (!isVertexExist(cpvertex)) {
    Vertex v = boost::add_vertex(cpvertex, G);
    vertices[cpvertex] = v;
    indegrees[cpvertex] = 0;
    outdegrees[cpvertex] = 0;
  }
}

int ChinesePostmanGraph::numEdges() {
  return boost::num_edges(G);
}

void ChinesePostmanGraph::addEdge(baldr::GraphId cpStartVertex,
                                  baldr::GraphId cpEndVertex,
                                  CPEdge cpEdge) {
  boost::add_edge(vertices[cpStartVertex], vertices[cpEndVertex], cpEdge, G);
  // Update the indegrees and outdegrees
  indegrees[cpEndVertex]++;
  outdegrees[cpStartVertex]++;
}

std::unordered_map<uint64_t, int> ChinesePostmanGraph::getUnbalancedVerticesMap() {
  std::unordered_map<uint64_t, int> unbalanced;
  for (auto const& v : vertices) {
    auto degree = indegrees[v.first] - outdegrees[v.first];
    if (degree != 0) {
      unbalanced[v.first] = degree;
    }
  }
  return unbalanced;
}

std::vector<baldr::GraphId> ChinesePostmanGraph::getUnbalancedVertices() {
  std::vector<GraphId> unbalaced_vertices;
  for (auto const& v : vertices) {
    if (indegrees[v.first] != outdegrees[v.first]) {
      unbalaced_vertices.push_back(GraphId(v.first));
    }
  }
  return unbalaced_vertices;
}

std::vector<int> ChinesePostmanGraph::computeIdealEulerCycle(const baldr::GraphId& start_vertex,
                                                             ExtraPaths extraPaths) {
  LOG_DEBUG("computeIdealEulerCycle");
  int startNodeIndex = getVertexIndex(start_vertex);

  setupDFSEulerCycle(std::move(extraPaths));
  dfsEulerCycle(startNodeIndex);

  int edgeUnvisited = 0;
  // Check if there is unvisited edges (this means, the graph is not strongly connected)
  for (auto const& v : outEdges) {
    if (v.second != 0) {
      edgeUnvisited++;
    }
  }
  return reversedEulerPath;
}

void ChinesePostmanGraph::setupDFSEulerCycle(ExtraPaths extraPaths) {
  reversedEulerPath.clear();
  outEdges.clear();
  // populate out edges
  expandedAdjacencyList = getAdjacencyList(std::move(extraPaths));
  for (const auto& x : expandedAdjacencyList) {
    outEdges[x.first] = x.second.size();
  }
}

std::unordered_map<int, std::vector<int>>
ChinesePostmanGraph::getAdjacencyList(ExtraPaths extraPaths) {
  std::unordered_map<int, std::vector<int>> adjacency_list;
  boost::graph_traits<CPGraph>::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(G); ei != ei_end; ++ei) {
    if (!adjacency_list.count(boost::source(*ei, G))) {
      std::vector<int> target_vertices;
      adjacency_list[boost::source(*ei, G)] = target_vertices;
    }
    adjacency_list[boost::source(*ei, G)].push_back(boost::target(*ei, G));
  }
  // Add extra paths
  for (auto ep : extraPaths) {
    adjacency_list[ep.first].push_back(ep.second);
  }
  return adjacency_list;
}

void ChinesePostmanGraph::dfsEulerCycle(int startNodeIndex) {
  while (outEdges[startNodeIndex] != 0) {
    int nextNodeIndex = expandedAdjacencyList[startNodeIndex][outEdges[startNodeIndex] - 1];
    outEdges[startNodeIndex]--;
    expandedAdjacencyList[startNodeIndex].pop_back();
    dfsEulerCycle(nextNodeIndex);
  }
  reversedEulerPath.push_back(startNodeIndex);
}

GraphId* ChinesePostmanGraph::getCPVertex(int i) {
  return &G[i];
}

CPEdge* ChinesePostmanGraph::getCPEdge(int i, int j) {
  auto e = boost::edge(i, j, G);
  if (e.second) {
    return &G[e.first];
  } else {
    return nullptr;
  }
}

bool ChinesePostmanGraph::isIdealGraph(baldr::GraphId start_vertex, baldr::GraphId end_vertex) {
  // start and end on the same vertex
  if (start_vertex == end_vertex) {
    for (auto const& v : vertices) {
      if (indegrees[v.first] != outdegrees[v.first]) {
        return false;
      }
    }
    return true;
  }

  // if the start and end are unbalanced symmetrically but all others are balanced its still ideal
  if (indegrees[start_vertex] - outdegrees[start_vertex] == -1 &&
      indegrees[end_vertex] - outdegrees[end_vertex] == 1 &&
      std::find_if(vertices.begin(), vertices.end(),
                   [&](const std::pair<uint64_t, Vertex>& v) -> bool {
                     return v.first != start_vertex && v.first != end_vertex &&
                            indegrees[v.first] - outdegrees[v.first] != 0;
                   }) == vertices.end()) {
    return true;
  }

  // not ideal..
  return false;
}

} // namespace thor
} // namespace valhalla
