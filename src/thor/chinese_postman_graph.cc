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

bool ChinesePostmanGraph::isVertexExist(CPVertex cpvertex) {
  return this->vertices.count(cpvertex.vertex_id) == 1;
}

VertexItr ChinesePostmanGraph::findVertex(CPVertex cpvertex) {
  VertexItr vi, vi_end;
  for (boost::tie(vi, vi_end) = boost::vertices(G); vi != vi_end; ++vi) {
    if (G[*vi].graph_id == cpvertex.graph_id)
      return vi;
  }
  return vi_end;
}

int ChinesePostmanGraph::getVertexIndex(CPVertex cpvertex) {
  for (int i = 0; i < boost::num_vertices(this->G); i++)
    if (this->G[i].graph_id == cpvertex.graph_id) {
      return i;
    }
  return -1;
}

int ChinesePostmanGraph::getVertexIndex(GraphId graph_id) {
  for (int i = 0; i < boost::num_vertices(this->G); i++)
    if (this->G[i].graph_id == graph_id) {
      return i;
    }
  return -1;
}

void ChinesePostmanGraph::addVertex(CPVertex cpvertex) {
  if (!this->isVertexExist(cpvertex)) {
    Vertex v = boost::add_vertex(cpvertex, this->G);
    this->vertices[cpvertex.vertex_id] = v;
    this->indegrees[cpvertex.vertex_id] = 0;
    this->outdegrees[cpvertex.vertex_id] = 0;
  }
}

int ChinesePostmanGraph::numVertices() {
  return boost::num_vertices(this->G);
}

int ChinesePostmanGraph::numEdges() {
  return boost::num_edges(this->G);
}

void ChinesePostmanGraph::addEdge(CPVertex cpStartVertex, CPVertex cpEndVertex, CPEdge cpEdge) {
  boost::add_edge(this->vertices[cpStartVertex.vertex_id], this->vertices[cpEndVertex.vertex_id],
                  cpEdge, this->G);
  // Update the indegrees and outdegrees
  this->indegrees[cpEndVertex.vertex_id]++;
  this->outdegrees[cpStartVertex.vertex_id]++;
}

std::map<std::string, int> ChinesePostmanGraph::getUnbalancedVerticesMap() {
  std::map<std::string, int> unbalaced_vertices;
  for (auto const& v : this->vertices) {
    if (this->indegrees[v.first] != this->outdegrees[v.first]) {
      unbalaced_vertices[v.first] = this->indegrees[v.first] - this->outdegrees[v.first];
    }
  }
  return unbalaced_vertices;
}

std::vector<baldr::GraphId> ChinesePostmanGraph::getUnbalancedVertices() {
  std::vector<GraphId> unbalaced_vertices;
  std::vector<std::string> unbalaced_graph_ids;
  for (auto const& v : this->vertices) {
    if (this->indegrees[v.first] != this->outdegrees[v.first]) {
      unbalaced_graph_ids.push_back(v.first);
    }
  }
  sort(unbalaced_graph_ids.begin(), unbalaced_graph_ids.end());
  unbalaced_vertices.reserve(unbalaced_graph_ids.size());
  for (auto const& graph_id : unbalaced_graph_ids) {
    unbalaced_vertices.push_back(GraphId(graph_id));
  }
  return unbalaced_vertices;
}

std::vector<int> ChinesePostmanGraph::computeIdealEulerCycle(const CPVertex& start_vertex,
                                                             ExtraPaths extraPaths) {
  LOG_DEBUG("computeIdealEulerCycle");
  int startNodeIndex = this->getVertexIndex(start_vertex);

  this->setupDFSEulerCycle(std::move(extraPaths));
  this->dfsEulerCycle(startNodeIndex);

  int edgeUnvisited = 0;
  // Check if there is unvisited edges (this means, the graph is not strongly connected)
  for (auto const& v : this->outEdges) {
    if (v.second != 0) {
      edgeUnvisited++;
    }
  }
  return this->reversedEulerPath;
}

void ChinesePostmanGraph::setupDFSEulerCycle(ExtraPaths extraPaths) {
  this->reversedEulerPath.clear();
  this->outEdges.clear();
  // populate out edges
  this->expandedAdjacencyList = this->getAdjacencyList(std::move(extraPaths));
  for (const auto& x : this->expandedAdjacencyList) {
    this->outEdges[x.first] = x.second.size();
  }
}

std::map<int, std::vector<int>> ChinesePostmanGraph::getAdjacencyList(ExtraPaths extraPaths) {
  std::map<int, std::vector<int>> adjacency_list;
  boost::graph_traits<CPGraph>::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(this->G); ei != ei_end; ++ei) {
    if (!adjacency_list.count(boost::source(*ei, this->G))) {
      std::vector<int> target_vertices;
      adjacency_list[boost::source(*ei, this->G)] = target_vertices;
    }
    adjacency_list[boost::source(*ei, this->G)].push_back(boost::target(*ei, this->G));
  }
  // Add extra paths
  for (auto ep : extraPaths) {
    adjacency_list[ep.first].push_back(ep.second);
  }
  return adjacency_list;
}

void ChinesePostmanGraph::dfsEulerCycle(int startNodeIndex) {
  while (this->outEdges[startNodeIndex] != 0) {
    int nextNodeIndex =
        this->expandedAdjacencyList[startNodeIndex][this->outEdges[startNodeIndex] - 1];
    this->outEdges[startNodeIndex]--;
    this->expandedAdjacencyList[startNodeIndex].pop_back();
    this->dfsEulerCycle(nextNodeIndex);
  }
  this->reversedEulerPath.push_back(startNodeIndex);
}

CPVertex* ChinesePostmanGraph::getCPVertex(int i) {
  return &this->G[i];
}

CPEdge* ChinesePostmanGraph::getCPEdge(int i, int j) {
  auto e = boost::edge(i, j, this->G);
  if (e.second) {
    return &this->G[e.first];
  } else {
    return nullptr;
  }
}

bool ChinesePostmanGraph::isIdealGraph(CPVertex start_vertex, CPVertex end_vertex) {
  auto unbalancedVertices = this->getUnbalancedVerticesMap();
  if (start_vertex.graph_id == end_vertex.graph_id) {
    return unbalancedVertices.size() == 0;
  } else {
    if (unbalancedVertices.count(start_vertex.vertex_id) &&
        unbalancedVertices.count(end_vertex.vertex_id) && unbalancedVertices.size() == 2) {
      if (unbalancedVertices[start_vertex.vertex_id] == -1 &&
          unbalancedVertices[end_vertex.vertex_id] == 1) {
        return true;
      }
      return false;
    } else {
      return false;
    }
  }
}

} // namespace thor
} // namespace valhalla
