#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <stack>

#include "midgard/util.h"
#include "thor/chinese_postman_graph.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {

void printStack(std::stack<int> s) {
  if (s.empty()) {
    return;
  }
  int x = s.top();

  s.pop();

  printStack(s);

  std::cout << x << ", ";

  s.push(x);
}

void printAdjacencyList(std::map<int, std::vector<int>> adjacency_list) {
  for (const auto& x : adjacency_list) {
    std::cout << x.first << ": ";
    for (const auto& y : x.second)
      std::cout << y << ", ";
    std::cout << std::endl;
  }
}

ChinesePostmanGraph::ChinesePostmanGraph(/* args */) {
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

std::map<std::string, int> ChinesePostmanGraph::getUnbalancedVertices() {
  std::map<std::string, int> unbalaced_vertices;
  for (auto const& v : this->vertices) {
    if (this->indegrees[v.first] != this->outdegrees[v.first]) {
      unbalaced_vertices[v.first] = this->indegrees[v.first] - this->outdegrees[v.first];
      ;
    }
  }
  return unbalaced_vertices;
}

std::vector<CPEdge> ChinesePostmanGraph::computeIdealEulerCycle(const CPVertex start_vertex) {
  int startNodeIndex = this->getVertexIndex(start_vertex);
  int edgeVisited = 0;

  this->setupDFSEulerCycle();
  this->dfsEulerCycle(startNodeIndex);

  std::cout << "Expanded adjancency list: " << std::endl;
  printAdjacencyList(this->expandedAdjacencyList);

  // Check if there is unvisited edges (this means, the graph is not strongly connected)
  for (auto const& v : this->outEdges) {
    if (v.second != 0) {
      throw valhalla_exception_t(450);
    }
  }
  std::cout << "Euler path (node IDs): ";
  std::vector<CPVertex> eulerPathVertices;
  std::vector<CPEdge> eulerPathEdges;
  // for (int i = 0; i < (this->reversedEulerPath.size() - 1); ++i){
  //   std::cout << i << ": " << this->G[i].graph_id << std::endl;
  //   // std::cout << boost::edge(i, i+1, this->G).first << " " << boost::edge(i, i+1,
  //   this->G).second << std::endl;
  //   // VertexItr vi_start = this->findVertex()
  // }
  for (auto it = this->reversedEulerPath.rbegin(); it != this->reversedEulerPath.rend(); ++it) {
    if (it + 1 == this->reversedEulerPath.rend()) {
      continue;
    }
    // eulerPathVertices.push_back(this->G[*it]);
    auto e = boost::edge(*it, *(it + 1), G);
    eulerPathEdges.push_back(this->G[e.first]);
  }
  std::cout << "\n";
  for (auto v : eulerPathVertices) {
    std::cout << v.graph_id << ", ";
  }
  std::cout << std::endl;
  return eulerPathEdges;
}

void ChinesePostmanGraph::setupDFSEulerCycle() {
  this->reversedEulerPath.clear();
  this->outEdges.clear();
  // populate out edges
  this->expandedAdjacencyList = this->getAdjacencyList();
  for (const auto& x : this->expandedAdjacencyList) {
    this->outEdges[x.first] = x.second.size();
  }
}

std::map<int, std::vector<int>> ChinesePostmanGraph::getAdjacencyList() {
  std::map<int, std::vector<int>> adjacency_list;
  boost::graph_traits<CPGraph>::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(this->G); ei != ei_end; ++ei) {
    if (!adjacency_list.count(boost::source(*ei, this->G))) {
      std::vector<int> target_vertices;
      adjacency_list[boost::source(*ei, this->G)] = target_vertices;
    }
    adjacency_list[boost::source(*ei, this->G)].push_back(boost::target(*ei, this->G));
  }
  return adjacency_list;
}

void ChinesePostmanGraph::dfsEulerCycle(int startNodeIndex) {
  while (this->outEdges[startNodeIndex] != 0) {
    int nextNodeIndex =
        this->expandedAdjacencyList[startNodeIndex][this->outEdges[startNodeIndex] - 1];
    this->outEdges[startNodeIndex]--;
    this->dfsEulerCycle(nextNodeIndex);
  }
  this->reversedEulerPath.push_back(startNodeIndex);
}

} // namespace thor
} // namespace valhalla
