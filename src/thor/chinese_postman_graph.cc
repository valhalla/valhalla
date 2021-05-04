#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>

#include "midgard/util.h"
#include "thor/chinese_postman_graph.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {

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

void ChinesePostmanGraph::addVertex(CPVertex cpvertex) {
  if (!this->isVertexExist(cpvertex)) {
    Vertex v = boost::add_vertex(cpvertex, this->G);
    this->vertices[cpvertex.vertex_id] = v;
  }
}

int ChinesePostmanGraph::numVertices() {
  return boost::num_vertices(this->G);
}

int ChinesePostmanGraph::numEdges() {
  return boost::num_edges(this->G);
}

void ChinesePostmanGraph::addEdge(CPVertex cpStartVertex, CPVertex cpEndVertex, CPEdge cpEdge) {
  boost::add_edge(this->vertices[cpStartVertex.vertex_id], this->vertices[cpEndVertex.vertex_id], {},
                  this->G);
}

} // namespace thor
} // namespace valhalla
