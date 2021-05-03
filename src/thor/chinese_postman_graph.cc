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

VertexItr ChinesePostmanGraph::findVertex(CPVertex cpvertex) {
  VertexItr vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(G); vi != vi_end; ++vi) {
    if (G[*vi].graph_id == cpvertex.graph_id)
      return vi;
  }
  return vi_end;
}

void ChinesePostmanGraph::addVertex(CPVertex cpvertex) {
  const auto iter = this->findVertex(cpvertex);
  const auto theEnd = boost::vertices(G).second;
  // not exist
  if (iter == theEnd) {
    boost::add_vertex(cpvertex, this->G);
  }
}

int ChinesePostmanGraph::numVertices() {
  return boost::num_vertices(this->G);
}

} // namespace thor
} // namespace valhalla
