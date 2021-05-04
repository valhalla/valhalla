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

void thor_worker_t::chinese_postman(Api& request) {

  ChinesePostmanGraph G;

  std::cout << "thor_worker_t::chinese_postman" << std::endl;
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request, "thor_worker_t::isochrones");

  parse_locations(request);
  auto costing = parse_costing(request);
  auto& options = *request.mutable_options();

  auto* co = options.mutable_costing_options(options.costing());
  google::protobuf::RepeatedPtrField<::valhalla::ChinesePostmanEdge> edges = co->chinese_edges();

  // User specified edges to route with percent along (for avoiding PathEdges of locations)
  std::unordered_map<baldr::GraphId, float> chinese_edges_;
  int i = 0;

  // Add chinese edges to internal set
  for (auto& edge : co->chinese_edges()) {
    chinese_edges_.insert({GraphId(edge.id()), edge.percent_along()});
    GraphId start_node = reader->edge_startnode(GraphId(edge.id()));
    GraphId end_node = reader->edge_endnode(GraphId(edge.id()));
    CPVertex start_vertex = CPVertex(start_node);
    G.addVertex(start_vertex);
    CPVertex end_vertex = CPVertex(end_node);
    G.addVertex(end_vertex);
    CPEdge cpedge = CPEdge(1.0);
    G.addEdge(start_vertex, end_vertex, cpedge);
  }

  std::cout << "Num of vertices: " << G.numVertices() << std::endl;
  std::cout << "Num of edges: " << G.numEdges() << std::endl;
  // std::cout << "chinese_edges_ size: " << chinese_edges_.size() << std::endl;
}

} // namespace thor
} // namespace valhalla
