#include <algorithm>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>

#include "midgard/util.h"
#include "sif/costconstants.h"
#include "thor/chinese_postman_graph.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

void thor_worker_t::chinese_postman(Api& request) {

  ChinesePostmanGraph G;
  sif::TravelMode mode_; // Current travel mode
  const std::shared_ptr<sif::DynamicCost> costing_ =
      mode_costing[static_cast<uint32_t>(sif::TravelMode::kBicycle)];

  std::cout << "thor_worker_t::chinese_postman" << std::endl;
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request, "thor_worker_t::isochrones");

  parse_locations(request);
  auto costing = parse_costing(request);
  auto& options = *request.mutable_options();

  auto* co = options.mutable_costing_options(options.costing());
  std::list<std::string> avoid_edge_ids;

  for (auto& avoid_edge : co->avoid_edges()) {
    avoid_edge_ids.push_back(std::to_string(GraphId(avoid_edge.id())));
  }

  // Add chinese edges to internal set
  for (auto& edge : co->chinese_edges()) {
    bool found = (std::find(avoid_edge_ids.begin(), avoid_edge_ids.end(),
                            std::to_string(GraphId(edge.id()))) != avoid_edge_ids.end());
    if (found)
      continue;

    GraphId start_node = reader->edge_startnode(GraphId(edge.id()));
    GraphId end_node = reader->edge_endnode(GraphId(edge.id()));
    CPVertex start_vertex = CPVertex(start_node);
    G.addVertex(start_vertex);
    CPVertex end_vertex = CPVertex(end_node);
    G.addVertex(end_vertex);
    // The cost of an edge is not relevant for the graph since we need to visit all the edges.
    // For a simplicity, I put Cost(1, 1) for it.
    // The cost is only considered when matching the unbalanced nodes.
    G.addEdge(start_vertex, end_vertex, Cost(1, 1));
  }

  std::cout << "Num of vertices: " << G.numVertices() << std::endl;
  std::cout << "Num of edges: " << G.numEdges() << std::endl;

  if (G.getUnbalancedVertices().size() == 0) {
    const CPVertex c = CPVertex(GraphId(1));
    std::vector<CPVertex> cpVerticesOrder = G.computeIdealEulerCycle(c);
    std::cout << "Ideal graph" << std::endl;
  } else {
    std::cout << "Non Ideal graph" << std::endl;
    for (auto const& v : G.getUnbalancedVertices()) {
      std::cout << v.first << " -> " << v.second << std::endl;
    }
  }
}

} // namespace thor
} // namespace valhalla
