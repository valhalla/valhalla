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

midgard::PointLL to_ll(const valhalla::Location& l) {
  return midgard::PointLL{l.ll().lng(), l.ll().lat()};
}
midgard::PointLL thor_worker_t::getPointLL(baldr::GraphId node) {
  const NodeInfo* ni_start = reader->nodeinfo(node);
  graph_tile_ptr tile = reader->GetGraphTile(node);
  return ni_start->latlng(tile->header()->base_ll());
}
std::string pointLLToJson(const midgard::PointLL l) {
  // // To be something like this {"lat":40.739735,"lon":-73.979713}
  std::string json = "{";
  json += "\"lat\":";
  json += std::to_string(l.lat());

  json += ",\"lon\":";
  json += std::to_string(l.lng());

  json += "}";

  return json;
}

std::string locationsToJson(std::vector<midgard::PointLL> locations) {
  // To be something like this [{"lat":40.744014,"lon":-73.990508},
  // {"lat":40.739735,"lon":-73.979713}]
  std::string json = "[";
  bool extraCharacter = false;
  for (const auto& location : locations) {
    // std::cout << kv.first << " has value " << kv.second << std::endl;
    json += pointLLToJson(location) + ", ";
    extraCharacter = true;
  }
  // remove last two character ", "
  if (extraCharacter) {
    json.pop_back();
    json.pop_back();
  }

  json += "]";
  return json;
}

std::string thor_worker_t::computeFloydWarshall(std::vector<midgard::PointLL> sources,
                                                std::vector<midgard::PointLL> targets,
                                                std::string costing) {
  Api request;
  // Update request with source and target, also costing
  std::string jsonMatrixRequest = "{\"sources\":" + locationsToJson(sources) +
                                  ", \"targets\":" + locationsToJson(targets) + ",\"costing\":\"" +
                                  costing + "\"}";
  ParseApi(jsonMatrixRequest, Options::sources_to_targets, request);
  std::cout << "matrix result:\n" << matrix(request);
  return matrix(request);
}

void thor_worker_t::chinese_postman(Api& request) {

  auto correlated = request.options().locations();
  auto it = correlated.begin();
  auto origin = &it;
  it++;
  auto destination = &it;

  valhalla::Location originLocation = **origin;
  valhalla::Location destinationLocation = **destination;

  midgard::PointLL originPoint = to_ll(originLocation);
  midgard::PointLL destinationPoint = to_ll(destinationLocation);

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

  bool originNodeFound = false;
  CPVertex originVertex;

  // Add chinese edges to internal set
  for (auto& edge : co->chinese_edges()) {
    // skip the edge if it's not allowed (reverse one way)
    const DirectedEdge* directed_edge = reader->directededge(baldr::GraphId(edge.id()));
    if (!directed_edge->forward()) {
      continue;
    }
    bool found = (std::find(avoid_edge_ids.begin(), avoid_edge_ids.end(),
                            std::to_string(GraphId(edge.id()))) != avoid_edge_ids.end());
    if (found)
      continue;
    GraphId start_node = reader->edge_startnode(GraphId(edge.id()));
    GraphId end_node = reader->edge_endnode(GraphId(edge.id()));
    CPVertex start_vertex = CPVertex(start_node);
    if (!originNodeFound) {
      originVertex = start_vertex;
      originNodeFound = true;
    }
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
    std::vector<CPVertex> cpVerticesOrder = G.computeIdealEulerCycle(originVertex);
    std::cout << "Ideal graph" << std::endl;
  } else {
    std::cout << "Non Ideal graph" << std::endl;
    std::vector<midgard::PointLL> overPoints; // Node that has too many incoming
    std::vector<midgard::PointLL> underPoints;
    std::vector<midgard::PointLL> locations;
    for (auto const& v : G.getUnbalancedVertices()) {
      auto l = getPointLL(GraphId(v.first));
      std::cout << "location (" << v.first << "): " << l.lng() << ", " << l.lat() << std::endl;
      locations.push_back(l);
      if (v.second > 0) {
        overPoints.push_back(l);
      } else if (v.second < 0) {
        underPoints.push_back(l);
      }
    }
    std::string matrixOutput = computeFloydWarshall(overPoints, underPoints, costing);
    std::cout << "matrix output:\n" << matrixOutput;
  }
}

} // namespace thor
} // namespace valhalla
