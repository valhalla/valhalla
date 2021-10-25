#include <algorithm>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/multi_array.hpp>
#include <boost/multi_array/subarray.hpp>

#include "loki/search.h"
#include "midgard/util.h"
#include "sif/costconstants.h"
#include "sif/recost.h"
#include "thor/Hungarian.h"
#include "thor/chinese_postman_graph.h"
#include "thor/costmatrix.h"
#include "thor/worker.h"
#include "tyr/serializers.h"
#include <thor/bidirectional_astar.h>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

// typedef DistanceMatrix::index DistanceMatrixIndex;

typedef boost::multi_array<std::vector<int>, 2> PathMatrix;
typedef PathMatrix::index PathMatrixIndex;

midgard::PointLL to_ll(const valhalla::Location& l) {
  return midgard::PointLL{l.ll().lng(), l.ll().lat()};
}

midgard::PointLL getPointLL(baldr::GraphId node, GraphReader& reader) {
  const NodeInfo* ni_start = reader.nodeinfo(node);
  graph_tile_ptr tile = reader.GetGraphTile(node);
  return ni_start->latlng(tile->header()->base_ll());
}

inline float find_percent_along(const valhalla::Location& location, const GraphId& edge_id) {
  for (const auto& e : location.path_edges()) {
    if (e.graph_id() == edge_id)
      return e.percent_along();
  }
  throw std::logic_error("Could not find candidate edge for the location");
}

// Return the index of an edge compared to the path_edge from a location. Assuming the path_edge is
// ordered by the best.
int get_node_candidate_index(const valhalla::Location& location,
                             const GraphId& edge_id,
                             int* percent_along) {
  int i = 0;
  for (const auto& e : location.path_edges()) {
    if (e.graph_id() == edge_id) {
      *percent_along = e.percent_along();
      return i;
    }
    i++;
  }
  return i;
}

std::vector<PathInfo> buildPath(GraphReader& graphreader,
                                const Options& /*options*/,
                                const valhalla::Location& origin,
                                const valhalla::Location& dest,
                                const baldr::TimeInfo& time_info,
                                const bool invariant,
                                std::vector<GraphId> path_edges,
                                const std::shared_ptr<sif::DynamicCost>& costing_,
                                float source_pct,
                                float target_pct) {
  // Build a vector of path info
  // once we recovered the whole path we should construct list of PathInfo objects
  // set of edges recovered from shortcuts (excluding shortcut's start edges)
  std::unordered_set<GraphId> recovered_inner_edges;

  std::vector<PathInfo> path;
  path.reserve(path_edges.size());

  auto edge_itr = path_edges.begin();
  const auto edge_cb = [&edge_itr, &path_edges]() {
    return (edge_itr == path_edges.end()) ? GraphId{} : (*edge_itr++);
  };

  const auto label_cb = [&path, &recovered_inner_edges](const EdgeLabel& label) {
    path.emplace_back(label.mode(), label.cost(), label.edgeid(), 0, label.path_distance(),
                      label.restriction_idx(), label.transition_cost(),
                      recovered_inner_edges.count(label.edgeid()));
  };

  // recost edges in final path; ignore access restrictions
  try {
    sif::recost_forward(graphreader, *costing_, edge_cb, label_cb, source_pct, target_pct, time_info,
                        invariant, true);
  } catch (const std::exception& e) {
    LOG_ERROR(std::string("Chinese Postman failed to recost final path: ") + e.what());
  }

  return path;
}

DistanceMatrix thor_worker_t::computeCostMatrix(std::vector<baldr::GraphId> graph_ids,
                                                const std::shared_ptr<sif::DynamicCost>& costing,
                                                const float max_matrix_distance) {
  DistanceMatrix distanceMatrix(boost::extents[graph_ids.size()][graph_ids.size()]);
  if (graph_ids.size() == 0) {
    return distanceMatrix;
  }

  // TODO: Need to populate these two variables
  google::protobuf::RepeatedPtrField<valhalla::Location> source_location_list;
  google::protobuf::RepeatedPtrField<valhalla::Location> target_location_list;

  for (const auto& graph_id : graph_ids) {
    GraphId g(graph_id);
    auto l = getPointLL(g, *reader);
    Location loc = Location();

    loc.mutable_ll()->set_lng(l.first);
    loc.mutable_ll()->set_lat(l.second);
    source_location_list.Add()->CopyFrom(loc);
    target_location_list.Add()->CopyFrom(loc);
  }

  try {
    auto locations = PathLocation::fromPBF(source_location_list, true);
    const auto projections = loki::Search(locations, *reader, costing);
    for (size_t i = 0; i < locations.size(); ++i) {
      const auto& correlated = projections.at(locations[i]);
      PathLocation::toPBF(correlated, source_location_list.Mutable(i), *reader);
    }
  } catch (const std::exception&) { throw valhalla_exception_t{171}; }

  try {
    auto locations = PathLocation::fromPBF(target_location_list, true);
    const auto projections = loki::Search(locations, *reader, costing);
    for (size_t i = 0; i < locations.size(); ++i) {
      const auto& correlated = projections.at(locations[i]);
      PathLocation::toPBF(correlated, target_location_list.Mutable(i), *reader);
    }
  } catch (const std::exception&) { throw valhalla_exception_t{171}; }

  CostMatrix costmatrix;
  std::vector<thor::TimeDistance> td =
      costmatrix.SourceToTarget(source_location_list, target_location_list, *reader, mode_costing,
                                mode, max_matrix_distance);

  // Update Distance Matrix
  for (int i = 0; i < graph_ids.size(); i++) {
    for (int j = 0; j < graph_ids.size(); j++) {
      distanceMatrix[i][j] = td[i * source_location_list.size() + j].dist;
    }
  }

  return distanceMatrix;
}

double getEdgeCost(GraphReader& reader, baldr::GraphId edge_id) {
  Cost cost{};
  // fetch the graph objects
  graph_tile_ptr tile;
  const baldr::DirectedEdge* edge = reader.directededge(edge_id, tile);
  // Notes: Use edge length for now
  return edge->length();
}

std::vector<GraphId>
thor_worker_t::computeFullRoute(CPVertex cpvertex_start,
                                CPVertex cpvertex_end,
                                const Options& options,
                                const std::string costing_str,
                                const std::shared_ptr<sif::DynamicCost>& costing) {
  LOG_DEBUG("computeFullRoute");
  std::vector<GraphId> edge_graph_ids;

  // Setup
  google::protobuf::RepeatedPtrField<Location> locations_;
  std::vector<baldr::GraphId> node_ids{GraphId(cpvertex_start.graph_id),
                                       GraphId(cpvertex_end.graph_id)};
  for (const auto& node_id : node_ids) {
    auto* loc = locations_.Add();
    auto tile = reader->GetGraphTile(node_id);
    const auto* node = tile->node(node_id);
    auto ll = node->latlng(tile->header()->base_ll());
    auto edge_id = tile->id();
    edge_id.set_id(node->edge_index());
    for (const auto& edge : tile->GetDirectedEdges(node_id)) {
      auto* path_edge = loc->add_path_edges();
      path_edge->set_distance(0);
      path_edge->set_begin_node(true);
      path_edge->set_end_node(false);
      loc->mutable_ll()->set_lng(ll.first);
      loc->mutable_ll()->set_lat(ll.second);
      path_edge->set_graph_id(edge_id);
      path_edge->mutable_ll()->set_lng(ll.first);
      path_edge->mutable_ll()->set_lat(ll.second);
      ++edge_id;
    }
  }

  // This try-catch block below can be replaced with something cheaper
  try {
    auto locations = PathLocation::fromPBF(locations_, true);
    const auto projections = loki::Search(locations, *reader, costing);
    for (size_t i = 0; i < locations.size(); ++i) {
      const auto& correlated = projections.at(locations[i]);
      PathLocation::toPBF(correlated, locations_.Mutable(i), *reader);
    }
  } catch (const std::exception&) { throw valhalla_exception_t{171}; }

  auto x = locations_.begin();
  auto y = std::next(x);
  thor::PathAlgorithm* path_algorithm = get_path_algorithm(costing_str, *x, *y, options);
  auto paths = path_algorithm->GetBestPath(*x, *y, *reader, mode_costing, mode, options);
  path_algorithm->Clear();
  // Only take the first path (there is only one path)
  if (paths.size() > 0) {
    auto path = paths[0];
    for (auto p : path) {
      edge_graph_ids.push_back(p.edgeid);
    }
  } else {
    auto start_loc_string = std::to_string(x->ll().lng()) + ", " + std::to_string(x->ll().lat());
    auto end_loc_string = std::to_string(y->ll().lng()) + ", " + std::to_string(y->ll().lat());
    LOG_WARN("No route from " + start_loc_string + " to " + end_loc_string);
  }

  return edge_graph_ids;
}

std::vector<GraphId> thor_worker_t::buildEdgeIds(std::vector<int> reversedEulerPath,
                                                 ChinesePostmanGraph& G,
                                                 const Options& options,
                                                 const std::string costing_str,
                                                 const std::shared_ptr<sif::DynamicCost>& costing) {
  std::vector<GraphId> eulerPathEdgeGraphIDs;
  for (auto it = reversedEulerPath.rbegin(); it != reversedEulerPath.rend(); ++it) {
    if (it + 1 == reversedEulerPath.rend()) {
      continue;
    }
    auto cpEdge = G.getCPEdge(*it, *(it + 1));
    if (cpEdge) {
      eulerPathEdgeGraphIDs.push_back(cpEdge->graph_id);
    } else {
      // Find the edges for path through outside polygon
      auto a = G.getCPVertex(*it);
      auto fullRoute = computeFullRoute(*G.getCPVertex(*it), *G.getCPVertex(*(it + 1)), options,
                                        costing_str, costing);

      for (auto edge_graph_id : fullRoute) {
        eulerPathEdgeGraphIDs.push_back(edge_graph_id);
      }
    }
  }
  return eulerPathEdgeGraphIDs;
}

int getCPVertexIndex(baldr::GraphId graph_id, std::vector<baldr::GraphId> cp_vertices) {
  for (int i = 0; i < cp_vertices.size(); i++) {
    if (graph_id == cp_vertices[i]) {
      return i;
    }
  }

  return -1;
}

void thor_worker_t::chinese_postman(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request, "thor_worker_t::isochrones");

  baldr::DateTime::tz_sys_info_cache_t tz_cache_;

  auto correlated = request.options().locations();
  auto it = correlated.begin();
  auto origin = &it;
  valhalla::Location originLocation = **origin;

  it++;
  auto destination = &it;
  valhalla::Location destinationLocation = **destination;

  midgard::PointLL originPoint = to_ll(originLocation);
  midgard::PointLL destinationPoint = to_ll(destinationLocation);

  ChinesePostmanGraph G;

  parse_locations(request);
  parse_filter_attributes(request);
  auto costing_str = parse_costing(request);
  auto& options = *request.mutable_options();
  const auto& costing_ = mode_costing[static_cast<uint32_t>(mode)];

  auto* co = options.mutable_costing_options(options.costing());
  std::list<std::string> avoid_edge_ids;

  for (auto& avoid_edge : co->exclude_edges()) {
    avoid_edge_ids.push_back(std::to_string(GraphId(avoid_edge.id())));
  }

  int currentOriginNodeIndex = originLocation.path_edges().size();
  CPVertex originVertex;
  int candidateOriginNodeIndex;
  int originPercentAlong;

  int currentDestinationNodeIndex = destinationLocation.path_edges().size();
  CPVertex destinationVertex;
  int candidateDestinationNodeIndex;
  int destinationPercentAlong;

  // Add chinese edges to internal set
  for (auto& edge : co->chinese_edges()) {
    // Exclude the edge if the edge is in avoid_edges
    bool excluded = (std::find(avoid_edge_ids.begin(), avoid_edge_ids.end(),
                               std::to_string(GraphId(edge.id()))) != avoid_edge_ids.end());
    if (excluded)
      continue;

    GraphId start_node = reader->edge_startnode(GraphId(edge.id()));
    CPVertex start_vertex = CPVertex(start_node);
    GraphId end_node = reader->edge_endnode(GraphId(edge.id()));
    CPVertex end_vertex = CPVertex(end_node);

    // Find the vertex for the origin location
    candidateOriginNodeIndex =
        get_node_candidate_index(originLocation, GraphId(edge.id()), &originPercentAlong);
    if (candidateOriginNodeIndex < currentOriginNodeIndex) {
      if (originPercentAlong < 0.5) {
        originVertex = start_vertex;
      } else {
        originVertex = end_vertex;
      }

      currentOriginNodeIndex = candidateOriginNodeIndex;
    }
    G.addVertex(start_vertex);

    // Find the vertex for the destination
    candidateDestinationNodeIndex =
        get_node_candidate_index(destinationLocation, GraphId(edge.id()), &destinationPercentAlong);
    if (candidateDestinationNodeIndex < currentDestinationNodeIndex) {
      // Check this part of the code
      if (destinationPercentAlong < 0.5) {
        destinationVertex = start_vertex;
      } else {
        destinationVertex = end_vertex;
      }
      currentDestinationNodeIndex = candidateDestinationNodeIndex;
    }
    G.addVertex(end_vertex);

    // The cost of an edge is not relevant for the graph since we need to visit all the edges.
    // For a simplicity, I put Cost(1, 1) for it.
    // The cost is only considered when matching the unbalanced nodes.
    // TODO: probably remove this since we use edge length as the heuristic
    Cost cost(1, 1);
    CPEdge cpEdge(cost, baldr::GraphId(edge.id()));
    G.addEdge(start_vertex, end_vertex, cpEdge);
  }
  // If the node index is more than the path_edge size, that means that there is no suitable
  // node for the origin or destination location.
  if (currentOriginNodeIndex >= originLocation.path_edges().size()) {
    throw valhalla_exception_t(451);
  }
  if (currentDestinationNodeIndex >= destinationLocation.path_edges().size()) {
    throw valhalla_exception_t(451);
  }

  LOG_DEBUG("Number of edges in Chinese Postman Graph: " + std::to_string(G.numEdges()));

  bool isSameOriginDestination = destinationVertex.graph_id == originVertex.graph_id;

  // Solving the Chinese Postman
  std::vector<GraphId> edgeGraphIds;

  // Check if the graph is ideal or not
  if (G.isIdealGraph(originVertex, destinationVertex)) {
    auto reverseEulerPath = G.computeIdealEulerCycle(originVertex);
    edgeGraphIds = buildEdgeIds(reverseEulerPath, G, options, costing_str, costing_);
  } else {

    // Do matching here
    auto sorted_unbalanced_nodes = G.getUnbalancedVertices();
    // A flag to check whether we already evaluate the origin and destination nodes
    bool originNodeChecked = false;
    bool destinationNodeChecked = false;

    // Populate the list of node which has too many incoming and too few incoming
    std::vector<baldr::GraphId> overNodes;
    std::vector<baldr::GraphId> underNodes;
    for (auto const& v : G.getUnbalancedVerticesMap()) {
      // Calculate the number of needed edges to make it balanced
      int extraEdges = 0;
      if (!isSameOriginDestination && v.first == originVertex.vertex_id) {
        extraEdges = abs(v.second + 1);
        originNodeChecked = true;
      } else if (!isSameOriginDestination && v.first == destinationVertex.vertex_id) {
        extraEdges = abs(v.second - 1);
        destinationNodeChecked = true;
      } else {
        extraEdges = abs(v.second);
      }
      for (int i = 0; i < extraEdges; i++) {
        if (v.second > 0) {
          overNodes.push_back(GraphId(v.first));
        } else {
          underNodes.push_back(GraphId(v.first));
        }
      }
    }
    // Handle if the origin or destination nodes are not managed yet
    if (!isSameOriginDestination) {
      if (!originNodeChecked) {
        sorted_unbalanced_nodes.push_back(originVertex.graph_id);
        overNodes.push_back(originVertex.graph_id);
      }
      if (!destinationNodeChecked) {
        sorted_unbalanced_nodes.push_back(destinationVertex.graph_id);
        underNodes.push_back(destinationVertex.graph_id);
      }
    }

    auto distance_matrix = computeCostMatrix(sorted_unbalanced_nodes, costing_,
                                             max_matrix_distance.find(costing_str)->second);
    // Populating matrix for pairing
    std::vector<std::vector<double>> pairingMatrix;
    for (int i = 0; i < overNodes.size(); i++) {
      pairingMatrix.push_back(std::vector<double>{});
      for (int j = 0; j < underNodes.size(); j++) {
        // int overNodeIndex = G.getVertexIndex(overNodes[i]);
        int overNodeIndex = getCPVertexIndex(overNodes[i], sorted_unbalanced_nodes);
        int underNodeIndex = getCPVertexIndex(underNodes[i], sorted_unbalanced_nodes);

        double distance = distance_matrix[overNodeIndex][underNodeIndex];
        pairingMatrix[i].push_back(distance);
      }
    }
    // Calling hungarian algorithm
    LOG_DEBUG("Run Hungarian Algorithm");
    LOG_DEBUG("Number of nodes: " + std::to_string(pairingMatrix.size()));
    HungarianAlgorithm hungarian_algorithm;
    vector<int> assignment;
    double cost = hungarian_algorithm.Solve(pairingMatrix, assignment);
    std::vector<std::pair<int, int>> extraPairs;
    for (unsigned int x = 0; x < pairingMatrix.size(); x++) {
      // Get node's index for that pair
      int overNodeIndex = G.getVertexIndex(overNodes[x]);
      int underNodeIndex = G.getVertexIndex(underNodes[assignment[x]]);
      // Concat with main vector
      extraPairs.push_back(make_pair(overNodeIndex, underNodeIndex));
    }
    auto reverseEulerPath = G.computeIdealEulerCycle(originVertex, extraPairs);
    edgeGraphIds = buildEdgeIds(reverseEulerPath, G, options, costing_str, costing_);
  }
  // Start build path here
  LOG_DEBUG("Building full path");
  bool invariant = options.has_date_time_type() && options.date_time_type() == Options::invariant;
  auto time_info = TimeInfo::make(originLocation, *reader, &tz_cache_);
  std::vector<PathInfo> path =
      buildPath(*reader, options, originLocation, destinationLocation, time_info, invariant,
                edgeGraphIds, costing_, originPercentAlong, destinationPercentAlong);

  std::list<valhalla::Location> throughs; // Empty
  std::vector<std::string> algorithms{"Chinese Postman"};
  TripRoute* route = nullptr;
  valhalla::Trip& trip = *request.mutable_trip();
  // Form output information based on path edges
  if (trip.routes_size() == 0 || options.alternates() > 0) {
    route = trip.mutable_routes()->Add();
    route->mutable_legs()->Reserve(options.locations_size());
  }
  auto& leg = *route->mutable_legs()->Add();
  std::unordered_map<size_t, std::pair<EdgeTrimmingInfo, EdgeTrimmingInfo>> vias; // Empty
  TripLegBuilder::Build(options, controller, *reader, mode_costing, path.begin(), path.end(),
                        originLocation, destinationLocation, throughs, leg, algorithms, interrupt,
                        &vias);
}

} // namespace thor
} // namespace valhalla
