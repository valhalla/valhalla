#include <algorithm>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/multi_array.hpp>
#include <boost/multi_array/subarray.hpp>

#include "Hungarian.h"

#include "loki/search.h"
#include "midgard/util.h"
#include "sif/costconstants.h"
#include "sif/recost.h"
#include "thor/chinese_postman_graph.h"
#include "thor/costmatrix.h"
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

// Return the index of an edge compared to the path_edge from a location. Assuming the path_edge is
// ordered by the best.
int get_node_candidate_index(const valhalla::Location& location,
                             const GraphId& edge_id,
                             int* percent_along) {
  int i = 0;
  for (const auto& e : location.correlation().edges()) {
    if (e.graph_id() == edge_id) {
      *percent_along = e.percent_along();
      return i;
    }
    i++;
  }
  return i;
}

std::vector<PathInfo> buildPath(GraphReader& graphreader,
                                const Options&,
                                const valhalla::Location& /*origin*/,
                                const valhalla::Location& /*destination*/,
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

// Computing the cost matrix between graph ids
DistanceMatrix thor_worker_t::computeCostMatrix(std::vector<baldr::GraphId> graph_ids,
                                                const std::shared_ptr<sif::DynamicCost>& costing,
                                                const float max_matrix_distance) {

  // Create a DistanceMatrix to with the size of graph_ids size
  DistanceMatrix distanceMatrix(boost::extents[graph_ids.size()][graph_ids.size()]);

  // If the graph_ids is empty, return empty DistanceMatrix
  if (graph_ids.size() == 0) {
    return distanceMatrix;
  }

  // get all the path locations for all the nodes
  google::protobuf::RepeatedPtrField<valhalla::Location> sources, targets;
  graph_tile_ptr tile, opp_tile;
  for (const auto& node_id : graph_ids) {
    // skip missing tiles
    if (!reader->GetGraphTile(node_id, tile))
      continue;

    // set up a basic path location for a node in the graph
    const auto* node = tile->node(node_id);
    if (!costing->Allowed(node))
      continue;
    auto ll = node->latlng(tile->header()->base_ll());
    PathLocation source(ll), target(ll);

    // get all the edges leaving the source and entering the target
    source.edges.reserve(node->edge_count());
    target.edges.reserve(node->edge_count());
    for (const auto& edge : tile->GetDirectedEdges(node)) {
      if (!costing->Allowed(&edge, tile, kDisallowShortcut))
        continue;
      auto edge_id = tile->id(&edge);
      source.edges.emplace_back(edge_id, 0, ll, 0);
      auto opp_id = reader->GetOpposingEdgeId(edge_id, opp_tile);
      if (opp_id)
        target.edges.emplace_back(opp_id, 1, ll, 0);
    }

    // convert it back to pbf for the matrix api
    PathLocation::toPBF(source, sources.Add(), *reader);
    PathLocation::toPBF(target, targets.Add(), *reader);
  }

  // get the all pairs matrix result
  CostMatrix costmatrix;
  std::vector<thor::TimeDistance> td =
      costmatrix.SourceToTarget(sources, targets, *reader, mode_costing, mode, max_matrix_distance);

  // Update Distance Matrix
  for (int i = 0; i < graph_ids.size(); i++) {
    for (int j = 0; j < graph_ids.size(); j++) {
      distanceMatrix[i][j] = td[i * graph_ids.size() + j].dist;
    }
  }

  return distanceMatrix;
}

// Compute a full route from a cpvertex_start to cpvertex_end
// The route found can go outside the chinese postman polygon, but will not go through the exclude
// polygons
std::vector<GraphId>
thor_worker_t::computeFullRoute(CPVertex cpvertex_start,
                                CPVertex cpvertex_end,
                                const Options& options,
                                const std::string& costing_str,
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
      auto* path_edge = loc->mutable_correlation()->add_edges();
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
                                                 const std::string& costing_str,
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
      auto fullRoute = computeFullRoute(*G.getCPVertex(*it), *G.getCPVertex(*(it + 1)), options,
                                        costing_str, costing);

      for (auto edge_graph_id : fullRoute) {
        eulerPathEdgeGraphIDs.push_back(edge_graph_id);
      }
    }
  }
  return eulerPathEdgeGraphIDs;
}

// Helper function to get the index of a graph id from a vector of graph id
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
  auto _ = measure_scope_time(request);

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
  controller = AttributesController(request.options());
  auto costing_str = parse_costing(request);
  auto& options = *request.mutable_options();
  const auto& costing_ = mode_costing[static_cast<uint32_t>(mode)];

  auto& co = *options.mutable_costings()->find(options.costing_type())->second.mutable_options();
  std::list<std::string> exclude_edge_ids;

  for (auto& exclude_edge : co.exclude_edges()) {
    exclude_edge_ids.push_back(std::to_string(GraphId(exclude_edge.id())));
  }

  int currentOriginNodeIndex = originLocation.correlation().edges().size();
  CPVertex originVertex;
  int candidateOriginNodeIndex;
  int originPercentAlong;

  int currentDestinationNodeIndex = destinationLocation.correlation().edges().size();
  CPVertex destinationVertex;
  int candidateDestinationNodeIndex;
  int destinationPercentAlong;

  // Add chinese edges to the CP Graph
  for (auto& edge : co.chinese_edges()) {
    // Exclude the edge if the edge is in exclude_edges
    bool excluded = (std::find(exclude_edge_ids.begin(), exclude_edge_ids.end(),
                               std::to_string(GraphId(edge.id()))) != exclude_edge_ids.end());
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
  if (currentOriginNodeIndex >= originLocation.correlation().edges().size()) {
    throw valhalla_exception_t(451);
  }
  if (currentDestinationNodeIndex >= destinationLocation.correlation().edges().size()) {
    throw valhalla_exception_t(451);
  }

  LOG_DEBUG("Number of edges in Chinese Postman Graph: " + std::to_string(G.numEdges()));

  // A flag if the destination and the origin is the same node
  bool isSameOriginDestination = destinationVertex.graph_id == originVertex.graph_id;

  // Solving the Chinese Postman
  std::vector<GraphId> edgeGraphIds;

  // Check if the graph is ideal or not
  if (G.isIdealGraph(originVertex, destinationVertex)) {
    // If the graph is ideal, we can already compute the euler path
    auto reverseEulerPath = G.computeIdealEulerCycle(originVertex);
    // Build the edge graph ids from the computed euler path
    edgeGraphIds = buildEdgeIds(reverseEulerPath, G, options, costing_str, costing_);
  } else {
    // If the graph is not ideal, we need to make it ideal by creating a "imaginary" edge between the
    // unbalanced nodes

    // First, get all the unbalanced nodes
    auto sorted_unbalanced_nodes = G.getUnbalancedVertices();

    // A flag to check whether we already evaluate the origin and destination nodes
    bool originNodeChecked = false;
    bool destinationNodeChecked = false;

    // Populate the list of node which has too many incoming and too few incoming edges
    std::vector<baldr::GraphId> overNodes;  // Incoming > Outcoming edges
    std::vector<baldr::GraphId> underNodes; // Incoming < Outcoming edges

    for (auto const& v : G.getUnbalancedVerticesMap()) {
      // Calculate the number of needed extra edges to make it balance
      int extraEdges = 0;

      if (!isSameOriginDestination && v.first == originVertex.vertex_id) {
        // If the origin != destination and the vertex is the origin vertex, we need to add 1 extra
        // edge
        extraEdges = abs(v.second + 1);
        originNodeChecked = true;
      } else if (!isSameOriginDestination && v.first == destinationVertex.vertex_id) {
        // If the origin != destination and the vertex is the destination vertex, we need to reduce 1
        // extra edge
        extraEdges = abs(v.second - 1);
        destinationNodeChecked = true;
      } else {
        // In other case, just add a number of difference between incoming and outcoming nodes to the
        // extra edges
        extraEdges = abs(v.second);
      }
      // Populate the overNode and undeNodes
      for (int i = 0; i < extraEdges; i++) {
        if (v.second > 0) {
          overNodes.push_back(GraphId(v.first));
        } else {
          underNodes.push_back(GraphId(v.first));
        }
      }
    }
    // Handle if the origin or destination nodes are not managed yet
    // This can happen when the origin or destinaton has the same number of incoming and outcoming
    // edge, but the route has different origin and destination
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

    // Compute the distance/cost between unbalanced nodes.
    auto distance_matrix = computeCostMatrix(sorted_unbalanced_nodes, costing_,
                                             max_matrix_distance.find(costing_str)->second);
    // Populating matrix for pairing
    std::vector<std::vector<double>> pairingMatrix;
    for (int i = 0; i < overNodes.size(); i++) {
      pairingMatrix.push_back(std::vector<double>{});
      for (int j = 0; j < underNodes.size(); j++) {
        int overNodeIndex = getCPVertexIndex(overNodes[i], sorted_unbalanced_nodes);
        int underNodeIndex = getCPVertexIndex(underNodes[i], sorted_unbalanced_nodes);

        double distance = distance_matrix[overNodeIndex][underNodeIndex];
        pairingMatrix[i].push_back(distance);
      }
    }
    // Calling hungarian algorithm to pair the unbalanced nodes
    LOG_DEBUG("Run Hungarian Algorithm");
    LOG_DEBUG("Number of nodes: " + std::to_string(pairingMatrix.size()));
    HungarianAlgorithm hungarian_algorithm;
    vector<int> assignment;
    hungarian_algorithm.Solve(pairingMatrix, assignment);
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
  bool invariant = options.date_time_type() != Options::no_time;
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
                        originLocation, destinationLocation, leg, algorithms, interrupt);
}

} // namespace thor
} // namespace valhalla
