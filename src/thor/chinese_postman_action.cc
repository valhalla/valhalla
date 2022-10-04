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

namespace {

// Return the index of an edge compared to the path_edge from a location. Assuming the path_edge is
// ordered by the best.
int get_node_candidate_index(const valhalla::Location& location,
                             const GraphId& edge_id,
                             double& percent_along) {
  // see if this edge is an edge candidate for this location
  for (int i = 0; i < location.correlation().edges_size(); ++i) {
    const auto& e = location.correlation().edges(i);
    if (e.graph_id() == edge_id) {
      // TODO: shouldnt we just round this to 0 always, basically if it lands on the edge at all lets
      //  include it in the postman route right?
      percent_along = e.percent_along();
      return i;
    }
  }
  // this signals its not found, ie its not an origin or destination edge candidate
  return location.correlation().edges_size();
}
} // namespace

namespace valhalla {
namespace thor {

using sources_targets_pair = std::pair<google::protobuf::RepeatedPtrField<valhalla::Location>,
                                       google::protobuf::RepeatedPtrField<valhalla::Location>>;
sources_targets_pair sources_targets_from_nodes(const std::vector<GraphId> nodes,
                                                const sif::DynamicCost& costing,
                                                GraphReader& reader) {

  // get all the path locations for all the nodes
  sources_targets_pair results;
  graph_tile_ptr seed_tile, opp_tile;
  for (const auto& seed_node_id : nodes) {
    // skip missing tiles
    if (!reader.GetGraphTile(seed_node_id, seed_tile))
      continue;

    // set up a basic path location for a node in the graph
    const auto* seed_node = seed_tile->node(seed_node_id);
    if (!costing.Allowed(seed_node))
      continue;
    auto ll = seed_node->latlng(seed_tile->header()->base_ll());
    PathLocation source(ll), target(ll);
    source.edges.reserve(seed_node->edge_count());
    target.edges.reserve(seed_node->edge_count());

    // all nodes that represent this node across all hierarchies
    auto all_nodes = seed_tile->GetNodesAcrossLevels(seed_node_id);
    for (const auto& node_id : all_nodes) {
      // resolve the tile and node on their level
      auto tile = seed_tile;
      if (!node_id.Is_Valid() || !reader.GetGraphTile(node_id, tile))
        continue;
      const auto* node = tile->node(node_id);

      // get all the edges leaving the source and entering the target
      for (const auto& edge : tile->GetDirectedEdges(node)) {
        // leaving the node as a source
        auto edge_id = tile->id(&edge);
        if (costing.Allowed(&edge, tile, kDisallowShortcut)) {
          source.edges.emplace_back(edge_id, 0, ll, 0);
        }

        // entering the node as a target
        const DirectedEdge* opp_edge = nullptr;
        auto opp_id = reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile);
        if (opp_id && costing.Allowed(opp_edge, opp_tile, kDisallowShortcut)) {
          target.edges.emplace_back(opp_id, 1, ll, 0);
        }
      }
    }

    // convert it back to pbf for the matrix api
    PathLocation::toPBF(source, results.first.Add(), reader);
    PathLocation::toPBF(target, results.second.Add(), reader);
  }

  return results;
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
DistanceMatrix thor_worker_t::computeCostMatrix(const std::vector<baldr::GraphId>& graph_ids,
                                                const std::shared_ptr<sif::DynamicCost>& costing,
                                                const float max_matrix_distance) {

  // Create a DistanceMatrix to with the size of graph_ids size
  DistanceMatrix distanceMatrix(boost::extents[graph_ids.size()][graph_ids.size()]);

  // If the graph_ids is empty, return empty DistanceMatrix
  if (graph_ids.size() == 0) {
    return distanceMatrix;
  }

  // Create the path locations
  sources_targets_pair::first_type sources;
  sources_targets_pair::second_type targets;
  std::tie(sources, targets) = sources_targets_from_nodes(graph_ids, *costing, *reader);

  // Get the all pairs matrix result
  CostMatrix costmatrix;
  std::vector<thor::TimeDistance> td =
      costmatrix.SourceToTarget(sources, targets, *reader, mode_costing, mode, max_matrix_distance);

  // Update Distance Matrix
  for (int i = 0; i < graph_ids.size(); i++) {
    for (int j = 0; j < graph_ids.size(); j++) {
      distanceMatrix[i][j] = td[i * sources.size() + j].dist;
    }
  }

  return distanceMatrix;
}

// Compute a full route from a cpvertex_start to cpvertex_end
// The route found can go outside the chinese postman polygon, but will not go through the exclude
// polygons
std::vector<GraphId>
thor_worker_t::computeFullRoute(baldr::GraphId cpvertex_start,
                                baldr::GraphId cpvertex_end,
                                const Options& options,
                                const std::string& costing_str,
                                const std::shared_ptr<sif::DynamicCost>& costing) {
  LOG_DEBUG("computeFullRoute");
  std::vector<GraphId> edge_graph_ids;

  // Get the locations setup for the algorithm
  std::vector<baldr::GraphId> node_ids{GraphId(cpvertex_start), GraphId(cpvertex_end)};
  auto all_pairs = sources_targets_from_nodes(node_ids, *costing, *reader);
  auto& source = *all_pairs.first.begin();
  auto& target = *all_pairs.second.rbegin();

  // Run the route
  thor::PathAlgorithm* path_algorithm = get_path_algorithm(costing_str, source, target, options);
  auto paths = path_algorithm->GetBestPath(source, target, *reader, mode_costing, mode, options);
  path_algorithm->Clear(); // might have to call this again
  // Only take the first path (there is only one path)
  if (paths.size() > 0) {
    auto path = paths[0];
    for (auto p : path) {
      edge_graph_ids.push_back(p.edgeid);
    }
  } else {
    auto sll = std::to_string(source.ll().lat()) + "," + std::to_string(source.ll().lng());
    auto tll = std::to_string(target.ll().lat()) + "," + std::to_string(target.ll().lng());
    LOG_WARN("No path for chinese vertex pair: " + sll + " to " + tll);
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

  // basic init
  auto costing_str = parse_costing(request);
  auto& options = request.options();
  const auto& costing_ = mode_costing[static_cast<uint32_t>(mode)];

  // setup for the origin location
  auto origin = options.locations(0);
  int currentOriginNodeIndex = origin.correlation().edges().size();
  GraphId originVertex;
  int candidateOriginNodeIndex;
  double originPercentAlong;

  // setup for the destination location
  auto destination = options.locations(1);
  int currentDestinationNodeIndex = destination.correlation().edges().size();
  GraphId destinationVertex;
  int candidateDestinationNodeIndex;
  double destinationPercentAlong;

  // Add edges to the CP Graph, which were found from the spatial bins in loki
  // avoided_edges
  ChinesePostmanGraph G;
  graph_tile_ptr tile;

  // TODO(nils): originVertex and destinationVertex is literally the only things
  // this loop looks up. that seems super wasteful, iterating over all correlated edges of both
  // origin/destination for every single chinese edge, only to find the origin/destination vertices.
  // aren't all correlated edges automatically part of the chinese edges so we can shortcut this
  // tremendously? the case out origin/destination outside of the polygon should be checked somewhere
  // else also the decision based on percentalong() which node should be , but maybe I just didn't
  // look enough into the rest yet..
  for (const auto& edge : options.chinese_edges()) {
    // we need to figure out which end of the edge we are going to go with
    const auto edge_id = GraphId(edge.id());
    GraphId end_vertex = reader->edge_endnode(edge_id, tile);
    if (!end_vertex.Is_Valid())
      continue;
    GraphId start_vertex = reader->edge_startnode(edge_id, tile);

    // Find the vertex for the origin
    candidateOriginNodeIndex = get_node_candidate_index(origin, edge_id, originPercentAlong);
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
        get_node_candidate_index(destination, edge_id, destinationPercentAlong);
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
    CPEdge cpEdge{cost, edge_id};
    G.addEdge(start_vertex, end_vertex, cpEdge);
  }

  // If the node index is more than the path_edge size, that means that there is no suitable
  // node for the origin or destination location.
  if (currentOriginNodeIndex == origin.correlation().edges().size() ||
      currentDestinationNodeIndex == destination.correlation().edges().size()) {
    throw valhalla_exception_t(451);
  }

  LOG_DEBUG("Number of edges in Chinese Postman Graph: " + std::to_string(G.numEdges()));

  // A flag if the destination and the origin is the same node
  bool isSameOriginDestination = destinationVertex == originVertex;

  // Solving the Chinese Postman
  std::vector<GraphId> edgeGraphIds;

  // Check if the graph is ideal or not
  if (G.isIdealGraph(originVertex, destinationVertex)) {
    // If the graph is ideal, we can already compute the euler path
    auto reverseEulerPath = G.computeIdealEulerCycle(originVertex);
    // Build the edge graph ids from the computed euler path
    edgeGraphIds = buildEdgeIds(reverseEulerPath, G, options, costing_str, costing_);
  } // If the graph is not ideal, we fix it by creating "imaginary" edges between the unbalanced nodes
  else {
    // First, get all the unbalanced nodes
    auto sorted_unbalanced_nodes = G.getUnbalancedVertices();

    // A flag to check whether we already evaluate the origin and destination nodes
    bool originNodeChecked = false;
    bool destinationNodeChecked = false;

    // Populate the list of nodes which have too many incoming and too few outgoing edges
    std::vector<baldr::GraphId> overNodes;  // Incoming > Outgoing edges
    std::vector<baldr::GraphId> underNodes; // Incoming < Outgoing edges
    for (auto const& v : G.getUnbalancedVerticesMap()) {
      // Calculate the number of needed extra edges to make it balance
      int extraEdges = 0;

      if (!isSameOriginDestination && v.first == originVertex) {
        // If the origin != destination and the vertex is the origin vertex, we need to add 1 extra
        // edge
        extraEdges = abs(v.second + 1);
        originNodeChecked = true;
      } else if (!isSameOriginDestination && v.first == destinationVertex) {
        // If the origin != destination and the vertex is the destination vertex, we need to reduce 1
        // extra edge
        extraEdges = abs(v.second - 1);
        destinationNodeChecked = true;
      } else {
        // In other case, just add a number of difference between incoming and outcoming nodes to the
        // extra edges
        extraEdges = abs(v.second);
      }
      // Populate the overNode and underNodes
      for (int i = 0; i < extraEdges; i++) {
        if (v.second > 0) {
          overNodes.push_back(GraphId(v.first));
        } else {
          underNodes.push_back(GraphId(v.first));
        }
      }
    }
    // Handle if the origin or destination nodes are not managed yet
    // This can happen when the origin or destinaton has the same number of incoming and outgoing
    // edges, but the route has a different origin and destination
    if (!isSameOriginDestination) {
      if (!originNodeChecked) {
        sorted_unbalanced_nodes.push_back(originVertex);
        overNodes.push_back(originVertex);
      }
      if (!destinationNodeChecked) {
        sorted_unbalanced_nodes.push_back(destinationVertex);
        underNodes.push_back(destinationVertex);
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
        // TODO: This should be using j, need to update tests
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
  baldr::DateTime::tz_sys_info_cache_t tz_cache_;
  auto time_info = TimeInfo::make(origin, *reader, &tz_cache_);
  std::vector<PathInfo> path =
      buildPath(*reader, options, origin, destination, time_info, invariant, edgeGraphIds, costing_,
                originPercentAlong, destinationPercentAlong);

  // Form output information based on path edges
  std::vector<std::string> algorithms{"Chinese Postman"};
  auto& leg = *request.mutable_trip()->mutable_routes()->Add()->mutable_legs()->Add();
  TripLegBuilder::Build(options, AttributesController(request.options()), *reader, mode_costing,
                        path.begin(), path.end(), origin, destination, leg, algorithms, interrupt);
}

} // namespace thor
} // namespace valhalla
