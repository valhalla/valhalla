#include <prime_server/prime_server.hpp>
using namespace prime_server;

#include <memory>

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/geojson.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/errorcode_util.h>
#include <valhalla/meili/map_matcher.h>
#include <valhalla/proto/trippath.pb.h>

#include "thor/service.h"
#include "thor/expandfromnode.h"
#include "thor/mapmatching.h"
#include "thor/trippathbuilder.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::meili;
using namespace valhalla::thor;

namespace valhalla {
namespace thor {

/*
 * The trace_route action takes a GPS trace and turns it into a route result.
 */
worker_t::result_t thor_worker_t::trace_route(const boost::property_tree::ptree &request,
    const std::string &request_str, const bool header_dnt) {
  parse_locations(request);
  parse_shape(request);
  parse_costing(request);

  //get time for start of request
  auto s = std::chrono::system_clock::now();
  worker_t::result_t result { true };
  // Forward the original request
  result.messages.emplace_back(request_str);

  // If the exact points from a prior route that was run agains the Valhalla road network,
  //then we can traverse the exact shape to form a path by using edge-walking algorithm
  auto trip_path = route_match();
  if (trip_path.node().size() == 0) {
    //If no Valhalla route match, then use meili map matching
    //to match to local route network. No shortcuts are used and detailed
    //information at every intersection becomes available.
    trip_path = map_match();
  }
  result.messages.emplace_back(trip_path.SerializeAsString());
  // Get processing time for thor
  auto e = std::chrono::system_clock::now();
  std::chrono::duration<float, std::milli> elapsed_time = e - s;
  // TODO determine what to log
  //log request if greater than X (ms)
  if (!header_dnt
      && (elapsed_time.count() / correlated.size()) > long_request) {
    LOG_WARN(
        "thor::trace_route elapsed time (ms)::"
            + std::to_string(elapsed_time.count()));
    LOG_WARN("thor::trace_route exceeded threshold::" + request_str);
    midgard::logging::Log("valhalla_thor_long_request_trace_route",
                          " [ANALYTICS] ");
  }
  return result;
}

// Form the path from the map-matching results. This path gets sent to TripPathBuilder.
// PathInfo is primarily a list of edge Ids but it also include elapsed time to the end
// of each edge. We will need to use the existing costing method to form the elapsed time
// the path. We will start with just using edge costs and will add transition costs.

/*
 * Returns true if an exact route match using an “edge-walking” algorithm.
 * This is for use when the input shape is exact shape from a prior Valhalla route.
 * This will walk the input shape and compare to Valhalla edge’s end node positions to
 * form the list of edges.
 *
 */

odin::TripPath thor_worker_t::route_match() {
  odin::TripPath trip_path;
  std::vector<PathInfo> path_infos;
  if (route_match(path_infos)) {
    // Empty through location list
    std::vector<baldr::PathLocation> through_loc;

    // Form the trip path based on mode costing, origin, destination, and path edges
    trip_path = thor::TripPathBuilder::Build(reader, mode_costing,
                                                  path_infos,
                                                  correlated.front(),
                                                  correlated.back(),
                                                  through_loc);

  }
  return trip_path;
}

bool thor_worker_t::route_match(std::vector<PathInfo>& path_infos) {
  float elapsed_time = 0.f;

  // Process and validate begin edge
  const PathLocation::PathEdge* begin_path_edge = find_begin_edge();
  if ((begin_path_edge == nullptr) || !(begin_path_edge->id.Is_Valid())) {
    throw std::runtime_error("Invalid begin edge id");
  }
  const GraphTile* begin_edge_tile = reader.GetGraphTile(begin_path_edge->id);
  if (begin_edge_tile == nullptr) {
    throw std::runtime_error("Begin tile is null");
  }

  // Process and validate end edge
  const PathLocation::PathEdge* end_path_edge = find_end_edge();
  if ((end_path_edge == nullptr) || !(end_path_edge->id.Is_Valid())) {
    throw std::runtime_error("Invalid end edge id");
  }
  const GraphTile* end_edge_tile = reader.GetGraphTile(end_path_edge->id);
  if (end_edge_tile == nullptr) {
    throw std::runtime_error("End tile is null");
  }

  // Assign the end edge start node
  const GraphId end_edge_start_node = find_start_node(end_path_edge->id);

  // Process directed edge and info
  const DirectedEdge* de = begin_edge_tile->directededge(begin_path_edge->id);
  const GraphTile* end_node_tile = reader.GetGraphTile(de->endnode());
  if (begin_edge_tile == nullptr) {
    throw std::runtime_error("End node tile is null");
  }
  PointLL de_end_ll = end_node_tile->node(de->endnode())->latlng();

  // If start and end have the same edge then add and return
  if (begin_path_edge->id == end_path_edge->id) {

    // Update the elapsed time edge cost at single edge
    elapsed_time += mode_costing[static_cast<int>(mode)]->EdgeCost(de).secs * (end_path_edge->dist - begin_path_edge->dist);

    // Add single edge
    path_infos.emplace_back(mode, std::round(elapsed_time), begin_path_edge->id, 0);
    return true;
  }

  // Initialize indexes and shape
  size_t index = 0;
  uint32_t shape_length = 0;
  uint32_t de_length = std::round(de->length() * (1 - begin_path_edge->dist)) + 50; // TODO make constant
  EdgeLabel prev_edge_label;
  // Loop over shape to form path from matching edges
  while (index < shape.size()
      && (std::round(shape.at(0).Distance(shape.at(index))) < de_length)) {
    if (shape.at(index).ApproximatelyEqual(de_end_ll)) {

      // Update the elapsed time edge cost at begin edge
      elapsed_time += mode_costing[static_cast<int>(mode)]->EdgeCost(de).secs * (1 - begin_path_edge->dist);

      // Add begin edge
      path_infos.emplace_back(mode, std::round(elapsed_time), begin_path_edge->id, 0);

      // Set previous edge label
      prev_edge_label = {kInvalidLabel, begin_path_edge->id, de, {}, 0, 0, mode, 0};

      // Continue walking shape to find the end edge...
      if (ExpandFromNode::FormPath(mode_costing, mode, reader, shape, index,
                                        end_node_tile, de->endnode(),
                                        end_edge_start_node, prev_edge_label,
                                        elapsed_time, path_infos, false)) {
        // Update the elapsed time based on transition cost
        elapsed_time += mode_costing[static_cast<int>(mode)]->TransitionCost(
            de, end_edge_tile->node(end_edge_start_node), prev_edge_label).secs;

        // Update the elapsed time based on edge cost
        elapsed_time += mode_costing[static_cast<int>(mode)]->EdgeCost(de).secs * end_path_edge->dist;

        // Add end edge
        path_infos.emplace_back(mode, std::round(elapsed_time), end_path_edge->id, 0);

        return true;
      } else {
        // Did not find end edge - so get out
        return false;
      }
    }
    index++;
  }
  return false;
}

const PathLocation::PathEdge* thor_worker_t::find_begin_edge() const {
  // Iterate through start edges
  for (const auto& edge : correlated.front().edges) {
    // If origin is at a node - skip any inbound edge
    if (edge.end_node()) {
      continue;
    }
    return &edge;  //TODO special case
  }
  return nullptr;
}

const PathLocation::PathEdge* thor_worker_t::find_end_edge() const{
  // Iterate through end edges
  for (const auto& edge : correlated.back().edges) {
    // If destination is at a node - skip any outbound edge
    if (edge.begin_node()) {
      continue;
    }

    return &edge;  //TODO special case
  }
  return nullptr;
}

const GraphId thor_worker_t::find_start_node(const GraphId& edge_id) {
  const GraphTile* tile = reader.GetGraphTile(edge_id);
  if (tile == nullptr) {
    throw std::runtime_error("Tile is null");
  }
  const DirectedEdge* de = tile->directededge(edge_id);

  GraphId opp_edge_id = tile->GetOpposingEdgeId(de);
  const DirectedEdge* opp_de = tile->directededge(opp_edge_id);

  return opp_de->endnode();
}

odin::TripPath thor_worker_t::map_match() {
  odin::TripPath trip_path;
  // Call Meili for map matching to get a collection of pathLocation Edges
  // Create a matcher
  std::shared_ptr<MapMatcher> matcher;
  try {
    matcher.reset(matcher_factory.Create(config));
  } catch (const std::invalid_argument& ex) {
    //return jsonify_error({400, 499}, request_info, std::string(ex.what()));
    throw std::runtime_error(std::string(ex.what()));
  }

  std::vector<Measurement> sequence;
  for (const auto& coord : shape) {
    sequence.emplace_back(coord, gps_accuracy, search_radius);
  }

  // Create the vector of matched path results
  std::vector<meili::MatchResult> results;
  for (size_t i = 0; i < sequence.size(); i++) {
    results = (matcher->OfflineMatch(sequence));
  }

  // Form the path edges based on the matched points
  std::vector<PathInfo> path_edges = MapMatching::FormPath(matcher.get(), results,
                                                          mode_costing, mode);

  // Set origin and destination from map matching results
  auto first_result_with_state = std::find_if(
      results.begin(), results.end(),
      [](const meili::MatchResult& result) {return result.HasState();});
  auto last_result_with_state = std::find_if(
      results.rbegin(), results.rend(),
      [](const meili::MatchResult& result) {return result.HasState();});
  if ((first_result_with_state != results.end())
      && (last_result_with_state != results.rend())) {
    baldr::PathLocation origin = matcher->mapmatching().state(
        first_result_with_state->stateid()).candidate();
    baldr::PathLocation destination = matcher->mapmatching().state(
        last_result_with_state->stateid()).candidate();

    // Empty through location list
    std::vector<baldr::PathLocation> through_loc;

    // Form the trip path based on mode costing, origin, destination, and path edges
    trip_path = thor::TripPathBuilder::Build(matcher->graphreader(),
                                                  mode_costing, path_edges,
                                                  origin, destination,
                                                  through_loc);
  } else {
    throw baldr::valhalla_exception_t { 400, 442 };
  }
  return trip_path;
}
}
}
