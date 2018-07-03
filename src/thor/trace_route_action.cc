#include "thor/worker.h"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "exception.h"
#include "meili/map_matcher.h"

#include "thor/attributes_controller.h"
#include "thor/map_matcher.h"
#include "thor/match_result.h"
#include "thor/route_matcher.h"
#include "thor/trippathbuilder.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::odin;
using namespace valhalla::thor;

namespace {

struct MapMatch {
  // Coordinate of the match point
  midgard::PointLL lnglat;
  // Which edge this match point stays
  baldr::GraphId edgeid;
  // Percentage distance along the edge
  float distance_along;
  int edge_index = -1;
};

// <Confidence score, raw score, match results, trip path> tuple indexes
constexpr size_t kConfidenceScoreIndex = 0;
constexpr size_t kRawScoreIndex = 1;
constexpr size_t kMatchResultsIndex = 2;
constexpr size_t kTripPathIndex = 3;

} // namespace

namespace valhalla {
namespace thor {

/*
 * The trace_route action takes a GPS trace and turns it into a route result.
 */
odin::TripPath thor_worker_t::trace_route(valhalla_request_t& request) {

  // Parse request
  parse_locations(request);
  parse_costing(request);
  parse_measurements(request);

  // Initialize the controller
  odin::TripPath trip_path;
  AttributesController controller;

  switch (request.options.shape_match()) {
    // If the exact points from a prior route that was run against the Valhalla road network,
    // then we can traverse the exact shape to form a path by using edge-walking algorithm
    case odin::ShapeMatch::edge_walk:
      try {
        trip_path = route_match(request, controller);
        if (trip_path.node().size() == 0) {
          throw std::exception{};
        }
      } catch (...) {
        throw valhalla_exception_t{
            443, odin::ShapeMatch_Name(request.options.shape_match()) +
                     " algorithm failed to find exact route match.  Try using "
                     "shape_match:'walk_or_snap' to fallback to map-matching algorithm"};
      }
      break;
    // If non-exact shape points are used, then we need to correct this shape by sending them
    // through the map-matching algorithm to snap the points to the correct shape
    case odin::ShapeMatch::map_snap:
      try {
        auto map_match_results = map_match(request, controller);
        if (!map_match_results.empty()) {
          trip_path = std::get<kTripPathIndex>(map_match_results.at(0));
        }
      } catch (...) { throw valhalla_exception_t{442}; }
      break;
    // If we think that we have the exact shape but there ends up being no Valhalla route match,
    // then we want to fallback to try and use meili map matching to match to local route
    // network. No shortcuts are used and detailed information at every intersection becomes
    // available.
    case odin::ShapeMatch::walk_or_snap:
      trip_path = route_match(request, controller);
      if (trip_path.node().size() == 0) {
        LOG_WARN(odin::ShapeMatch_Name(request.options.shape_match()) +
                 " algorithm failed to find exact route match; Falling back to map_match...");
        try {
          auto map_match_results = map_match(request, controller);
          if (!map_match_results.empty()) {
            trip_path = std::get<kTripPathIndex>(map_match_results.at(0));
          }
        } catch (...) { throw valhalla_exception_t{442}; }
      }
      break;
  }

  if (!request.options.do_not_track()) {
    log_admin(trip_path);
  }

  return trip_path;
}

/*
 * Returns trip path using an “edge-walking” algorithm.
 * This is for use when the input shape is exact shape from a prior Valhalla route.
 * This will walk the input shape and compare to Valhalla edge’s end node positions to
 * form the list of edges. It will return no nodes if path not found.
 *
 */
odin::TripPath thor_worker_t::route_match(valhalla_request_t& request,
                                          const AttributesController& controller) {
  odin::TripPath trip_path;
  std::vector<PathInfo> path_infos;
  if (RouteMatcher::FormPath(mode_costing, mode, reader, trace, request.options.locations(),
                             path_infos)) {
    // Form the trip path based on mode costing, origin, destination, and path edges
    trip_path = thor::TripPathBuilder::Build(controller, reader, mode_costing, path_infos,
                                             *request.options.mutable_locations()->begin(),
                                             *request.options.mutable_locations()->rbegin(),
                                             std::list<odin::Location>{}, interrupt);
  }

  return trip_path;
}

// Form the path from the map-matching results. This path gets sent to TripPathBuilder.
// PathInfo is primarily a list of edge Ids but it also include elapsed time to the end
// of each edge. We will need to use the existing costing method to form the elapsed time
// the path. We will start with just using edge costs and will add transition costs.
std::vector<std::tuple<float, float, std::vector<thor::MatchResult>, odin::TripPath>>
thor_worker_t::map_match(valhalla_request_t& request,
                         const AttributesController& controller,
                         uint32_t best_paths) {
  std::vector<std::tuple<float, float, std::vector<thor::MatchResult>, odin::TripPath>>
      map_match_results;

  // Call Meili for map matching to get a collection of Location Edges
  matcher->set_interrupt(interrupt);
  // Create the vector of matched path results
  std::vector<meili::MatchResults> offline_results;
  if (trace.size() > 0) {
    offline_results = matcher->OfflineMatch(trace, best_paths);
  }

  // Process each score/match result
  for (const auto& result : offline_results) {
    const auto& match_results = result.results;
    const auto& edge_segments = result.segments;
    std::vector<thor::MatchResult> enhanced_match_results;
    odin::TripPath trip_path;
    std::unordered_map<size_t, std::pair<RouteDiscontinuity, RouteDiscontinuity>>
        route_discontinuities;

    // Form the path edges based on the matched points and populate disconnected edges
    std::vector<std::pair<GraphId, GraphId>> disconnected_edges;
    std::vector<PathInfo> path_edges =
        MapMatcher::FormPath(matcher.get(), match_results, edge_segments, mode_costing, mode,
                             disconnected_edges);

    // Throw exception if not trace attributes action and disconnected path
    if (request.options.action() == odin::DirectionsOptions::trace_route &&
        disconnected_edges.size()) {
      throw valhalla_exception_t{442};
    };

    // OSRM map matching format has both the match points and the route, fill out the match points
    // here Note that we only support trace_route as OSRM format so best_paths == 1
    if (request.options.action() == odin::DirectionsOptions::trace_route &&
        request.options.format() == odin::DirectionsOptions::osrm) {
      const GraphTile* tile = nullptr;
      for (int i = 0; i < match_results.size(); ++i) {
        // Get the match
        const auto& match = match_results[i];
        if (!match.edgeid.Is_Valid()) {
          continue;
        }

        // Make one path edge from it
        reader.GetGraphTile(match.edgeid, tile);
        auto* pe = request.options.mutable_shape(i)->mutable_path_edges()->Add();
        pe->mutable_ll()->set_lat(match.lnglat.lat());
        pe->mutable_ll()->set_lng(match.lnglat.lng());
        for (const auto& n : reader.edgeinfo(match.edgeid).GetNames()) {
          pe->mutable_names()->Add()->assign(n);
        }

        // signal how many edge candidates there were at this stateid by adding empty path edges
        if (!match.HasState()) {
          continue;
        }
        for (int j = 0;
             j < matcher->state_container().state(match.stateid).candidate().edges.size() - 1; ++j) {
          request.options.mutable_shape(i)->mutable_path_edges()->Add();
        }
      }
    }

    // Associate match points to edges, if enabled
    if (request.options.action() == odin::DirectionsOptions::trace_attributes &&
        controller.category_attribute_enabled(kMatchedCategory)) {
      // Populate for matched points so we have 1:1 with trace points
      for (const auto& match_result : match_results) {
        // Matched type is set in constructor
        enhanced_match_results.emplace_back(match_result);
      }

      // Iterate over results to set edge_index, if found
      int edge_index = 0;
      int last_matched_edge_index = edge_index;
      auto edge = path_edges.cbegin();
      auto last_matched_edge = edge;
      for (auto& enhanced_match_result : enhanced_match_results) {
        // Reset edge and edge_index to last matched for every matched result
        edge = last_matched_edge;
        edge_index = last_matched_edge_index;

        // Check for result for valid edge id
        if (enhanced_match_result.edgeid.Is_Valid()) {
          // Walk edges to find matching id
          while (edge != path_edges.cend()) {
            // Find matching edge id in path
            if (enhanced_match_result.edgeid == edge->edgeid) {
              // Set match result with matched edge index and break out of the loop
              enhanced_match_result.edge_index = edge_index;

              // Set the last matched edge and edge_index so we skip matched transition edges
              last_matched_edge = edge;
              last_matched_edge_index = edge_index;
              break;
            } else {
              // Increment to next edge and edge_index
              ++edge;
              ++edge_index;
            }
          }
        }
      }

      // Mark the disconnected route boundaries
      auto curr_match_result = enhanced_match_results.begin();
      auto prev_match_result = curr_match_result;
      for (auto& disconnected_edge_pair : disconnected_edges) {

        // Find previous(first) edge within the match results
        while (curr_match_result != enhanced_match_results.end()) {
          if (curr_match_result->edgeid == disconnected_edge_pair.first.value) {
            // Found previous edge therefore stop looking
            break;
          }
          // Increment previous and current match results to continue looking
          prev_match_result = curr_match_result;
          ++curr_match_result;
        }

        // Find the last match result of the previous edge
        while (curr_match_result != enhanced_match_results.end()) {
          if (curr_match_result->edgeid != disconnected_edge_pair.first.value) {
            // Set previous match result as disconnected path and break
            prev_match_result->begin_route_discontinuity = true;

            // The begin route discontinuity is the edge end info
            // therefore the second item in the pair
            if (route_discontinuities.count(prev_match_result->edge_index) > 0) {
              // Update edge_end_info values
              auto& edge_end_info = route_discontinuities.at(prev_match_result->edge_index).second;
              edge_end_info.exists = true;
              edge_end_info.vertex = prev_match_result->lnglat;
              edge_end_info.distance_along = prev_match_result->distance_along;
            } else {
              // Add new item
              // Begin distance along defaulted to 0
              route_discontinuities.insert(
                  {prev_match_result->edge_index,
                   {{false, {}, 0.f},
                    {true, prev_match_result->lnglat, prev_match_result->distance_along}}});
            }
            break;
          }
          // Increment previous and current match results to continue looking
          prev_match_result = curr_match_result;
          ++curr_match_result;
        }

        // Find the current(second) edge within the match results
        while (curr_match_result != enhanced_match_results.end()) {
          if (curr_match_result->edgeid == disconnected_edge_pair.second.value) {
            // Set current match result as disconnected and break
            curr_match_result->end_route_discontinuity = true;

            // The end route discontinuity is the edge begin info
            // therefore the first item in the pair
            if (route_discontinuities.count(curr_match_result->edge_index) > 0) {
              // Update edge_begin_info values
              auto& edge_begin_info = route_discontinuities.at(curr_match_result->edge_index).first;
              edge_begin_info.exists = true;
              edge_begin_info.vertex = curr_match_result->lnglat;
              edge_begin_info.distance_along = curr_match_result->distance_along;
            } else {
              // Add new item
              // End distance along defaulted to 1
              route_discontinuities.insert(
                  {curr_match_result->edge_index,
                   {{true, curr_match_result->lnglat, curr_match_result->distance_along},
                    {false, {}, 1.f}}});
            }
            break;
          }
          // Increment previous and current match results to continue looking
          prev_match_result = curr_match_result;
          ++curr_match_result;
        }
      }

#ifdef LOGGING_LEVEL_TRACE
      ////////////////////////////////////////////////////////////////////////////
      // This trace block is used to visualize the trace and matched points
      // Print geojson header
      printf("\n{\"type\":\"FeatureCollection\",\"features\":[\n");

      // Print trace points
      int index = 0;
      for (const auto& trace_point : trace) {
        printf("{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":[%.6f,%.6f]}"
               ",\"properties\":{\"marker-color\":\"#abd9e9\",\"marker-size\":\"small\",\"trace_"
               "point_index\":%d}},\n",
               trace_point.lnglat().first, trace_point.lnglat().second, index++);
      }

      // Print matched points
      index = 0;
      std::string marker_color;
      std::string marker_size;
      std::string matched_point_type;
      for (const auto& match_result : enhanced_match_results) {
        if (match_result.begin_route_discontinuity || match_result.end_route_discontinuity) {
          marker_color = "#d7191c"; // red
          marker_size = "large";
          if (match_result.type == thor::MatchResult::Type::kMatched)
            matched_point_type = "matched";
          else
            matched_point_type = "interpolated";
        } else if (match_result.type == thor::MatchResult::Type::kMatched) {
          marker_color = "#2c7bb6"; // dark blue
          marker_size = "medium";
          matched_point_type = "matched";
        } else if (match_result.type == thor::MatchResult::Type::kInterpolated) {
          marker_color = "#ffffbf"; // yellow
          marker_size = "small";
          matched_point_type = "interpolated";
        } else {
          marker_color = "#fdae61"; // orange
          marker_size = "small";
          matched_point_type = "unmatched";
        }
        printf("{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":[%.6f,%.6f]}"
               ",\"properties\":{\"marker-color\":\"%s\",\"marker-size\":\"%s\",\"matched_point_"
               "index\":%d,\"matched_point_type\":\"%s\",\"edge_index\":%u,\"distance_along_edge\":"
               "%.3f,\"distance_from_trace_point\":%.3f}}%s\n",
               match_result.lnglat.lng(), match_result.lnglat.lat(), marker_color.c_str(),
               marker_size.c_str(), index++, matched_point_type.c_str(), match_result.edge_index,
               match_result.distance_along, match_result.distance_from,
               ((index != enhanced_match_results.size() - 1) ? "," : ""));
      }

      // Print geojson footer
      printf("]}\n");
////////////////////////////////////////////////////////////////////////////
#endif
    }

    // Set origin and destination from map matching results
    auto first_result_with_state =
        std::find_if(match_results.begin(), match_results.end(),
                     [](const meili::MatchResult& result) {
                       return result.HasState() && result.edgeid.Is_Valid();
                     });

    auto last_result_with_state = std::find_if(match_results.rbegin(), match_results.rend(),
                                               [](const meili::MatchResult& result) {
                                                 return result.HasState() && result.edgeid.Is_Valid();
                                               });

    if ((first_result_with_state != match_results.end()) &&
        (last_result_with_state != match_results.rend())) {
      odin::Location origin;
      PathLocation::toPBF(matcher->state_container()
                              .state(first_result_with_state->stateid)
                              .candidate(),
                          &origin, reader);
      odin::Location destination;
      PathLocation::toPBF(matcher->state_container()
                              .state(last_result_with_state->stateid)
                              .candidate(),
                          &destination, reader);

      bool found_origin = false;
      for (const auto& e : origin.path_edges()) {
        if (e.graph_id() == path_edges.front().edgeid) {
          found_origin = true;
          break;
        }
      }

      if (!found_origin) {
        // 1. origin must be at a node, so we can reuse any one of
        // origin's edges

        // 2. path_edges.front().edgeid must be the downstream edge that
        // connects one of origin.edges (twins) at its start node
        auto* pe = origin.mutable_path_edges()->Add();
        pe->CopyFrom(origin.path_edges(0));
        pe->set_graph_id(path_edges.front().edgeid);
        pe->set_percent_along(0.f);
      }

      bool found_destination = false;
      for (const auto& e : destination.path_edges()) {
        if (e.graph_id() == path_edges.back().edgeid) {
          found_destination = true;
          break;
        }
      }

      if (!found_destination) {
        // 1. destination must be at a node, so we can reuse any one of
        // destination's edges

        // 2. path_edges.back().edgeid must be the upstream edge that
        // connects one of destination.edges (twins) at its end node
        auto* pe = destination.mutable_path_edges()->Add();
        pe->CopyFrom(destination.path_edges(0));
        pe->set_graph_id(path_edges.back().edgeid);
        pe->set_percent_along(1.f);
      }

      // assert origin.edges contains path_edges.front() &&
      // destination.edges contains path_edges.back()

      // Form the trip path based on mode costing, origin, destination, and path edges
      trip_path =
          thor::TripPathBuilder::Build(controller, matcher->graphreader(), mode_costing, path_edges,
                                       origin, destination, std::list<odin::Location>{}, interrupt,
                                       &route_discontinuities);
    } else {
      throw valhalla_exception_t{442};
    }
    // Keep the result
    map_match_results.emplace_back(map_match_results.empty()
                                       ? 1.0f
                                       : std::get<kRawScoreIndex>(map_match_results.front()) /
                                             result.score,
                                   result.score, enhanced_match_results, trip_path);
  }

  return map_match_results;
}

} // namespace thor
} // namespace valhalla
