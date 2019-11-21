#include "thor/worker.h"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "meili/map_matcher.h"

#include "thor/attributes_controller.h"
#include "thor/map_matcher.h"
#include "thor/match_result.h"
#include "thor/route_matcher.h"
#include "thor/triplegbuilder.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;
using std::vector;

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
constexpr size_t kTripLegIndex = 3;

void add_path_edge(valhalla::Location* l, const meili::MatchResult& m) {
  auto* edge = l->mutable_path_edges()->Add();
  edge->set_graph_id(m.edgeid);
  edge->set_percent_along(m.distance_along);
  edge->mutable_ll()->set_lng(m.lnglat.first);
  edge->mutable_ll()->set_lat(m.lnglat.second);
  edge->set_distance(m.distance_from);
  // NOTE: we dont need side of street here because the match is continuous we dont know if they were
  // starting a route from the side of the road or whatever so calling that out is not a good idea
  // edge->set_side_of_street();
  // NOTE: we dont care about reachablity because the match will have worked or not worked!
  // edge->set_minimum_reachability();
}

} // namespace

namespace valhalla {
namespace thor {

/*
 * The trace_route action takes a GPS trace and turns it into a route result.
 */
void thor_worker_t::trace_route(Api& request) {
  // Parse request
  parse_locations(request);
  parse_costing(request);
  parse_measurements(request);
  parse_filter_attributes(request);

  const auto& options = *request.mutable_options();

  switch (options.shape_match()) {
    // If the exact points from a prior route that was run against the Valhalla road network,
    // then we can traverse the exact shape to form a path by using edge-walking algorithm
    case ShapeMatch::edge_walk:
      try {
        route_match(request);
      } catch (...) {
        throw valhalla_exception_t{
            443, ShapeMatch_Enum_Name(options.shape_match()) +
                     " algorithm failed to find exact route match.  Try using "
                     "shape_match:'walk_or_snap' to fallback to map-matching algorithm"};
      }
      break;
    // If non-exact shape points are used, then we need to correct this shape by sending them
    // through the map-matching algorithm to snap the points to the correct shape
    case ShapeMatch::map_snap:
      try {
        map_match(request);
      } catch (...) { throw valhalla_exception_t{442}; }
      break;
    // If we think that we have the exact shape but there ends up being no Valhalla route match,
    // then we want to fallback to try and use meili map matching to match to local route
    // network. No shortcuts are used and detailed information at every intersection becomes
    // available.
    case ShapeMatch::walk_or_snap:
      try {
        route_match(request);
      } catch (...) {
        LOG_WARN(ShapeMatch_Enum_Name(options.shape_match()) +
                 " algorithm failed to find exact route match; Falling back to map_match...");
        try {
          map_match(request);
        } catch (...) { throw valhalla_exception_t{442}; }
      }
      break;
  }

  // log admin areas
  if (!options.do_not_track()) {
    for (const auto& route : request.trip().routes()) {
      for (const auto& leg : route.legs()) {
        log_admin(leg);
      }
    }
  }
}

/*
 * Returns trip path using an “edge-walking” algorithm.
 * This is for use when the input shape is exact shape from a prior Valhalla route.
 * This will walk the input shape and compare to Valhalla edge’s end node positions to
 * form the list of edges. It will return no nodes if path not found.
 *
 */
void thor_worker_t::route_match(Api& request) {
  // TODO - make sure the trace has timestamps..
  auto& options = *request.mutable_options();
  if (RouteMatcher::FormPath(mode_costing, mode, *reader, trace, options, m_path_infos)) {
    // TODO: we dont support multileg here as it ignores location types but...
    // if this were a time dependent match you need to propogate the date time
    // information to each legs origin location because triplegbuilder relies on it.
    // form path set the first one but on the subsequent legs we will need to set them
    // by doing time offsetting like is done in route_action.cc thor_worker_t::depart_at

    // For now we ignore multileg complications and just make sure the searched locations
    // get the same data information the shape informations had
    if (options.shape(0).has_date_time())
      options.mutable_locations(0)->set_date_time(options.shape(0).date_time());

    // Form the trip path based on mode costing, origin, destination, and path edges
    auto& leg = *request.mutable_trip()->mutable_routes()->Add()->mutable_legs()->Add();
    thor::TripLegBuilder::Build(controller, *reader, mode_costing, m_path_infos.begin(),
                                m_path_infos.end(), *options.mutable_locations()->begin(),
                                *options.mutable_locations()->rbegin(),
                                std::list<valhalla::Location>{}, leg, interrupt);
  } else {
    throw std::exception{};
  }
}

// Form the path from the map-matching results. This path gets sent to TripLegBuilder.
// PathInfo is primarily a list of edge Ids but it also include elapsed time to the end
// of each edge. We will need to use the existing costing method to form the elapsed time
// the path. We will start with just using edge costs and will add transition costs.
std::vector<std::tuple<float, float, std::vector<thor::MatchResult>>>
thor_worker_t::map_match(Api& request) {
  auto& options = *request.mutable_options();
  // Call Meili for map matching to get a collection of Location Edges
  matcher->set_interrupt(interrupt);
  // Create the vector of matched path results
  if (trace.size() == 0) {
    return {};
  }

  // we don't allow multi path for trace route at the moment, discontinuities force multi route
  int best_path =
      request.options().action() == Options::trace_attributes ? request.options().best_paths() : 1;
  m_offline_results = matcher->OfflineMatch(trace, best_path);

  // Process each score/match result
  for (const auto& result : m_offline_results) {
    const auto& match_results = result.results;
    const auto& edge_segments = result.segments;
    m_temp_enhanced_match_results.clear();
    m_temp_route_discontinuities.clear();
    m_temp_disconnected_edges.clear();

    // Form the path edges based on the matched points and populate disconnected edges
    m_temp_path_edges = MapMatcher::FormPath(matcher.get(), match_results, edge_segments,
                                             mode_costing, mode, m_temp_disconnected_edges, options);

    // If we want a route but there actually isnt a path, we cant give you one
    if (m_temp_path_edges.empty() && options.action() == Options::trace_route) {
      throw std::exception{};
    }

    // OSRM map matching format has both the match points and the route, fill out the match points
    // here Note that we only support trace_route as OSRM format so best_paths == 1
    if (options.action() == Options::trace_route && options.format() == Options::osrm) {
      const GraphTile* tile = nullptr;
      for (int i = 0; i < match_results.size(); ++i) {
        // Get the match
        const auto& match = match_results[i];
        if (!match.edgeid.Is_Valid()) {
          continue;
        }

        // Make one path edge from it
        reader->GetGraphTile(match.edgeid, tile);
        auto* pe = options.mutable_shape(i)->mutable_path_edges()->Add();
        pe->mutable_ll()->set_lat(match.lnglat.lat());
        pe->mutable_ll()->set_lng(match.lnglat.lng());
        for (const auto& n : reader->edgeinfo(match.edgeid).GetNames()) {
          pe->mutable_names()->Add()->assign(n);
        }

        // signal how many edge candidates there were at this stateid by adding empty path edges
        if (!match.HasState()) {
          continue;
        }
        for (int j = 0;
             j < matcher->state_container().state(match.stateid).candidate().edges.size() - 1; ++j) {
          options.mutable_shape(i)->mutable_path_edges()->Add();
        }
      }
    }

    // Associate match points to edges, if enabled
    if (options.action() == Options::trace_attributes &&
        controller.category_attribute_enabled(kMatchedCategory)) {
      // Populate for matched points so we have 1:1 with trace points
      for (const auto& match_result : match_results) {
        // Matched type is set in constructor
        m_temp_enhanced_match_results.emplace_back(match_result);
      }

      // Iterate over results to set edge_index, if found
      int edge_index = 0;
      int last_matched_edge_index = edge_index;
      auto edge = m_temp_path_edges.cbegin();
      auto last_matched_edge = edge;
      for (auto& enhanced_match_result : m_temp_enhanced_match_results) {
        // Reset edge and edge_index to last matched for every matched result
        edge = last_matched_edge;
        edge_index = last_matched_edge_index;

        // Check for result for valid edge id
        if (enhanced_match_result.edgeid.Is_Valid()) {
          // Walk edges to find matching id
          while (edge != m_temp_path_edges.cend()) {
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
      auto curr_match_result = m_temp_enhanced_match_results.begin();
      auto prev_match_result = curr_match_result;
      for (auto& disconnected_edge_pair : m_temp_disconnected_edges) {

        // Find previous(first) edge within the match results
        while (curr_match_result != m_temp_enhanced_match_results.end()) {
          if (curr_match_result->edgeid == disconnected_edge_pair.first.value) {
            // Found previous edge therefore stop looking
            break;
          }
          // Increment previous and current match results to continue looking
          prev_match_result = curr_match_result;
          ++curr_match_result;
        }

        // Find the last match result of the previous edge
        while (curr_match_result != m_temp_enhanced_match_results.end()) {
          if (curr_match_result->edgeid != disconnected_edge_pair.first.value) {
            // Set previous match result as disconnected path and break
            prev_match_result->begin_route_discontinuity = true;

            // The begin route discontinuity is the edge end info
            // therefore the second item in the pair
            if (m_temp_route_discontinuities.count(prev_match_result->edge_index) > 0) {
              // Update edge_end_info values
              auto& edge_end_info =
                  m_temp_route_discontinuities.at(prev_match_result->edge_index).second;
              edge_end_info.exists = true;
              edge_end_info.vertex = prev_match_result->lnglat;
              edge_end_info.distance_along = prev_match_result->distance_along;
            } else {
              // Add new item
              // Begin distance along defaulted to 0
              m_temp_route_discontinuities.insert(
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
        while (curr_match_result != m_temp_enhanced_match_results.end()) {
          if (curr_match_result->edgeid == disconnected_edge_pair.second.value) {
            // Set current match result as disconnected and break
            curr_match_result->end_route_discontinuity = true;

            // The end route discontinuity is the edge begin info
            // therefore the first item in the pair
            if (m_temp_route_discontinuities.count(curr_match_result->edge_index) > 0) {
              // Update edge_begin_info values
              auto& edge_begin_info =
                  m_temp_route_discontinuities.at(curr_match_result->edge_index).first;
              edge_begin_info.exists = true;
              edge_begin_info.vertex = curr_match_result->lnglat;
              edge_begin_info.distance_along = curr_match_result->distance_along;
            } else {
              // Add new item
              // End distance along defaulted to 1
              m_temp_route_discontinuities.insert(
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
      for (const auto& match_result : m_temp_enhanced_match_results) {
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
               ((index != m_temp_enhanced_match_results.size() - 1) ? "," : ""));
      }

      // Print geojson footer
      printf("]}\n");
////////////////////////////////////////////////////////////////////////////
#endif
    }

    // trace_attributes always returns a single trip path and may have discontinuities
    if (options.action() == Options::trace_attributes) {
      // we make a new route to hold the result of each iteraton of top k
      auto& route = *request.mutable_trip()->mutable_routes()->Add();
      path_map_match(match_results, m_temp_path_edges, *route.mutable_legs()->Add(),
                     m_temp_route_discontinuities);
    } // trace_route can return multiple trip paths
    else {

      // here we break the path edges into groups of contiguous ones so that
      // where they are discontinuous we can make them into separate routes
      m_temp_disjoint_edge_groups = {{{m_temp_path_edges.front()}, {}}};
      auto discontinuity_edges_iter = m_temp_disconnected_edges.begin();
      for (int i = 1, n = static_cast<int>(m_temp_path_edges.size()); i < n; ++i) {
        const auto& front_edge_info = m_temp_path_edges[i - 1];
        const auto& back_edge_info = m_temp_path_edges[i];
        // make a new group when discontinuity occurs
        if (discontinuity_edges_iter != m_temp_disconnected_edges.end() &&
            discontinuity_edges_iter->first == front_edge_info.edgeid &&
            discontinuity_edges_iter->second == back_edge_info.edgeid) {
          m_temp_disjoint_edge_groups.emplace_back();
          ++discontinuity_edges_iter;
        }
        m_temp_disjoint_edge_groups.back().first.emplace_back(back_edge_info);
      }

      // here we mark edges in the path that get "upgraded" to break points
      // because they occur just before a discontinuity
      auto discontinuity_point_iter = match_results.cbegin();
      for (auto& disjoint_edge_group : m_temp_disjoint_edge_groups) {
        const auto& last_edge_in_group = disjoint_edge_group.first.back();
        // in each edge group, we only care about the last edge where the discontinuity happens
        while (discontinuity_point_iter->edgeid != last_edge_in_group.edgeid) {
          ++discontinuity_point_iter;
        }
        // for the last edge, the last match result on this edge is where discontinuity happens
        while (discontinuity_point_iter->edgeid == last_edge_in_group.edgeid) {
          ++discontinuity_point_iter;
        }

        disjoint_edge_group.second = discontinuity_point_iter - 1;
      }

      // The following logic put break points (matches results) on edge candidates to form legs
      // logic assumes the both match results and edge candidates are topologically sorted in correct
      // order
      auto leg_origin_iter = match_results.cbegin();
      for (const auto& disjoint_edge_group : m_temp_disjoint_edge_groups) {
        // for each disjoint edge group, we make a new route for it.
        // We use multi-route to handle discontinuity
        const auto& edges = disjoint_edge_group.first;
        const auto& disjoint_point = disjoint_edge_group.second;
        auto* route = request.mutable_trip()->mutable_routes()->Add();
        // first we find where this leg is going to begin
        while (leg_origin_iter != match_results.end() &&
               leg_origin_iter->edgeid != edges.front().edgeid) {
          ++leg_origin_iter;
        }
        if (leg_origin_iter == match_results.cend()) {
          break;
        }

        // loop through each edge in the group, build legs accordingly
        int last_edge_index = 0;
        for (int i = 0, n = static_cast<int>(edges.size()); i < n; ++i) {
          const auto& path_edge = edges[i];
          // then we find where each leg is going to end by finding the
          // first valid destination matched points after origin matched points
          for (auto leg_destination_iter = leg_origin_iter + 1;
               leg_destination_iter != match_results.cend(); ++leg_destination_iter) {
            // we skip input location points that are:
            // 1. not on the match results edge
            // 2. neither break points nor the disjoint (upgraded break) points
            if (path_edge.edgeid != leg_destination_iter->edgeid ||
                (options.shape(leg_destination_iter - match_results.begin()).type() !=
                     valhalla::Location::kBreak &&
                 options.shape(leg_destination_iter - match_results.begin()).type() !=
                     valhalla::Location::kBreakThrough &&
                 leg_destination_iter != disjoint_point)) {
              continue;
            }

            // for trip leg builder to work it needs to have origin and destination locations
            // so we fake them here before calling it
            Location* origin_location =
                options.mutable_shape(leg_origin_iter - match_results.cbegin());
            Location* destination_location =
                options.mutable_shape(leg_destination_iter - match_results.cbegin());
            add_path_edge(origin_location, *leg_origin_iter);
            add_path_edge(destination_location, *leg_destination_iter);

            // add a new leg to the current route
            TripLegBuilder::Build(controller, matcher->graphreader(), mode_costing,
                                  edges.begin() + last_edge_index, edges.begin() + i + 1,
                                  *origin_location, *destination_location,
                                  std::list<valhalla::Location>{}, *route->mutable_legs()->Add(),
                                  interrupt, &m_temp_route_discontinuities);

            // beginning of next leg will be the end of this leg
            leg_origin_iter = leg_destination_iter;
            // store the starting index of the path_edges
            last_edge_index = i;
          }
        }
      }
    }
    // TODO: move this info to the trip leg
    // Keep the result
    m_map_match_results.emplace_back(m_map_match_results.empty()
                                         ? 1.0f
                                         : std::get<kRawScoreIndex>(m_map_match_results.front()) /
                                               result.score,
                                     result.score, std::move(m_temp_enhanced_match_results));
  }

  return m_map_match_results;
}

// Function that returns a trip path for a trace_attributes map match.
// TODO merge this logic with the trace_route version above. Much of the
// logic is similar but handles discontinuities at the origin/destination.
// We need to add a test for that scenario and then we can merge the logic.
void thor_worker_t::path_map_match(
    const std::vector<meili::MatchResult>& match_results,
    const std::vector<PathInfo>& path_edges,
    TripLeg& trip_path,
    std::unordered_map<size_t, std::pair<RouteDiscontinuity, RouteDiscontinuity>>&
        route_discontinuities) {

  // Set origin and destination from map matching results
  auto first_result_with_state =
      std::find_if(match_results.begin(), match_results.end(), [](const meili::MatchResult& result) {
        return result.HasState() && result.edgeid.Is_Valid();
      });

  auto last_result_with_state = std::find_if(match_results.rbegin(), match_results.rend(),
                                             [](const meili::MatchResult& result) {
                                               return result.HasState() && result.edgeid.Is_Valid();
                                             });

  if ((first_result_with_state != match_results.end()) &&
      (last_result_with_state != match_results.rend())) {
    valhalla::Location origin;
    PathLocation::toPBF(matcher->state_container()
                            .state(first_result_with_state->stateid)
                            .candidate(),
                        &origin, *reader);
    valhalla::Location destination;
    PathLocation::toPBF(matcher->state_container().state(last_result_with_state->stateid).candidate(),
                        &destination, *reader);

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
    thor::TripLegBuilder::Build(controller, matcher->graphreader(), mode_costing, path_edges.begin(),
                                path_edges.end(), origin, destination,
                                std::list<valhalla::Location>{}, trip_path, interrupt,
                                &route_discontinuities);
  } else {
    throw valhalla_exception_t{442};
  }
}

} // namespace thor
} // namespace valhalla
