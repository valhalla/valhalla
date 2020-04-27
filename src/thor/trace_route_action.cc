#include "thor/worker.h"

#include <algorithm>
#include <limits>
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
  l->mutable_path_edges()->Clear();
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
  std::vector<PathInfo> path;
  if (RouteMatcher::FormPath(mode_costing, mode, *reader, trace, options, path)) {
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
    thor::TripLegBuilder::Build(controller, *reader, mode_costing, path.begin(), path.end(),
                                *options.mutable_locations()->begin(),
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
  int topk =
      request.options().action() == Options::trace_attributes ? request.options().best_paths() : 1;
  auto topk_match_results = matcher->OfflineMatch(trace, topk);

  // Process each score/match result
  std::vector<std::tuple<float, float, std::vector<thor::MatchResult>>> map_match_results;
  for (const auto& result : topk_match_results) {
    const auto& match_results = result.results;
    const auto& edge_segments = result.segments;

    // Form the path edges based on the matched points and populate disconnected edges
    std::vector<std::pair<GraphId, GraphId>> disconnected_edges;
    auto path_edges = MapMatcher::FormPath(matcher.get(), match_results, edge_segments, mode_costing,
                                           mode, disconnected_edges, options);

    // If we want a route but there actually isnt a path, we cant give you one
    if (path_edges.empty()) {
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
    std::vector<thor::MatchResult> enhanced_match_results;
    std::unordered_map<size_t, std::pair<RouteDiscontinuity, RouteDiscontinuity>>
        route_discontinuities;
    if (options.action() == Options::trace_attributes &&
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

    // trace_attributes always returns a single trip path and may have discontinuities
    if (options.action() == Options::trace_attributes) {
      // we make a new route to hold the result of each iteration of top k
      auto& route = *request.mutable_trip()->mutable_routes()->Add();
      path_map_match(match_results, path_edges, *route.mutable_legs()->Add(), route_discontinuities);
    } // trace_route can return multiple trip paths
    else {
      // here we break the path edges and match results into contiguous sets so that
      // where they are discontinuous we can make them into separate routes
      using edge_group_t =
          std::tuple<std::vector<PathInfo>::const_iterator, std::vector<PathInfo>::const_iterator,
                     std::vector<meili::MatchResult>::const_iterator,
                     std::vector<meili::MatchResult>::const_iterator>;

      std::vector<edge_group_t> edge_groups{
          edge_group_t{path_edges.cbegin(), path_edges.cbegin(), {}, {}}};
      auto discontinuity_edges_iter = disconnected_edges.begin();
      for (auto path_edge_itr = std::next(path_edges.cbegin()); path_edge_itr != path_edges.cend();
           ++path_edge_itr) {
        auto prev_path_edge_itr = std::prev(path_edge_itr);
        // make a new group when discontinuity occurs
        if (discontinuity_edges_iter != disconnected_edges.end() &&
            discontinuity_edges_iter->first == prev_path_edge_itr->edgeid &&
            discontinuity_edges_iter->second == path_edge_itr->edgeid) {
          // start a new one
          edge_groups.emplace_back(edge_group_t{path_edge_itr, path_edge_itr, {}, {}});
          ++discontinuity_edges_iter;
        }
        // remember where the last one ended
        std::get<1>(edge_groups.back()) = path_edge_itr;
      }

      // here we mark the original shape points that just after and just before the
      // discontinuities so that in the path they get "upgraded" to break points
      auto match_result_itr = match_results.cbegin();
      for (auto& edge_group : edge_groups) {
        auto first_edge = std::get<0>(edge_group);
        auto last_edge = std::get<1>(edge_group);
        // we know the discontinuity happens on this edge so we need to
        // find the first match result on this edge
        match_result_itr = std::find_if(match_result_itr, match_results.cend(),
                                        [first_edge](const MatchResult& result) {
                                          return result.edgeid == first_edge->edgeid;
                                        });
        std::get<2>(edge_group) = match_result_itr;

        // we know the discontinuity happens on this edge so we need to find the last match result
        // on this edge before discontinuity occurs. There might be scenarios that there is a loop on
        // the last edge. We have to navigate the iterator through all the in between path edges.
        // (NOTE that, in between edges may have same edge id with the last edge) right before where
        // discontinuity occurs
        auto prev_match_result = match_result_itr++;
        if (match_result_itr >= match_results.cend()) {
          // Is there a way to handle this better than just failing?
          throw valhalla_exception_t{442};
        }

        // TODO: optimize this linear search away in the form path method
        bool loop_on_last_edge = std::find_if(first_edge, last_edge, [last_edge](const auto& edge) {
                                   return edge.edgeid == last_edge->edgeid;
                                 }) != last_edge;
        bool found_last_match = false;
        while (!found_last_match) {
          // if we could locate an edge with the same edge id of last edge we start our search from
          // that edge. Otherwise, we exit the search, save prev_match_result as the last edge and
          // proceed to a new edge group.
          auto iter = std::find_if(match_result_itr, match_results.cend(),
                                   [last_edge](const MatchResult& result) {
                                     return result.edgeid == last_edge->edgeid;
                                   });
          // we find an edge has same edgeid with last edge.
          if (iter != match_results.cend()) {
            match_result_itr = iter;
          }
          // Search exhausted, prev_match_result is last match result on this edge group
          else {
            break;
          }

          // we are on the edge with the same edge id of the last edge, we have to detect whether
          // this edge is the last edge where discontibuity occurs
          while (match_result_itr != match_results.end() &&
                 match_result_itr->edgeid == last_edge->edgeid) {
            prev_match_result = match_result_itr;
            ++match_result_itr;
            // this indicates discontinuity on the same edge (if there is no loop) see below:
            //
            //     curr distance_along    prev distance_along
            //            |                       |
            //            ▼                       ▼
            //  X---------------------------------------------> 100%
            //
            // we found the last_match result either when distance_along suggests discontinuity
            // on the current edge or we exhaust the search
            if (match_result_itr == match_results.cend() ||
                (prev_match_result->edgeid == last_edge->edgeid &&
                 match_result_itr->distance_along < prev_match_result->distance_along &&
                 !loop_on_last_edge)) {
              found_last_match = true;
              break;
            }
          }
        }
        std::get<3>(edge_group) = prev_match_result;
      }

      // The following logic put break points (matches results) on edge candidates to form legs
      // logic assumes the both match results and edge candidates are topologically sorted in correct
      // order. Only the first location will be populated with corresponding input date_time
      std::string date_time = options.shape(0).has_date_time() ? options.shape(0).date_time() : "";
      auto origin_match_result = match_results.cbegin();
      for (const auto& edge_group : edge_groups) {
        // for each disjoint edge group, we make a new route for it.
        // We use multi-route to handle discontinuity
        uint64_t route_index = request.trip().routes_size();
        auto* route = request.mutable_trip()->mutable_routes()->Add();
        // first we set origin_match_result to leg is going to begin
        origin_match_result = std::get<2>(edge_group);
        if (origin_match_result == match_results.cend()) {
          break;
        }

        // loop through each edge in the group, build legs accordingly
        int last_edge_index = 0;
        int way_point_index = 0;
        for (auto path_edge_itr = std::get<0>(edge_group);
             path_edge_itr < std::next(std::get<1>(edge_group)); ++path_edge_itr) {
          // then we find where each leg is going to end by finding the
          // first valid destination matched points after origin matched points
          for (auto destination_match_result = std::next(origin_match_result);
               destination_match_result != std::next(std::get<3>(edge_group));
               ++destination_match_result) {
            // skip input location points that are not valid
            if (!destination_match_result->edgeid.Is_Valid()) {
              continue;
            }

            // we want to skip edges that are not matching the valid matched locations
            while (path_edge_itr != std::next(std::get<1>(edge_group)) &&
                   path_edge_itr->edgeid != destination_match_result->edgeid) {
              ++path_edge_itr;
            }

            if (path_edge_itr == std::next(std::get<1>(edge_group))) {
              break;
            }

            // If both origin location and destination location are on the same edge, when
            // origin's distance along is greater than destination's distance along, it indicates
            // that there should be other edges in between them (loop occurs). We should put the
            // path edge iter onto the correct edge (jumping over all the edges in the loop).
            // see below:
            //                      origin distance_along
            //                                 |
            //      X---------------------------------------------►X
            //      ▲                 |                            |
            //      |  destination distance along                  |
            //      |                                              ▼
            //      X◄---------------------------------------------X
            //                        edge loop
            if (origin_match_result->edgeid == destination_match_result->edgeid &&
                origin_match_result->distance_along > destination_match_result->distance_along) {
              path_edge_itr =
                  std::find_if(std::next(path_edge_itr), std::next(std::get<1>(edge_group)),
                               [&destination_match_result](const PathInfo& edge) {
                                 return edge.edgeid == destination_match_result->edgeid;
                               });
              if (path_edge_itr == std::next(std::get<1>(edge_group))) {
                break;
              }
            }

            // we only build legs on 3 types of locations:
            // break, breakthrough and disjoint points (if there is disconnect edges)
            // for locations that matched but are break types nor disjoint points
            // we set its waypoint_index to limits::max to notify the serializer thus distinguish
            // them from the first waypoint of the route whose waypoint_index is 0.
            auto break_type = options.shape(destination_match_result - match_results.begin()).type();
            if (break_type != valhalla::Location::kBreak &&
                break_type != valhalla::Location::kBreakThrough &&
                destination_match_result != std::get<3>(edge_group)) {
              Location* via_location =
                  options.mutable_shape(destination_match_result - match_results.cbegin());
              via_location->set_route_index(route_index);
              via_location->set_shape_index(std::numeric_limits<uint32_t>::max());
              continue;
            }

            // initialize the origin and destination location for route
            Location* origin_location =
                options.mutable_shape(origin_match_result - match_results.cbegin());
            Location* destination_location =
                options.mutable_shape(destination_match_result - match_results.cbegin());

            // populate date time for the first location, either with the start time of the route
            // or with the offset datetime of the previous leg in case of multiple legs.
            if (!date_time.empty()) {
              origin_location->set_date_time(date_time);
            }

            // when handling multi routes, orsm serializer need to know both the
            // matching_index(route_index) and the waypoint_index(shape_index).
            origin_location->set_route_index(route_index);
            origin_location->set_shape_index(way_point_index);
            destination_location->set_route_index(route_index);
            destination_location->set_shape_index(++way_point_index);

            // we fake up something that looks like the output of loki
            add_path_edge(&*origin_location, *origin_match_result);
            add_path_edge(&*destination_location, *destination_match_result);

            // mark the beginning and end of the edges on the path for this leg (inclusive)
            auto leg_begin = std::next(std::get<0>(edge_group), last_edge_index);
            auto leg_end = path_edge_itr;

            // "handling partial edges will be the death of us" -- Confucius 504 BC
            // trip leg builder expects the leg to start with 0 elapsed time, however map matches
            // dont care about legs, their path is for the whole match. so we need to trim off any
            // prior legs elapsed time. also because map matching returns full edge path infos we
            // will have to deal with partial edges wherever discontinuities or legs occur except..
            // on the first and last path info or on any discontinuities (because edge segments will
            // have source and target that are not 0 or 1 respectively), FormPath takes care of those
            // for time tracking. to take care of trimming off elapsed time at the front we keep track
            // of the accumulated amount of previous legs' elapsed times. however this is not enough
            // the last edge of every match or the edge where a discontinuity starts will also already
            // be trimmed so we need to take care not to trim it more

            // if its the first or last path info in the edge group then its been already trimmed
            bool begin_trimmed = leg_begin == std::get<0>(edge_group);
            bool end_trimmed = leg_end == std::get<1>(edge_group);
            bool trivial_group = std::get<0>(edge_group) == std::get<1>(edge_group);

            // we need to scale the elapsed time of the current edge to undo what FormPath did
            double begin_pct = begin_trimmed ? std::get<2>(edge_group)->distance_along : 0;
            double end_pct = end_trimmed ? std::get<3>(edge_group)->distance_along : 1;
            double dist_from_begin = (trivial_group ? end_pct : 1) - begin_pct;
            double dist_to_end = end_pct - (trivial_group ? begin_pct : 0);
            double begin_edge_scale = dist_from_begin > 0 ? 1 / dist_from_begin : 0;
            double end_edge_scale = dist_to_end > 0 ? 1 / dist_to_end : 0;

            // we get the time up to the last edge before this begin edge if any. we also remove
            // the turn cost at the begging of this edge if there is any
            double trim_begin = leg_begin == path_edges.cbegin()
                                    ? 0.0
                                    : std::prev(leg_begin)->elapsed_time + leg_begin->turn_cost;
            // then we scale the elapsed time on the edge based on the distance along the edge but
            // we need to scale that distance down in the case FormPath knew it was a partial edge
            trim_begin += (leg_begin->elapsed_time - trim_begin) *
                          (origin_match_result->distance_along - begin_pct) * begin_edge_scale;

            // we get the time up to the last edge before this end edge if any. we also remove
            // the turn cost at the begging of this edge if there is any
            double trim_end = leg_end == path_edges.cbegin()
                                  ? 0.0
                                  : std::prev(leg_end)->elapsed_time + leg_end->turn_cost;

            // then we scale the elapsed time on the edge based on the distance along the edge but
            // we need to scale that distance down in the case FormPath knew it was a partial edge
            trim_end = (leg_end->elapsed_time - trim_end) *
                       (end_pct - destination_match_result->distance_along) * end_edge_scale;

            // add a new leg to the current route
            TripLegBuilder::Build(controller, matcher->graphreader(), mode_costing, leg_begin,
                                  leg_end + 1, *origin_location, *destination_location,
                                  std::list<valhalla::Location>{}, *route->mutable_legs()->Add(),
                                  interrupt, &route_discontinuities, trim_begin, trim_end);

            // we remember the datetime at the end of this leg, to use as the datetime at the start
            // of the next leg.
            if (!date_time.empty()) {
              date_time = offset_date(*reader, date_time, leg_begin->edgeid,
                                      route->legs().rbegin()->node().rbegin()->elapsed_time(),
                                      leg_end->edgeid);
            }

            // beginning of next leg will be the end of this leg
            origin_match_result = destination_match_result;
            // store the starting index of the path_edges
            last_edge_index = std::distance(std::get<0>(edge_group), path_edge_itr);
          }
        }
      }
    }
    // TODO: move this info to the trip leg
    // Keep the result
    map_match_results.emplace_back(map_match_results.empty()
                                       ? 1.0f
                                       : std::get<kRawScoreIndex>(map_match_results.front()) /
                                             result.score,
                                   result.score, std::move(enhanced_match_results));
  }

  return map_match_results;
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
