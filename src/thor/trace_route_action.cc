#include "thor/worker.h"

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "baldr/attributes_controller.h"
#include "meili/map_matcher.h"
#include "meili/match_result.h"
#include "midgard/util.h"
#include "thor/map_matcher.h"
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

void add_path_edge(valhalla::Location* l,
                   const GraphId& edge_id,
                   float percent_along,
                   const midgard::PointLL& ll,
                   float distance) {
  l->mutable_correlation()->mutable_edges()->Clear();
  auto* edge = l->mutable_correlation()->mutable_edges()->Add();
  edge->set_graph_id(edge_id);
  edge->set_percent_along(percent_along);
  edge->mutable_ll()->set_lng(ll.first);
  edge->mutable_ll()->set_lat(ll.second);
  edge->set_distance(distance);
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
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  // Parse request
  auto& options = *request.mutable_options();
  adjust_scores(options);
  parse_costing(request);
  parse_measurements(request);
  controller = AttributesController(options);

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
      // clang-format off
      try {
        map_match(request);
      } catch (const valhalla_exception_t& e) {
        throw e;
      } catch (...) {
        throw valhalla_exception_t{442};
      }
      // clang-format on
      break;
    // If we think that we have the exact shape but there ends up being no Valhalla route match,
    // then we want to fallback to try and use meili map matching to match to local route
    // network. No shortcuts are used and detailed information at every intersection becomes
    // available.
    // clang-format off
    case ShapeMatch::walk_or_snap:
      try {
        route_match(request);
      } catch (...) {
        LOG_WARN(ShapeMatch_Enum_Name(options.shape_match()) +
                 " algorithm failed to find exact route match; Falling back to map_match...");
        try {
          map_match(request);
        } catch (const valhalla_exception_t& e) {
          throw e;
        } catch (...) {
          throw valhalla_exception_t{442};
        }
      }
      // clang-format on
      break;
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
  std::vector<std::vector<PathInfo>> legs;

  // if the shape walking succeeds
  if (RouteMatcher::FormPath(mode_costing, mode, *reader, options, legs)) {
    // the origin is the first location for sure
    auto origin = options.mutable_shape()->begin();
    // build each leg of the route
    for (const auto& pleg : legs) {
      // find the destination for this leg
      auto dest = std::find_if(origin + 1, options.mutable_shape()->end(),
                               [&options](const valhalla::Location& l) {
                                 return l.type() == Location::kBreak ||
                                        l.type() == Location::kBreakThrough ||
                                        &l == &*options.shape().rbegin();
                               });
      assert(dest != options.mutable_shape()->end());
      // Form the trip path based on mode costing, origin, destination, and path edges
      auto& leg = *request.mutable_trip()->mutable_routes()->Add()->mutable_legs()->Add();
      thor::TripLegBuilder::Build(options, controller, *reader, mode_costing, pleg.begin(),
                                  pleg.end(), *origin, *dest, leg, {"edge_walk"}, interrupt);
      // Next leg
      origin = dest;
    }
  } // the shape walking failed
  else {
    throw std::exception{};
  }
}

// Form the path from the map-matching results. This path gets sent to TripLegBuilder.
// PathInfo is primarily a list of edge Ids but it also include elapsed time to the end
// of each edge. We will need to use the existing costing method to form the elapsed time
// the path. We will start with just using edge costs and will add transition costs.
std::vector<std::tuple<float, float, std::vector<meili::MatchResult>>>
thor_worker_t::map_match(Api& request) {
  auto& options = *request.mutable_options();
  // Call Meili for map matching to get a collection of Location Edges
  matcher->set_interrupt(interrupt);
  // Create the vector of matched path results
  if (trace.size() == 0) {
    return {};
  }

  // we don't allow multi path for trace route at the moment, discontinuities force multi route
  int topk = request.options().action() == Options::trace_attributes
                 ? request.options().alternates() + 1
                 : 1;
  auto topk_match_results = matcher->OfflineMatch(trace, topk);

  // Process each score/match result
  std::vector<std::tuple<float, float, std::vector<meili::MatchResult>>> map_match_results;
  for (auto& result : topk_match_results) {
    // There is no path so you're done
    if (result.segments.empty()) {
      throw std::exception{};
    }

    // Form the path edges based on the matched points and populate disconnected edges
    auto paths = MapMatcher::FormPath(matcher.get(), result.results, result.segments, mode_costing,
                                      mode, options);

    // TODO: revisit this, should we always do this? can it go into the functions below?
    // OSRM map matching format has both the match points and the route, fill out the match points
    // here Note that we only support trace_route as OSRM format so best_paths == 1
    if (options.action() == Options::trace_route && options.format() == Options::osrm) {
      graph_tile_ptr tile = nullptr;
      for (int i = 0; i < result.results.size(); ++i) {
        // Get the match
        const auto& match = result.results[i];
        if (!match.edgeid.Is_Valid()) {
          continue;
        }

        // Make one path edge from it
        reader->GetGraphTile(match.edgeid, tile);
        auto* pe = options.mutable_shape(i)->mutable_correlation()->mutable_edges()->Add();
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
          options.mutable_shape(i)->mutable_correlation()->mutable_edges()->Add();
        }
      }
    }

    // trace_attributes always returns a single trip path and may have discontinuities
    if (options.action() == Options::trace_attributes) {
      build_trace(paths, result.results, options, request);
    } // trace_route can return multiple trip paths
    else {
      build_route(paths, result.results, options, request);
    }

    // TODO: move this info to the trip leg
    // Keep the result
    map_match_results.emplace_back(map_match_results.empty()
                                       ? 1.0f
                                       : std::get<kRawScoreIndex>(map_match_results.front()) /
                                             result.score,
                                   result.score, std::move(result.results));
  }

  return map_match_results;
}

void thor_worker_t::build_trace(
    const std::deque<std::pair<std::vector<PathInfo>, std::vector<const meili::EdgeSegment*>>>& paths,
    std::vector<meili::MatchResult>& match_results,
    Options& options,
    Api& request) {

  // couldnt find any match
  if (paths.empty() || paths.front().second.empty())
    throw valhalla_exception_t{442};

  // here we enumerate the discontinuities and set the edge index of each input trace point
  std::unordered_map<size_t, std::pair<EdgeTrimmingInfo, EdgeTrimmingInfo>> edge_trimming;
  baldr::GraphId last_id;
  size_t edge_index = -1;
  for (const auto& path : paths) {
    // remember the global edge index of every input point
    for (const auto* segment : path.second) {
      if (last_id != segment->edgeid) {
        ++edge_index;
      }
      // they can be -1,-1 or l,-1 or -1,h or l,h
      for (int low = (segment->first_match_idx >= 0 ? segment->first_match_idx
                                                    : segment->last_match_idx),
               high = (segment->last_match_idx >= 0 ? segment->last_match_idx
                                                    : segment->first_match_idx);
           low >= 0 && low <= high; ++low) {
        // assign edge_index if edgeid is correct
        if (match_results[low].edgeid == segment->edgeid)
          match_results[low].edge_index = edge_index;
      }
      last_id = segment->edgeid;
    }

    // handle the end of a discontinuity, assumes that there is no start of a discontinuity that is
    // handled below
    const auto* first_segment = path.second.front();
    const auto& first_match = match_results[first_segment->first_match_idx];
    if (first_match.ends_discontinuity) {
      edge_trimming[first_match.edge_index] = {{true, first_match.lnglat, first_match.distance_along},
                                               {false, {}, 1.f}};
    }

    // handle the start of a discontinuity, could be on the same edge where we just ended one. in that
    // case we only touch .second. if there was no discontinuity ending on this edge then we rely on
    // the default initializer for .first when we index the map which sets the distance to 0.f
    const auto* last_segment = path.second.back();
    const auto& last_match = match_results[last_segment->last_match_idx]; // cant use edge_index
    if (last_match.begins_discontinuity) {
      auto found = edge_trimming[last_match.edge_index].second = {true, last_match.lnglat,
                                                                  last_match.distance_along};
    }
  }

  // smash all the path edges into a single vector
  std::vector<PathInfo> path_edges;
  path_edges.reserve(edge_index);
  for (const auto& path : paths) {
    bool merge_last_edge =
        !path_edges.empty() && path_edges.back().edgeid == path.first.front().edgeid;
    path_edges.insert(path_edges.end(), path.first.begin() + merge_last_edge, path.first.end());
  }

  // initialize the origin and destination location for route
  const meili::EdgeSegment* origin_segment = paths.front().second.front();
  const meili::MatchResult& origin_match = match_results[origin_segment->first_match_idx];
  const meili::EdgeSegment* dest_segment = paths.back().second.back();
  const meili::MatchResult& dest_match = match_results[dest_segment->last_match_idx];
  Location* origin_location = options.mutable_shape(&origin_match - &match_results.front());
  Location* destination_location = options.mutable_shape(&dest_match - &match_results.front());

  // we fake up something that looks like the output of loki. segment edge id and matchresult edge ids
  // can disagree at node snaps but leg building requires that we refer to edges in the path. because
  // of that, we use the segment to get edge and percent but we use matchresult for the snap location
  add_path_edge(origin_location, origin_segment->edgeid, origin_segment->source, origin_match.lnglat,
                origin_match.distance_from);
  add_path_edge(destination_location, dest_segment->edgeid, dest_segment->target, dest_match.lnglat,
                dest_match.distance_from);

  // TODO: do we actually need to supply the via/through type locations?

  // actually build the leg and add it to the route
  auto& leg = *request.mutable_trip()->add_routes()->add_legs();
  thor::TripLegBuilder::Build(options, controller, matcher->graphreader(), mode_costing,
                              path_edges.begin(), path_edges.end(), *origin_location,
                              *destination_location, leg, {"map_snap"}, interrupt, edge_trimming);
}

void thor_worker_t::build_route(
    const std::deque<std::pair<std::vector<PathInfo>, std::vector<const meili::EdgeSegment*>>>& paths,
    const std::vector<meili::MatchResult>& match_results,
    Options& options,
    Api& request) {
  // The following logic put break points (matches results) on edge candidates to form legs
  // logic assumes the both match results and edge candidates are topologically sorted in correct
  // order. Only the first location will be populated with corresponding input date_time

  int way_point_index = 0;
  valhalla::TripRoute* route = nullptr;
  std::vector<PathInfo> edges;
  int route_index = 0;
  std::unordered_map<size_t, std::pair<EdgeTrimmingInfo, EdgeTrimmingInfo>> edge_trimming;
  for (const auto& path : paths) {
    if (route == nullptr) {
      route = request.mutable_trip()->mutable_routes()->Add();
      way_point_index = 0;
    }

    const auto* origin_segment = path.second.front();
    int origin_match_idx = origin_segment->first_match_idx;
    const auto* dest_segment = path.second.back();
    int dest_match_idx = dest_segment->last_match_idx;

    for (int i = origin_match_idx; i <= dest_match_idx; ++i) {
      options.mutable_shape(i)->mutable_correlation()->set_route_index(route_index);
      options.mutable_shape(i)->mutable_correlation()->set_waypoint_index(
          std::numeric_limits<uint32_t>::max());
    }

    // initialize the origin and destination location for route
    Location* origin_location = options.mutable_shape(origin_match_idx);
    Location* destination_location = options.mutable_shape(dest_match_idx);

    // when handling multi routes, orsm serializer need to know both the
    // matching_index(route_index) and the waypoint_index(shape_index).
    origin_location->mutable_correlation()->set_waypoint_index(way_point_index);
    destination_location->mutable_correlation()->set_waypoint_index(++way_point_index);

    // we fake up something that looks like the output of loki. segment edge id and matchresult edge
    // ids can disagree at node snaps but leg building requires that we refer to edges in the path.
    // because of that, we use the segment to get edge and percent but we use matchresult for the snap
    // location
    const auto& origin_match = match_results[origin_match_idx];
    const auto& dest_match = match_results[dest_match_idx];
    add_path_edge(origin_location, origin_segment->edgeid, origin_segment->source,
                  origin_match.lnglat, origin_match.distance_from);
    add_path_edge(destination_location, dest_segment->edgeid, dest_segment->target, dest_match.lnglat,
                  dest_match.distance_from);

    // build up the discontinuities so we can trim shape where we do uturns
    edge_trimming.clear();
    for (size_t i = 0; i < path.second.size(); ++i) {
      const auto* prev_segment = i > 0 ? path.second[i - 1] : nullptr;
      const auto* segment = path.second[i];
      const auto* next_segment = i < path.second.size() - 1 ? path.second[i + 1] : nullptr;

      // Note: below we use the operator[] to default initialize the "trim" boolean to false
      // and then we overwrite it. Since its a pair this handles the case when just one is set

      // if we uturn onto this edge we must trim the beginning
      if (prev_segment && prev_segment->edgeid != segment->edgeid && prev_segment->target < 1.f &&
          segment->first_match_idx > -1 && segment->first_match_idx < match_results.size()) {
        edge_trimming[i].first = {true, match_results[segment->first_match_idx].lnglat,
                                  segment->source};
      }
      // if we uturn off of this edge we must trim the end
      if (next_segment && segment->edgeid != next_segment->edgeid && segment->target < 1.f &&
          segment->last_match_idx > -1 && segment->last_match_idx < match_results.size()) {
        edge_trimming[i].second = {true, match_results[segment->last_match_idx].lnglat,
                                   segment->target};
      }
    }

    // TODO: do we actually need to supply the via/through type locations?

    // actually build the leg and add it to the route
    auto& leg = *route->mutable_legs()->Add();
    TripLegBuilder::Build(options, controller, matcher->graphreader(), mode_costing,
                          path.first.cbegin(), path.first.cend(), *origin_location,
                          *destination_location, leg, {"map_snap"}, interrupt, edge_trimming);

    if (path.second.back()->discontinuity) {
      ++route_index;
      route = nullptr;
    }
  }
}

} // namespace thor
} // namespace valhalla
