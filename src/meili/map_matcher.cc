#include <valhalla/midgard/distanceapproximator.h>

#include "meili/graph_helpers.h"
#include "meili/routing.h"
#include "meili/geometry_helpers.h"
#include "meili/graph_helpers.h"
#include "meili/match_route.h"
#include "meili/map_matcher.h"


namespace {

using namespace valhalla;
using namespace valhalla::meili;


inline float
GreatCircleDistanceSquared(const Measurement& left,
                           const Measurement& right)
{ return left.lnglat().DistanceSquared(right.lnglat()); }


inline float
GreatCircleDistance(const Measurement& left,
                    const Measurement& right)
{ return left.lnglat().Distance(right.lnglat()); }


struct Interpolation {
  midgard::PointLL projected;
  baldr::GraphId edgeid;
  float sq_distance;
  float route_distance;

  float sortcost(const MapMatching& mm, float mmt_dist) const
  { return mm.CalculateTransitionCost(0.f, route_distance, mmt_dist) + mm.CalculateEmissionCost(sq_distance); }
};


// Find the interpolation along the route where the transitin cost +
// emission cost is minimal
template <typename segment_iterator_t>
Interpolation
InterpolateMeasurement(const MapMatching& mapmatching,
                       segment_iterator_t begin,
                       segment_iterator_t end,
                       const Measurement& measurement,
                       float match_measurement_distance,
                       bool reverse)
{
  const baldr::GraphTile* tile(nullptr);
  midgard::DistanceApproximator approximator(measurement.lnglat());

  // Route distance from each segment begin to the beginning segment
  float segment_begin_route_distance = 0.f;
  float minimal_cost = std::numeric_limits<float>::infinity();

  // Invalid edgeid indicates that no interpolation found
  Interpolation best_interp{{}, {}, 0.f, 0.f};

  for (auto segment = begin; segment != end; segment++) {
    const auto directededge = helpers::edge_directededge(mapmatching.graphreader(), segment->edgeid, tile);
    if (!directededge) {
      continue;
    }

    const auto edgeinfo = tile->edgeinfo(directededge->edgeinfo_offset());

    const auto& shape = edgeinfo->shape();
    if (shape.empty()) {
      continue;
    }

    midgard::PointLL projected_point;
    float sq_distance, offset;
    std::tie(projected_point, sq_distance, std::ignore, offset) = helpers::Project(measurement.lnglat(), shape, approximator);

    // Find out the correct offset
    if (!directededge->forward()) {
      offset = 1.f - offset;
    }

    // Distance from the projected point to the segment begin, or
    // segment begin if we walk reversely
    const auto distance_to_segment_ends = std::abs(directededge->length() * (reverse? (segment->target - offset) : (offset - segment->source)));

    // The absolute route distance from projected point to the
    // beginning segment
    const auto route_distance = segment_begin_route_distance + distance_to_segment_ends;

    Interpolation interp{projected_point, segment->edgeid, sq_distance, route_distance};
    const auto cost = interp.sortcost(mapmatching, match_measurement_distance);
    if (cost < minimal_cost) {
      minimal_cost = cost;
      best_interp = std::move(interp);
    }

    // Assume segments are connected
    segment_begin_route_distance += directededge->length() * (segment->target - segment->source);
  }

  return best_interp;
}


// Iterpolate measurements along the route from prevous state to
// current state, or along the route from current state to next state
std::vector<MatchResult>
InterpolateMeasurements(const MapMatching& mapmatching,
                        const MapMatching::state_iterator& previous_state,
                        const MapMatching::state_iterator& state,
                        const MapMatching::state_iterator& next_state,
                        const std::vector<Measurement>& interpolated_measurements)
{
  if (interpolated_measurements.empty()) {
    return {};
  }

  std::vector<MatchResult> results;

  if (!state.IsValid()) {
    for (const auto& measurement: interpolated_measurements) {
      results.emplace_back(measurement.lnglat());
    }
    return results;
  }

  std::vector<EdgeSegment> upstream_route;
  if (previous_state.IsValid()) {
    MergeRoute(upstream_route, *previous_state, *state);
  }

  std::vector<EdgeSegment> downstream_route;
  if (next_state.IsValid()) {
    MergeRoute(downstream_route, *state, *next_state);
  }

  if (upstream_route.empty() && downstream_route.empty()) {
    for (const auto& measurement: interpolated_measurements) {
      results.emplace_back(measurement.lnglat());
    }
    return results;
  }

  for (const auto& measurement: interpolated_measurements) {
    const auto& match_measurement = mapmatching.measurement(state->time());
    const auto match_measurement_distance = GreatCircleDistance(measurement, match_measurement);

    const auto& up_interp = InterpolateMeasurement(
        mapmatching,
        upstream_route.rbegin(),
        upstream_route.rend(),
        measurement,
        match_measurement_distance,
        true);

    const auto& down_interp = InterpolateMeasurement(
        mapmatching,
        downstream_route.begin(),
        downstream_route.end(),
        measurement,
        match_measurement_distance,
        false);

    if (up_interp.edgeid.Is_Valid() && down_interp.edgeid.Is_Valid()) {
      const auto down_cost = down_interp.sortcost(mapmatching, match_measurement_distance),
                   up_cost = up_interp.sortcost(mapmatching, match_measurement_distance);
      if (down_cost < up_cost) {
        results.emplace_back(down_interp.projected, std::sqrt(down_interp.sq_distance), down_interp.edgeid);
      } else {
        results.emplace_back(up_interp.projected, std::sqrt(up_interp.sq_distance), up_interp.edgeid);
      }

    } else if (up_interp.edgeid.Is_Valid()) {
      results.emplace_back(up_interp.projected, std::sqrt(up_interp.sq_distance), up_interp.edgeid);

    } else if (down_interp.edgeid.Is_Valid()) {
      results.emplace_back(down_interp.projected, std::sqrt(down_interp.sq_distance), down_interp.edgeid);

    } else {
      results.emplace_back(measurement.lnglat());
    }
  }

  return results;
}


// Interplolate measurements grouped by previous match measurement
// time
std::unordered_map<Time, std::vector<MatchResult>>
InterpolateTimedMeasurements(const MapMatching& mapmatching,
                             const MapMatching::state_iterator& state_rbegin,
                             const MapMatching::state_iterator& state_rend,
                             const std::unordered_map<Time, std::vector<Measurement>>& interpolated_measurements,
                             Time time)
{
  std::unordered_map<Time, std::vector<MatchResult>> resultmap;

  for (auto previous_state = std::next(state_rbegin),
                     state = state_rbegin,
                next_state = MapMatching::state_iterator(nullptr);
       state != state_rend;
       next_state = state, state++, previous_state = std::next(state)) {

    const auto it = interpolated_measurements.find(time);
    if (it != interpolated_measurements.end()) {
      if (state.IsValid()) {
        const auto& results = InterpolateMeasurements(mapmatching, previous_state, state, next_state, it->second);
        resultmap.emplace(time, results);
      } else {
        auto& results = resultmap[time];
        for (const auto& measurement: it->second) {
          results.push_back(MatchResult(measurement.lnglat()));
        }
      }
    }

    time--;
  }

  return resultmap;
}


// Find the match result of a state, given its previous state and next
// state
MatchResult
FindMatchResult(const MapMatching::state_iterator& previous_state,
                const State& state,
                const MapMatching::state_iterator& next_state)
{
  baldr::GraphId edgeid;

  // Construct the route from previous state to current state, and
  // find out which edge it matches
  if (previous_state.IsValid()) {
    // It must stay on the last edge of the route
    const auto rbegin = previous_state->RouteBegin(state),
                 rend = previous_state->RouteEnd();
    if (rbegin != rend) {
      edgeid = rbegin->edgeid;
    }
  }

  // Do the same from current state to next state
  if (!edgeid.Is_Valid() && next_state.IsValid()) {
    // It must stay on the first edge of the route
    for (auto label = state.RouteBegin(*next_state);
         label != state.RouteEnd(); label++) {
      if (label->edgeid.Is_Valid()) {
        edgeid = label->edgeid;
      }
    }
  }

  // If we failed to infer the route and the edge, at least we know
  // which point it matches
  const auto& c = state.candidate();
  //Note: technically a candidate can have correlated to more than one place in the graph
  //but the way its used in meili we only correlated it to one place so .front() is safe
  return {c.edges.front().projected, std::sqrt(c.edges.front().score), edgeid, state.id()};
}


// Find the corresponding match results of a list of states
std::vector<MatchResult>
FindMatchResults(const MapMatching& mapmatching,
                 const MapMatching::state_iterator& state_rbegin,
                 const MapMatching::state_iterator& state_rend,
                 Time time)
{
  if (state_rbegin == state_rend) {
    return {};
  }

  std::vector<MatchResult> results;

  for (auto previous_state = std::next(state_rbegin),
                     state = state_rbegin,
                next_state = MapMatching::state_iterator(nullptr);
       state != state_rend;
       next_state = state, state++, previous_state = std::next(state)) {
    if (state.IsValid()) {
      results.push_back(FindMatchResult(previous_state, *state, next_state));
    } else {
      results.emplace_back(mapmatching.measurement(time).lnglat());
    }
    time--;
  }

  std::reverse(results.begin(), results.end());
  return results;
}


// Insert the interpolated results into the result list, and
// return a new result list
std::vector<MatchResult>
MergeMatchResults(const std::vector<MatchResult>& results,
                  const std::unordered_map<Time, std::vector<MatchResult>>& interpolated_results)
{
  std::vector<MatchResult> merged_results;

  Time time = 0;
  for (const auto& result: results) {
    merged_results.push_back(result);

    const auto it = interpolated_results.find(time);
    if (it != interpolated_results.end()) {
      for (const auto& result: it->second) {
        merged_results.push_back(result);
      }
    }

    time++;
  }

  return merged_results;
}

}


namespace valhalla {
namespace meili {

MapMatcher::MapMatcher(const boost::property_tree::ptree& config,
                       baldr::GraphReader& graphreader,
                       CandidateQuery& candidatequery,
                       const sif::cost_ptr_t* mode_costing,
                       sif::TravelMode travelmode)
    : config_(config),
      graphreader_(graphreader),
      candidatequery_(candidatequery),
      mode_costing_(mode_costing),
      travelmode_(travelmode),
      mapmatching_(graphreader_, mode_costing_, travelmode_, config_) {}


MapMatcher::~MapMatcher() {}


std::vector<MatchResult>
MapMatcher::OfflineMatch(const std::vector<Measurement>& measurements)
{
  mapmatching_.Clear();

  const auto begin = measurements.begin(),
               end = measurements.end();

  if (begin == end) {
    return {};
  }

  const auto interpolation_distance = config_.get<float>("interpolation_distance"),
          sq_interpolation_distance = interpolation_distance * interpolation_distance;
  std::unordered_map<Time, std::vector<Measurement>> interpolated_measurements;

  // Always match the first measurement
  auto time = AppendMeasurement(*begin);
  auto latest_match_measurement = begin;
  std::size_t match_count = 1;

  for (auto measurement = next(begin); measurement != end; measurement++) {
    const auto sq_distance = GreatCircleDistanceSquared(*latest_match_measurement, *measurement);
    // Always match the last measurement
    if (sq_interpolation_distance < sq_distance || std::next(measurement) == end) {
      time = AppendMeasurement(*measurement);
      latest_match_measurement = measurement;
      match_count++;
    } else {
      interpolated_measurements[time].push_back(*measurement);
    }
  }

  const auto state_rbegin = mapmatching_.SearchPath(time),
               state_rend = std::next(state_rbegin, match_count);
  const auto& results = FindMatchResults(mapmatching_, state_rbegin, state_rend, time);

  // Done if no measurements to interpolate
  if (interpolated_measurements.empty()) {
    return results;
  }

  // Find results for all interpolated measurements
  const auto& interpolated_results = InterpolateTimedMeasurements(
      mapmatching_,
      state_rbegin,
      state_rend,
      interpolated_measurements,
      time);

  // Insert the interpolated results into the result list
  return MergeMatchResults(results, interpolated_results);
}


Time
MapMatcher::AppendMeasurement(const Measurement& measurement)
{
  const auto& candidates = candidatequery_.Query(
      measurement.lnglat(),
      measurement.sq_search_radius(),
      mapmatching_.costing()->GetEdgeFilter());
  return mapmatching_.AppendState(measurement, candidates.begin(), candidates.end());
}

}
}
