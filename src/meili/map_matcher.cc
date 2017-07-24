#include <cmath>

#include "midgard/distanceapproximator.h"
#include "meili/routing.h"
#include "meili/geometry_helpers.h"
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

inline float
ClockDistance(const Measurement& left,
              const Measurement& right)
{ return right.epoch_time() - left.epoch_time(); }


struct Interpolation {
  midgard::PointLL projected;
  baldr::GraphId edgeid;
  float sq_distance;
  float route_distance;
  float route_time;
  float edge_distance;

  float sortcost(const MapMatching& mm, float gc_dist, float clk_dist) const
  { return mm.CalculateTransitionCost(0.f, route_distance, gc_dist, route_time, clk_dist) +
      mm.CalculateEmissionCost(sq_distance); }
};

inline MatchResult
CreateMatchResult(const Measurement& measurement)
{ return {measurement.lnglat(), 0.f, baldr::GraphId{}, -1.f, measurement.epoch_time(), StateId()}; }

inline MatchResult
CreateMatchResult(const Measurement& measurement, const Interpolation& interp)
{ return {interp.projected, std::sqrt(interp.sq_distance), interp.edgeid, interp.edge_distance, measurement.epoch_time(), StateId()}; }

// Find the interpolation along the route where the transition cost +
// emission cost is minimal
template <typename segment_iterator_t>
Interpolation
InterpolateMeasurement(const MapMatching& mapmatching,
                       segment_iterator_t begin,
                       segment_iterator_t end,
                       const Measurement& measurement,
                       float match_measurement_distance,
                       float match_measurement_time)
{
  const baldr::GraphTile* tile(nullptr);
  midgard::DistanceApproximator approximator(measurement.lnglat());

  // Route distance from each segment begin to the beginning segment
  float segment_begin_route_distance = 0.f;
  float minimal_cost = std::numeric_limits<float>::infinity();

  // Invalid edgeid indicates that no interpolation found
  Interpolation best_interp;

  for (auto segment = begin; segment != end; segment++) {
    const auto directededge = mapmatching.graphreader().directededge(segment->edgeid, tile);
    if (!directededge) {
      continue;
    }

    const auto edgeinfo = tile->edgeinfo(directededge->edgeinfo_offset());

    const auto& shape = edgeinfo.shape();
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
    const auto distance_to_segment_ends = std::abs(directededge->length() * (offset - segment->source));

    // The absolute route distance from projected point to the
    // beginning segment
    const auto route_distance = segment_begin_route_distance + distance_to_segment_ends;

    // Get the amount of time spent on this segment
    auto edge_percent = segment->target - segment->source;
    auto route_time = mapmatching.costing()->EdgeCost(directededge).secs * edge_percent;

    Interpolation interp{projected_point, segment->edgeid, sq_distance, route_distance, route_time, offset};
    const auto cost = interp.sortcost(mapmatching, match_measurement_distance, match_measurement_time);
    if (cost < minimal_cost) {
      minimal_cost = cost;
      best_interp = std::move(interp);
    }

    // Assume segments are connected
    segment_begin_route_distance += directededge->length() * edge_percent;
  }

  return best_interp;
}


// Interpolate measurements along the route from state at current time
// to state at next time
std::vector<MatchResult>
InterpolateMeasurements(const MapMatching& mapmatching,
                        const StateId& stateid,
                        const StateId& next_stateid,
                        const std::vector<Measurement>& interpolated_measurements)
{
  // Nothing to do here
  if (interpolated_measurements.empty()) {
    return {};
  }

  //can't interpolate these because they don't happen between two valid states or
  //we weren't able to get a downstream route
  std::vector<MatchResult> results;
  if (!stateid.IsValid() || !next_stateid.IsValid()) {
    for (const auto& measurement: interpolated_measurements) {
      results.push_back(CreateMatchResult(measurement));
    }
    return results;
  }

  std::vector<EdgeSegment> route;
  // route is updated in-place
  MergeRoute(route, mapmatching.state(stateid), mapmatching.state(next_stateid));

  //for each point that needs interpolated
  for (const auto& measurement: interpolated_measurements) {
    const auto& match_measurement = mapmatching.measurement(stateid.time());
    const auto match_measurement_distance = GreatCircleDistance(measurement, match_measurement);
    const auto match_measurement_time = ClockDistance(measurement, match_measurement);
    //interpolate this point along the route
    const auto& interp = InterpolateMeasurement(mapmatching, route.begin(), route.end(),
        measurement, match_measurement_distance, match_measurement_time);

    //if it was able to do the interpolation
    if (interp.edgeid.Is_Valid()) {
      //dont allow subsequent points to get interpolated before this point
      //we do this by editing the route to start where this point was interpolated
      auto itr = std::find_if(route.begin(), route.end(), [&interp](const EdgeSegment& e){ return e.edgeid == interp.edgeid; });
      itr = std::find_if(itr, route.end(), [&interp](const EdgeSegment& e){ return e.edgeid == interp.edgeid && e.target > interp.edge_distance; });
      if(itr != route.end()) {
        itr->source = interp.edge_distance;
        route.erase(route.begin(), itr);
      }
      //keep the interpolated match result
      results.push_back(CreateMatchResult(measurement, interp));
    }//couldnt interpolate this point
    else {
      results.push_back(CreateMatchResult(measurement));
    }
  }

  return results;
}

// Find the match result of a state, given its previous state and next
// state
MatchResult
FindMatchResult(const MapMatching& mapmatching,
                const std::vector<StateId>& stateids,
                StateId::Time time)
{
  if (!(time < stateids.size())) {
    throw std::runtime_error("reading stateid at time out of bounds");
  }

  const auto& prev_stateid = 0 < time ? stateids[time - 1] : StateId(),
                   stateid = stateids[time],
              next_stateid = time + 1 < stateids.size() ? stateids[time + 1] : StateId();
  const auto& measurement = mapmatching.measurement(time);

  if (!stateid.IsValid()) {
    return CreateMatchResult(measurement);
  }

  const auto& state = mapmatching.state(stateid);
  baldr::GraphId edgeid;

  // Construct the route from previous state to current state, and
  // find out which edge it matches
  if (prev_stateid.IsValid()) {
    const auto& prev_state = mapmatching.state(prev_stateid);
    // It must stay on the last edge of the route
    const auto rbegin = prev_state.RouteBegin(state),
                 rend = prev_state.RouteEnd();
    if (rbegin != rend) {
      edgeid = rbegin->edgeid;
    }
  }

  // Do the same from current state to next state
  if (!edgeid.Is_Valid() && next_stateid.IsValid()) {
    const auto& next_state = mapmatching.state(next_stateid);
    // It must stay on the first edge of the route
    for (auto label = state.RouteBegin(next_state); label != state.RouteEnd(); label++) {
      if (label->edgeid.Is_Valid()) {
        edgeid = label->edgeid;
      }
    }
  }

  // If we get a valid edge and find it in the state that's good
  for(const auto& edge : state.candidate().edges) {
    if (edge.id == edgeid) {
      return {edge.projected, std::sqrt(edge.score), edgeid, edge.dist, measurement.epoch_time(), stateid};
    }
  }

  // If we failed to get a valid edge or can't find it
  // At least we know which point it matches
  const auto& edge = state.candidate().edges.front();
  return {edge.projected, std::sqrt(edge.score), edgeid, edge.dist, measurement.epoch_time(), stateid};
}


// Find the corresponding match results of a list of states
std::vector<MatchResult>
FindMatchResults(const MapMatching& mapmatching, const std::vector<StateId>& stateids)
{
  std::vector<MatchResult> results;

  for (StateId::Time time = 0; time < stateids.size(); time++) {
    results.push_back(FindMatchResult(mapmatching, stateids, time));
  }

  return results;
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
      mapmatching_(graphreader_, mode_costing_, travelmode_, config_),
      interrupt_(nullptr) {}


MapMatcher::~MapMatcher() {}


std::vector<MatchResult>
MapMatcher::OfflineMatch(
    const std::vector<Measurement>& measurements, uint32_t k) {
  mapmatching_.Clear();

  const auto begin = measurements.begin(),
               end = measurements.end();

  if (begin == end) {
    return {};
  }

  const auto interpolation_distance = config_.get<float>("interpolation_distance"),
          sq_interpolation_distance = interpolation_distance * interpolation_distance;
  std::unordered_map<StateId::Time, std::vector<Measurement>> interpolated_measurements;

  // Always match the first measurement
  auto measurement = begin;
  auto time = AppendMeasurement(*measurement);
  auto latest_match_measurement = measurement;
  for (measurement++; std::next(measurement) != end; measurement++) {
    const auto sq_distance = GreatCircleDistanceSquared(*latest_match_measurement, *measurement);
    if (sq_interpolation_distance < sq_distance) {
      time = AppendMeasurement(*measurement);
      latest_match_measurement = measurement;
    } else {
      interpolated_measurements[time].push_back(*measurement);
    }
  }
  // Always match the last measurement
  time = AppendMeasurement(*measurement);

  // Search path and put the states into an array reversely
  std::vector<StateId> stateids;
  std::copy(
      mapmatching_.SearchPath(time),
      mapmatching_.PathEnd(),
      std::back_inserter(stateids));
  std::reverse(stateids.begin(), stateids.end());

  // Verify that stateids are in correct order
  for (StateId::Time time = 0; time < stateids.size(); time++) {
    if (!(!stateids[time].IsValid() || stateids[time].time() == time)) {
      std::logic_error("got state with time " + std::to_string(stateids[time].time())
                       + " at time " + std::to_string(time));
    }
  }

  const auto& results = FindMatchResults(mapmatching_, stateids);

  // Done if no measurements to interpolate
  if (interpolated_measurements.empty()) {
    return results;
  }

  // Insert the interpolated results into the result list
  std::vector<MatchResult> merged_results;
  for (StateId::Time time = 0; time < stateids.size(); time++) {
    merged_results.push_back(results[time]);

    const auto it = interpolated_measurements.find(time);
    if (it == interpolated_measurements.end()) {
      continue;
    }

    const auto& interpolated_results = InterpolateMeasurements(
        mapmatching_,
        stateids[time],
        time + 1 < stateids.size() ? stateids[time + 1] : StateId(),
        it->second);

    std::copy(
        interpolated_results.cbegin(),
        interpolated_results.cend(),
        std::back_inserter(merged_results));
  }

  return merged_results;
}


StateId::Time
MapMatcher::AppendMeasurement(const Measurement& measurement)
{
  // Test interrupt
  if (interrupt_) {
    (*interrupt_)();
  }
  const auto& candidates = candidatequery_.Query(
      measurement.lnglat(),
      std::max(measurement.sq_search_radius(), measurement.sq_gps_accuracy()),
      mapmatching_.costing()->GetEdgeFilter());
  return mapmatching_.AppendState(measurement, candidates.begin(), candidates.end());
}

}
}
