#include <cmath>

#include "midgard/distanceapproximator.h"
#include "meili/routing.h"
#include "meili/geometry_helpers.h"
#include "meili/match_route.h"
#include "meili/map_matcher.h"
#include "meili/emission_cost_model.h"
#include "meili/transition_cost_model.h"

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
ClockDistance(const Measurement& left, const Measurement& right)
{ return right.epoch_time() < 0 || left.epoch_time() < 0 ?
    -1 : right.epoch_time() - left.epoch_time(); }

struct Interpolation {
  midgard::PointLL projected;
  baldr::GraphId edgeid;
  float sq_distance;
  float route_distance;
  float route_time;
  float edge_distance;

  float sortcost(
      const EmissionCostModel& emission_model,
      const TransitionCostModel& transition_model,
      float gc_dist,
      float clk_dist) const
  {
    const auto transition_cost = transition_model.CalculateTransitionCost(
        0.f,
        route_distance,
        gc_dist,
        route_time,
        clk_dist);
    const auto emission_cost = emission_model.CalculateEmissionCost(sq_distance);
    return transition_cost + emission_cost;
  }
};

inline MatchResult
CreateMatchResult(const Measurement& measurement)
{ return {measurement.lnglat(), 0.f, baldr::GraphId{}, -1.f, measurement.epoch_time(), StateId()}; }

inline MatchResult
CreateMatchResult(const Measurement& measurement, const Interpolation& interp)
{ return {interp.projected, std::sqrt(interp.sq_distance), interp.edgeid, interp.edge_distance, measurement.epoch_time(), StateId()}; }

// Find the interpolation along the route where the transition cost +
// emission cost is minimal
Interpolation
InterpolateMeasurement(
    const MapMatcher& mapmatcher,
    const Measurement& measurement,
    std::vector<EdgeSegment>::const_iterator begin,
    std::vector<EdgeSegment>::const_iterator end,
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
    const auto directededge = mapmatcher.graphreader().directededge(segment->edgeid, tile);
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
    auto route_time = mapmatcher.costing()->EdgeCost(directededge).secs * edge_percent;

    Interpolation interp{projected_point, segment->edgeid, sq_distance, route_distance, route_time, offset};

    const auto cost = interp.sortcost(
        mapmatcher.emission_cost_model(),
        mapmatcher.transition_cost_model(),
        match_measurement_distance,
        match_measurement_time);

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
InterpolateMeasurements(
    const MapMatcher& mapmatcher,
    const std::vector<Measurement>& measurements,
    const StateId& stateid,
    const StateId& next_stateid)
{
  // Nothing to do here
  if (measurements.empty()) {
    return {};
  }

  //can't interpolate these because they don't happen between two valid states or
  //we weren't able to get a downstream route
  std::vector<MatchResult> results;
  if (!stateid.IsValid() || !next_stateid.IsValid()) {
    for (const auto& measurement: measurements) {
      results.push_back(CreateMatchResult(measurement));
    }
    return results;
  }

  std::vector<EdgeSegment> route;
  // route is updated in-place
  MergeRoute(route, mapmatcher.state_container().state(stateid), mapmatcher.state_container().state(next_stateid));

  //for each point that needs interpolated
  for (const auto& measurement: measurements) {
    const auto& match_measurement = mapmatcher.state_container().measurement(stateid.time());
    const auto match_measurement_distance = GreatCircleDistance(measurement, match_measurement);
    const auto match_measurement_time = ClockDistance(measurement, match_measurement);
    //interpolate this point along the route
    const auto& interp = InterpolateMeasurement(
        mapmatcher,
        measurement,
        route.begin(),
        route.end(),
        match_measurement_distance,
        match_measurement_time);

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
FindMatchResult(const MapMatcher& mapmatcher,
                const std::vector<StateId>& stateids,
                StateId::Time time)
{
  if (!(time < stateids.size())) {
    throw std::runtime_error("reading stateid at time out of bounds");
  }

  const auto& prev_stateid = 0 < time ? stateids[time - 1] : StateId(),
                   stateid = stateids[time],
              next_stateid = time + 1 < stateids.size() ? stateids[time + 1] : StateId();
  const auto& measurement = mapmatcher.state_container().measurement(time);

  if (!stateid.IsValid()) {
    return CreateMatchResult(measurement);
  }

  const auto& state = mapmatcher.state_container().state(stateid);
  baldr::GraphId edgeid;

  // Construct the route from previous state to current state, and
  // find out which edge it matches
  if (prev_stateid.IsValid()) {
    const auto& prev_state = mapmatcher.state_container().state(prev_stateid);
    // It must stay on the last edge of the route
    const auto rbegin = prev_state.RouteBegin(state),
                 rend = prev_state.RouteEnd();
    if (rbegin != rend) {
      edgeid = rbegin->edgeid();
    }
  }

  // Do the same from current state to next state
  if (!edgeid.Is_Valid() && next_stateid.IsValid()) {
    const auto& next_state = mapmatcher.state_container().state(next_stateid);
    // It must stay on the first edge of the route
    for (auto label = state.RouteBegin(next_state); label != state.RouteEnd(); label++) {
      if (label->edgeid().Is_Valid()) {
        edgeid = label->edgeid();
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
FindMatchResults(const MapMatcher& mapmatcher, const std::vector<StateId>& stateids)
{
  std::vector<MatchResult> results;

  for (StateId::Time time = 0; time < stateids.size(); time++) {
    results.push_back(FindMatchResult(mapmatcher, stateids, time));
  }

  return results;
}

}


namespace valhalla {
namespace meili {

MapMatcher::MapMatcher(
    const boost::property_tree::ptree& config,
    baldr::GraphReader& graphreader,
    CandidateQuery& candidatequery,
    const sif::cost_ptr_t* mode_costing,
    sif::TravelMode travelmode)
    : config_(config),
      graphreader_(graphreader),
      candidatequery_(candidatequery),
      mode_costing_(mode_costing),
      travelmode_(travelmode),
      interrupt_(nullptr),
      vs_(),
      ts_(vs_),
      container_(),
      emission_cost_model_(
          graphreader_,
          container_,
          config_),
      transition_cost_model_(
          graphreader_,
          vs_,
          ts_,
          container_,
          mode_costing_,
          travelmode_,
          config_)
{
  vs_.set_emission_cost_model(emission_cost_model_);
  vs_.set_transition_cost_model(transition_cost_model_);
}

MapMatcher::~MapMatcher() {}

void MapMatcher::Clear()
{
  vs_.Clear();
  // reset cost models because they were possibly replaced by topk
  vs_.set_emission_cost_model(emission_cost_model_);
  vs_.set_transition_cost_model(transition_cost_model_);
  ts_.Clear();
  container_.Clear();
}

std::vector<std::vector<MatchResult>>
MapMatcher::OfflineMatch(const std::vector<Measurement>& measurements, uint32_t k)
{
  Clear();

  if (measurements.empty()) {
    return {};
  }

  const float max_search_radius = config_.get<float>("max_search_radius"),
           sq_max_search_radius = max_search_radius * max_search_radius;
  const float interpolation_distance = config_.get<float>("interpolation_distance"),
           sq_interpolation_distance = interpolation_distance * interpolation_distance;
  std::unordered_map<StateId::Time, std::vector<Measurement>> interpolated;

  // Always match the first measurement
  auto last = measurements.cbegin();
  auto time = AppendMeasurement(*last, sq_max_search_radius);
  double interpolated_epoch_time = -1;
  for (auto m = std::next(last); m != measurements.end(); ++m) {
    const auto sq_distance = GreatCircleDistanceSquared(*last, *m);
    // Always match the last measurement and if its far enough away
    if (sq_interpolation_distance < sq_distance || std::next(m) == measurements.end()) {
      // If there were interpolated points between these two points with time information
      if (interpolated_epoch_time != -1) {
        // Project the last interpolated point onto the line between the two match points
        auto p = interpolated[time].back().lnglat().Project(last->lnglat(), m->lnglat());
        // If its significantly closer to the previous match point then it looks like the trace lingered
        // so we use the time information of the last interpolation point as the actual time they started
        // traveling towards the next match point which will help us determine what paths are really likely
        if(p.Distance(last->lnglat())/last->lnglat().Distance(m->lnglat()) < .2f) {
          container_.SetMeasurementLeaveTime(time, interpolated_epoch_time);
        }
      }
      // This one isnt interpolated so we make room for its state
      time = AppendMeasurement(*m, sq_max_search_radius);
      last = m;
      interpolated_epoch_time = -1;
    }//TODO: if its the last measurement and it wants to be interpolated
    // then what we need to do is make last match interpolated
    // and copy its epoch_time into the last measurements epoch time
    // else if(std::next(measurement) == measurements.end()) { }
    // This one is so close to the last match that we will just interpolate it
    else {
      interpolated[time].push_back(*m);
      interpolated_epoch_time = m->epoch_time();
    }
  }

  //For k paths
  std::vector<std::vector<MatchResult>> best_paths;
  for(uint32_t i = 0; i < k; ++i) {
    // Get the states for the kth best path its in reverse order
    std::vector<StateId> stateids;
    std::copy(vs_.SearchPath(time), vs_.PathEnd(), std::back_inserter(stateids));
    std::reverse(stateids.begin(), stateids.end());

    std::transform(
        stateids.begin(),
        stateids.end(),
        stateids.begin(),
        [this](const StateId& stateid) {
          const auto& origin = ts_.GetOrigin(stateid);
          return origin.IsValid() ? origin : stateid;
        });

    // Verify that stateids are in correct order
    for (StateId::Time time = 0; time < stateids.size(); time++) {
      if (!(!stateids[time].IsValid() || stateids[time].time() == time)) {
        throw std::logic_error("got state with time " + std::to_string(stateids[time].time()) + " at time " + std::to_string(time));
      }
    }

    // Get the match result for each of the states
    const auto& results = FindMatchResults(*this, stateids);

    // Insert the interpolated results into the result list
    best_paths.emplace_back();
    for (StateId::Time time = 0; time < stateids.size(); time++) {
      // Add in this states result
      best_paths.back().push_back(results[time]);

      // See if there were any interpolated points with this state move on if not
      const auto it = interpolated.find(time);
      if (it == interpolated.end()) {
        continue;
      }

      // Interpolate the points between this and the next state
      const auto& this_stateid = stateids[time];
      const auto& next_stateid = time + 1 < stateids.size() ? stateids[time + 1] : StateId();
      const auto& interpolated_results = InterpolateMeasurements(*this, it->second, this_stateid, next_stateid);

      // Copy the interpolated match results into the final set
      std::copy(interpolated_results.cbegin(), interpolated_results.cend(), std::back_inserter(best_paths.back()));
    }

    // Remove this particular sequence of stateids
    ts_.RemovePath(time);
  }

  //Here are all k paths
  if (!(best_paths.size() == k)) {
    // TODO relax it to be best_paths.size() <= k
    std::logic_error("should get " + std::to_string(k) + " paths but got " + std::to_string(best_paths.size()));
  }

  return best_paths;
}


StateId::Time
MapMatcher::AppendMeasurement(const Measurement& measurement, const float sq_max_search_radius)
{
  // Test interrupt
  if (interrupt_) {
    (*interrupt_)();
  }

  auto sq_radius = std::min(
      sq_max_search_radius,
      std::max(measurement.sq_search_radius(), measurement.sq_gps_accuracy()));

  const auto& candidates = candidatequery_.Query(
      measurement.lnglat(),
      sq_radius,
      costing()->GetEdgeFilter());

  const auto time = container_.AppendMeasurement(measurement);

  for (const auto& candidate: candidates) {
    const auto& stateid = container_.AppendCandidate(candidate);
    vs_.AddStateId(stateid);
  }

  return time;
}

}
}
