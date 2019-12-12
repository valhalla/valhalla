#include <cmath>

#include "meili/emission_cost_model.h"
#include "meili/geometry_helpers.h"
#include "meili/map_matcher.h"
#include "meili/routing.h"
#include "meili/transition_cost_model.h"
#include "midgard/distanceapproximator.h"

namespace {

using namespace valhalla;
using namespace valhalla::meili;

constexpr float MAX_ACCUMULATED_COST = 99999999;

inline float GreatCircleDistanceSquared(const Measurement& left, const Measurement& right) {
  return left.lnglat().DistanceSquared(right.lnglat());
}

inline float GreatCircleDistance(const Measurement& left, const Measurement& right) {
  return left.lnglat().Distance(right.lnglat());
}

inline float ClockDistance(const Measurement& left, const Measurement& right) {
  return right.epoch_time() < 0 || left.epoch_time() < 0 ? -1
                                                         : right.epoch_time() - left.epoch_time();
}

struct Interpolation {
  midgard::PointLL projected;
  baldr::GraphId edgeid;
  float sq_distance;
  float route_distance;
  float route_time;
  float edge_distance;

  float sortcost(const EmissionCostModel& emission_model,
                 const TransitionCostModel& transition_model,
                 float gc_dist,
                 float clk_dist) const {
    const auto transition_cost =
        transition_model.CalculateTransitionCost(0.f, route_distance, gc_dist, route_time, clk_dist);
    const auto emission_cost = emission_model.CalculateEmissionCost(sq_distance);
    return transition_cost + emission_cost;
  }
};

inline MatchResult CreateMatchResult(const Measurement& measurement) {
  return {measurement.lnglat(), 0.f, baldr::GraphId{}, -1.f, measurement.epoch_time(), StateId()};
}

inline MatchResult CreateMatchResult(const Measurement& measurement, const Interpolation& interp) {
  return {interp.projected,     std::sqrt(interp.sq_distance), interp.edgeid,
          interp.edge_distance, measurement.epoch_time(),      StateId()};
}

// Find the interpolation along the route where the transition cost +
// emission cost is minimal
Interpolation InterpolateMeasurement(const MapMatcher& mapmatcher,
                                     const Measurement& measurement,
                                     std::vector<EdgeSegment>::const_iterator begin,
                                     std::vector<EdgeSegment>::const_iterator end,
                                     float match_measurement_distance,
                                     float match_measurement_time) {
  const baldr::GraphTile* tile(nullptr);
  midgard::projector_t projector(measurement.lnglat());

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

    auto shape = edgeinfo.lazy_shape();
    if (shape.empty()) {
      continue;
    }

    midgard::PointLL projected_point;
    float sq_distance, offset;
    std::tie(projected_point, sq_distance, std::ignore, offset) = helpers::Project(projector, shape);

    // Find out the correct offset
    if (!directededge->forward()) {
      offset = 1.f - offset;
    }

    // Distance from the projected point to the segment begin, or
    // segment begin if we walk reversely
    const auto distance_to_segment_ends =
        std::abs(directededge->length() * (offset - segment->source));

    // The absolute route distance from projected point to the
    // beginning segment
    const auto route_distance = segment_begin_route_distance + distance_to_segment_ends;

    // Get the amount of time spent on this segment
    auto edge_percent = segment->target - segment->source;
    auto route_time = mapmatcher.costing()->EdgeCost(directededge, tile).secs * edge_percent;

    Interpolation interp{projected_point, segment->edgeid, sq_distance,
                         route_distance,  route_time,      offset};

    const auto cost =
        interp.sortcost(mapmatcher.emission_cost_model(), mapmatcher.transition_cost_model(),
                        match_measurement_distance, match_measurement_time);

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
std::vector<MatchResult> InterpolateMeasurements(const MapMatcher& mapmatcher,
                                                 const std::vector<Measurement>& measurements,
                                                 const StateId& stateid,
                                                 const StateId& next_stateid) {
  // Nothing to do here
  if (measurements.empty()) {
    return {};
  }

  // can't interpolate these because they don't happen between two valid states or
  // we weren't able to get a downstream route
  std::vector<MatchResult> results;
  if (!stateid.IsValid() || !next_stateid.IsValid()) {
    for (const auto& measurement : measurements) {
      results.push_back(CreateMatchResult(measurement));
    }
    return results;
  }

  std::vector<EdgeSegment> route = MergeRoute(mapmatcher.state_container().state(stateid),
                                              mapmatcher.state_container().state(next_stateid));

  // for each point that needs interpolated
  for (const auto& measurement : measurements) {
    const auto& match_measurement = mapmatcher.state_container().measurement(stateid.time());
    const auto match_measurement_distance = GreatCircleDistance(measurement, match_measurement);
    const auto match_measurement_time = ClockDistance(measurement, match_measurement);
    // interpolate this point along the route
    const auto& interp = InterpolateMeasurement(mapmatcher, measurement, route.begin(), route.end(),
                                                match_measurement_distance, match_measurement_time);

    // if it was able to do the interpolation
    if (interp.edgeid.Is_Valid()) {
      // dont allow subsequent points to get interpolated before this point
      // we do this by editing the route to start where this point was interpolated
      auto itr = std::find_if(route.begin(), route.end(),
                              [&interp](const EdgeSegment& e) { return e.edgeid == interp.edgeid; });
      itr = std::find_if(itr, route.end(), [&interp](const EdgeSegment& e) {
        return e.edgeid == interp.edgeid && e.target > interp.edge_distance;
      });
      if (itr != route.end()) {
        itr->source = interp.edge_distance;
        route.erase(route.begin(), itr);
      }
      // keep the interpolated match result
      results.push_back(CreateMatchResult(measurement, interp));
    } // couldnt interpolate this point
    else {
      results.push_back(CreateMatchResult(measurement));
    }
  }

  return results;
}

// Find the match result of a state, given its previous state and next
// state
MatchResult FindMatchResult(const MapMatcher& mapmatcher,
                            const std::vector<StateId>& stateids,
                            StateId::Time time) {
  if (!(time < stateids.size())) {
    throw std::runtime_error("reading stateid at time out of bounds");
  }

  const auto &prev_stateid = 0 < time ? stateids[time - 1] : StateId(), stateid = stateids[time],
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
    const auto rbegin = prev_state.RouteBegin(state), rend = prev_state.RouteEnd();
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
  for (const auto& edge : state.candidate().edges) {
    if (edge.id == edgeid) {
      return {edge.projected,     std::sqrt(edge.distance), edgeid,
              edge.percent_along, measurement.epoch_time(), stateid};
    }
  }

  // If we failed to get a valid edge or can't find it
  // At least we know which point it matches
  const auto& edge = state.candidate().edges.front();
  return {edge.projected,     std::sqrt(edge.distance), edgeid,
          edge.percent_along, measurement.epoch_time(), stateid};
}

// Find the corresponding match results of a list of states
std::vector<MatchResult> FindMatchResults(const MapMatcher& mapmatcher,
                                          const std::vector<StateId>& stateids) {
  std::vector<MatchResult> results;

  for (StateId::Time time = 0; time < stateids.size(); time++) {
    results.push_back(FindMatchResult(mapmatcher, stateids, time));
  }

  return results;
}

struct path_t {
  path_t(const std::vector<EdgeSegment>& segments) {
    edges.reserve(segments.size());
    for (const auto& segment : segments) {
      if (edges.empty() || edges.back() != segment.edgeid) {
        edges.push_back(segment.edgeid);
      }
    }
    e1 = segments.empty() || segments.front().source < 1.0f ? edges.cbegin() : edges.cbegin() + 1;
    e2 = segments.empty() || segments.back().target > 0.0f ? edges.cend() : edges.cend() - 1;
  }
  bool operator!=(const path_t& p) const {
    return std::search(e1, e2, p.e1, p.e2) == e2 && std::search(p.e1, p.e2, e1, e2) == p.e2;
  }
  bool crosses(const baldr::PathLocation& candidate) const {
    for (const auto& edge : candidate.edges) {
      if (std::find(edges.cbegin(), edges.cend(), edge.id) != edges.cend()) {
        return true;
      }
    }
    return false;
  }
  std::vector<uint64_t> edges;
  std::vector<uint64_t>::const_iterator e1;
  std::vector<uint64_t>::const_iterator e2;
};

} // namespace

namespace valhalla {
namespace meili {

MapMatcher::MapMatcher(const boost::property_tree::ptree& config,
                       baldr::GraphReader& graphreader,
                       CandidateQuery& candidatequery,
                       const sif::cost_ptr_t* mode_costing,
                       sif::TravelMode travelmode)
    : config_(config), graphreader_(graphreader), candidatequery_(candidatequery),
      mode_costing_(mode_costing), travelmode_(travelmode), interrupt_(nullptr), vs_(), ts_(vs_),
      container_(), emission_cost_model_(graphreader_, container_, config_),
      transition_cost_model_(graphreader_,
                             vs_,
                             ts_,
                             container_,
                             mode_costing_,
                             travelmode_,
                             config_) {
  vs_.set_emission_cost_model(emission_cost_model_);
  vs_.set_transition_cost_model(transition_cost_model_);
}

MapMatcher::~MapMatcher() {
}

void MapMatcher::Clear() {
  vs_.Clear();
  // reset cost models because they were possibly replaced by topk
  vs_.set_emission_cost_model(emission_cost_model_);
  vs_.set_transition_cost_model(transition_cost_model_);
  ts_.Clear();
  container_.Clear();
}

void MapMatcher::RemoveRedundancies(const std::vector<StateId>& result) {
  if (result.empty()) {
    return;
  }
  // For each pair of states in the last sequence of states
  for (auto left_state_id_itr = result.cbegin(); left_state_id_itr != result.cend() - 1;
       ++left_state_id_itr) {
    // Get all the paths that use the left winner
    auto time = left_state_id_itr->time();
    auto left_used_candidate = container_.state(*left_state_id_itr);
    std::unordered_map<StateId, path_t> paths_from_winner;
    for (const auto& right_candidate : container_.column(time + 1)) {
      std::vector<EdgeSegment> edges;
      if (!ts_.IsRemoved(right_candidate.stateid()) &&
          MergeRoute(edges, left_used_candidate, right_candidate)) {
        paths_from_winner.emplace(right_candidate.stateid(), std::move(edges));
      }
    }
    /*
        std::cout << std::endl << "Paths from left winner:" << std::endl;
        for(const auto& kv:paths_from_winner) {
          std::cout << R"({"type":"FeatureCollection","features":[)";
          std::cout << container_.geojson(left_used_candidate) << ',' <<
       container_.geojson(kv.first) << R"(]})" << std::endl;
        }
    */
    // For each candidate of the left state that isnt the winner
    std::unordered_set<StateId> right_uniques;
    std::unordered_set<StateId> redundancies;
    for (const auto& left_unused_candidate : container_.column(time)) {
      // We cant remove candidates that were used in the result or already removed
      if (left_used_candidate.stateid() == left_unused_candidate.stateid() ||
          ts_.IsRemoved(left_unused_candidate.stateid())) {
        continue;
      }

      // If we didnt compute the paths from this loser we need to do it now
      if (!left_unused_candidate.routed()) {
        vs_.transition_cost_model()(left_unused_candidate.stateid(), StateId(time + 1, 0));
      }

      // For each candidate in the right state
      bool found_unique = false;
      // std::cout << std::endl << "Paths from left loser:" << std::endl;
      for (const auto& right_candidate : container_.column(time + 1)) {
        /*
                std::cout << R"({"type":"FeatureCollection","features":[)";
                std::cout << container_.geojson(left_unused_candidate) << ',' <<
           container_.geojson(right_candidate); std::cout << R"(]})" << std::endl;
        */

        // If there is no route its not really unique since we dont need discontinuities
        std::vector<EdgeSegment> edges;
        if (ts_.IsRemoved(right_candidate.stateid()) ||
            !MergeRoute(edges, left_unused_candidate, right_candidate)) {
          continue;
        }

        // This subpath is unique if: there is no path from the winner
        // or its node to node (no edges) and not on winner path or they dont overlap
        path_t path_from_loser(std::move(edges));
        auto path_from_winner = paths_from_winner.find(right_candidate.stateid());
        if (path_from_winner == paths_from_winner.end() ||
            (path_from_loser.edges.empty() &&
             !path_from_winner->second.crosses(right_candidate.candidate())) ||
            path_from_winner->second != path_from_loser) {
          found_unique = true;
          right_uniques.emplace(right_candidate.stateid());
          break;
        }
      }

      // We didnt find any unique paths for this left candidate so we need to mark it has useless
      if (!found_unique) {
        redundancies.emplace(left_unused_candidate.stateid());
      }
    }

    /*
        if(redundancies.size()) {
          std::cout << std::endl << "Removing: " << R"({"type":"FeatureCollection","features":[)";
          std::string fsep = "";
          for(auto r : redundancies) { std::cout << fsep << container_.geojson(r); fsep = ","; }
          std::cout << R"(]})" << std::endl;
        }
    */

    // Clean up the left hand redundancies
    for (const auto& r : redundancies) {
      ts_.RemoveStateId(r);
      vs_.RemoveStateId(r);
    }

    // If this was the last state pair we can possibly remove some of the right candidates
    if (left_state_id_itr == result.cend() - 2) {
      auto right_state_id = *(left_state_id_itr + 1);
      // For each right candidate
      redundancies.clear();
      for (const auto& right_candidate : container_.column(time + 1)) {
        // If its not the winner and there are no uniques for this right candidate we can remove
        // it
        if (right_candidate.stateid() != right_state_id &&
            right_uniques.find(right_candidate.stateid()) == right_uniques.cend()) {
          redundancies.emplace(right_candidate.stateid());
        }
      }

      /*
            if(redundancies.size()) {
              std::cout << std::endl << "Removing: " <<
         R"({"type":"FeatureCollection","features":[)"; std::string fsep = ""; for(auto r :
         redundancies) { std::cout << fsep << container_.geojson(r); fsep = ","; } std::cout <<
         R"(]})" << std::endl;
            }
      */

      // Cleanup the right hand redundancies
      for (const auto& r : redundancies) {
        ts_.RemoveStateId(r);
        vs_.RemoveStateId(r);
      }
    }
  }
}

// Mark any sub paths (pairs of candidates between two states) that are redundant with this one
/*void MapMatcher::RemoveRedundancies(const path_t& path, std::vector<StateId>& state_ids) {
  // For each state pair
  for(StateId::Time time = 0; time < state_ids.back().time(); ++time) {
    // For each left candidate
    for(const auto& lhs : container_.column(time)) {
      // If we dont have routes from this state
      if(!lhs.routed())
        vs_.transition_cost_model()(lhs.stateid(), StateId(time + 1, 0));

      // For each right candidate
      for(const auto& rhs : container_.column(time + 1)) {

        // If this is the path we already used skip it
        if(lhs.stateid() == state_ids[time] && rhs.stateid() == state_ids[time + 1])
          continue;

        // Get the path for this pair of candidates
        path_t sub_path(MergeRoute(lhs, rhs));

        // If this is redundant if its empty (because we dont want discontinuities in subsequent
results
        // or the first subpath starts the original path or the last subpath ends the original
path
or
        // the intermediate subpath is contained within the original path
        if(sub_path.edges.empty() ||
           (time == state_ids.front().time() && sub_path.start_of(path)) ||
           (time == state_ids.back().time() - 1 && sub_path.end_of(path)) ||
           sub_path.within(path)) {

          // Swap out the real candidates for the redundant ones
          auto lid = lhs.stateid(); std::swap(lid, state_ids[time]);
          auto rid = rhs.stateid(); std::swap(rid, state_ids[time + 1]);

          std::cout << std::endl << "Removing: ";
          std::cout << R"({"type":"FeatureCollection","features":[)";
          std::string fsep = "";
          for(auto s : state_ids) {
            std::cout << fsep << container_.geojson(s);
            fsep = ",";
          }
          std::cout << R"(]})" << std::endl;

          // Mark them redundant in searching
          ts_.RemovePath(state_ids);
          // Swap back the real candidates for the redundant ones
          std::swap(lid, state_ids[time]);
          std::swap(rid, state_ids[time + 1]);
          // TODO: Mark them so we dont consider them again in another pass
        }
        else {
          auto lid = lhs.stateid(); std::swap(lid, state_ids[time]);
          auto rid = rhs.stateid(); std::swap(rid, state_ids[time + 1]);

          std::cout << std::endl << "Keeping: ";
          std::cout << R"({"type":"FeatureCollection","features":[)";
          std::string fsep = "";
          for(auto s : state_ids) {
            std::cout << fsep << container_.geojson(s);
            fsep = ",";
          }
          std::cout << R"(]})" << std::endl;

          std::swap(lid, state_ids[time]);
          std::swap(rid, state_ids[time + 1]);
        }
      }
    }
  }
}*/

std::vector<MatchResults> MapMatcher::OfflineMatch(const std::vector<Measurement>& measurements,
                                                   uint32_t k) {
  if (k <= 0) {
    throw std::invalid_argument("expect k to be positive but got " + std::to_string(k));
  }

  // Reset everything
  Clear();

  std::vector<MatchResults> best_paths;

  // Nothing to do
  if (measurements.empty()) {
    best_paths.emplace_back(std::vector<MatchResult>{}, std::vector<EdgeSegment>{}, 0);
    return best_paths;
  }

  bool found_broken_path = false;

  // Separate the measurements we are using for matching from the ones we'll just interpolate
  auto interpolated = AppendMeasurements(measurements);

  // For k paths
  while (best_paths.size() < k && !found_broken_path) {
    // Get the states for the kth best path in reversed order then fix the order
    std::vector<StateId> state_ids;
    double accumulated_cost = 0.f;
    while (state_ids.size() < container_.size()) {
      // Get the time at the last column of states
      const auto time = container_.size() - state_ids.size() - 1;
      std::copy(vs_.SearchPath(time, false), vs_.PathEnd(), std::back_inserter(state_ids));
      const auto& winner = vs_.SearchWinner(time);
      if (winner.IsValid()) {
        accumulated_cost += vs_.AccumulatedCost(winner);
      } else {
        // TODO need a sane constant cost for invalid state
        accumulated_cost += MAX_ACCUMULATED_COST;
        found_broken_path = true;
      }

      if (state_ids.size() < container_.size()) {
        found_broken_path = true;
        // cost for disconnection
        accumulated_cost += MAX_ACCUMULATED_COST;
      }
    }

    // early quit if we found best paths
    if (!best_paths.empty() && found_broken_path) {
      break;
    }

    // Get back the real state ids in order
    std::vector<StateId> original_state_ids;
    for (auto s_itr = state_ids.rbegin(); s_itr != state_ids.rend(); ++s_itr) {
      original_state_ids.push_back(ts_.GetOrigin(*s_itr, *s_itr));
    }

    // Verify that stateids are in correct order
    for (StateId::Time time = 0; time < original_state_ids.size(); time++) {
      if (!original_state_ids[time].IsValid()) {
        continue;
      }
      if (original_state_ids[time].time() != time) {
        throw std::logic_error("got state with time " +
                               std::to_string(original_state_ids[time].time()) + " at time " +
                               std::to_string(time));
      }
    }

    // Get the match result for each of the states
    auto results = FindMatchResults(*this, original_state_ids);

    // Insert the interpolated results into the result list
    std::vector<MatchResult> best_path;
    for (StateId::Time time = 0; time < original_state_ids.size(); time++) {
      // Add in this states result
      best_path.emplace_back(std::move(results[time]));

      // See if there were any interpolated points with this state move on if not
      const auto it = interpolated.find(time);
      if (it == interpolated.end()) {
        continue;
      }

      // Interpolate the points between this and the next state
      const auto& this_stateid = original_state_ids[time];
      const auto& next_stateid =
          time + 1 < original_state_ids.size() ? original_state_ids[time + 1] : StateId();
      const auto& interpolated_results =
          InterpolateMeasurements(*this, it->second, this_stateid, next_stateid);

      // Copy the interpolated match results into the final set
      std::copy(interpolated_results.cbegin(), interpolated_results.cend(),
                std::back_inserter(best_path));
    }

    // Construct a result
    auto segments = ConstructRoute(*this, best_path.cbegin(), best_path.cend());
    MatchResults match_results(std::move(best_path), std::move(segments), accumulated_cost);

    // We'll keep it if we don't have a duplicate already
    auto found_path = std::find(best_paths.rbegin(), best_paths.rend(), match_results);
    if (found_path == best_paths.rend()) {
      /*std::cout << "Result: " << best_paths.size() << std::endl;
      std::cout << R"({"type":"FeatureCollection","features":[)";
      std::string fsep = "";
      for(auto s : original_state_ids) {
        std::cout << fsep << container_.geojson(s);
        fsep = ",";
      }
      std::cout << R"(]})" << std::endl;*/
      best_paths.emplace_back(std::move(match_results));
    }

    // RemoveRedundancies doesn't work with broken paths yet,
    // also we want to avoid removing path for the last best path
    if (!found_broken_path && best_paths.size() < k) {
      // Remove all the candidates pairs whose paths are redundant with this one
      RemoveRedundancies(original_state_ids);
      // Remove this particular sequence of stateids
      ts_.RemovePath(state_ids);
      // Prepare for a fresh search in the next search iteration
      vs_.ClearSearch();
    }
  }

  if (!(0 < best_paths.size() && best_paths.size() <= k)) {
    throw std::logic_error("got " + std::to_string(best_paths.size()) +
                           " paths but k = " + std::to_string(k));
  }

  // Give back anywhere from 1 to k results
  return best_paths;
}

std::unordered_map<StateId::Time, std::vector<Measurement>>
MapMatcher::AppendMeasurements(const std::vector<Measurement>& measurements) {
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
        // If its significantly closer to the previous match point then it looks like the trace
        // lingered so we use the time information of the last interpolation point as the actual
        // time they started traveling towards the next match point which will help us determine
        // what paths are really likely
        if (p.Distance(last->lnglat()) / last->lnglat().Distance(m->lnglat()) < .2f) {
          container_.SetMeasurementLeaveTime(time, interpolated_epoch_time);
        }
      }
      // This one isnt interpolated so we make room for its state
      time = AppendMeasurement(*m, sq_max_search_radius);
      last = m;
      interpolated_epoch_time = -1;
    } // TODO: if its the last measurement and it wants to be interpolated
    // then what we need to do is make last match interpolated
    // and copy its epoch_time into the last measurements epoch time
    // else if(std::next(measurement) == measurements.end()) { }
    // This one is so close to the last match that we will just interpolate it
    else {
      interpolated[time].push_back(*m);
      interpolated_epoch_time = m->epoch_time();
    }
  }

  return interpolated;
}

StateId::Time MapMatcher::AppendMeasurement(const Measurement& measurement,
                                            const float sq_max_search_radius) {
  // Test interrupt
  if (interrupt_) {
    (*interrupt_)();
  }

  auto sq_radius = std::min(sq_max_search_radius,
                            std::max(measurement.sq_search_radius(), measurement.sq_gps_accuracy()));

  const auto& candidates =
      candidatequery_.Query(measurement.lnglat(), sq_radius, costing()->GetEdgeFilter());

  const auto time = container_.AppendMeasurement(measurement);

  //  std::string fsep = "";
  //  std::cout << std::endl << "Candidates at time " << time << std::endl <<
  //  R"({"type":"FeatureCollection","features":[)";
  for (const auto& candidate : candidates) {
    const auto& stateid = container_.AppendCandidate(candidate);
    vs_.AddStateId(stateid);

    //    std::cout << fsep << container_.geojson(stateid);
    //    fsep = ",";
  }
  //  std::cout << R"(]})" << std::endl;

  return time;
}

} // namespace meili
} // namespace valhalla
