#include <cmath>

#include "meili/emission_cost_model.h"
#include "meili/geometry_helpers.h"
#include "meili/map_matcher.h"
#include "meili/routing.h"
#include "meili/transition_cost_model.h"
#include "midgard/distanceapproximator.h"
#include "worker.h"

#include <array>

namespace {

using namespace valhalla;
using namespace valhalla::meili;

constexpr float MAX_ACCUMULATED_COST = 99999999.f;

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

std::string print_result(const StateContainer& container,
                         const std::vector<StateId>& original_state_ids) {
  std::string result = R"({"type":"FeatureCollection","features":[)";
  std::string fsep;
  for (auto s : original_state_ids) {
    result += fsep + container.geojson(s);
    fsep = ",";
  }
  result += R"(]})";
  return result;
}

struct Interpolation {
  midgard::PointLL projected;
  baldr::GraphId edgeid;
  double sq_distance;
  double route_distance;
  double route_time;
  double edge_distance;
  std::vector<EdgeSegment>::const_iterator segment;

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

inline MatchResult CreateMatchResult(const Measurement& measurement, StateId state_id = {}) {
  return {measurement.lnglat(),
          0.f,
          baldr::GraphId{},
          -1.f,
          measurement.epoch_time(),
          state_id,
          measurement.is_break_point()};
}

inline MatchResult CreateMatchResult(const Measurement& measurement, const Interpolation& interp) {
  return {interp.projected,
          std::sqrt(interp.sq_distance),
          interp.edgeid,
          interp.edge_distance,
          measurement.epoch_time(),
          StateId(),
          measurement.is_break_point()};
}

// Find the interpolation along the route where the transition cost +
// emission cost is minimal
Interpolation InterpolateMeasurement(const MapMatcher& mapmatcher,
                                     const Measurement& measurement,
                                     std::vector<EdgeSegment>::const_iterator begin,
                                     float begin_source_offset,
                                     std::vector<EdgeSegment>::const_iterator end,
                                     float match_measurement_distance,
                                     float match_measurement_time,
                                     const midgard::PointLL& left_most_projected_point,
                                     const midgard::PointLL& right_most_projected_point) {
  graph_tile_ptr tile;
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

    const auto edgeinfo = tile->edgeinfo(directededge);

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

    // clip distance along and projection to the match boundaries
    if (segment == begin && offset < begin_source_offset) {
      offset = begin_source_offset;
      sq_distance = projector.approx.DistanceSquared(left_most_projected_point);
      projected_point = left_most_projected_point;
    }
    if (segment == end - 1 && offset > segment->target) {
      offset = segment->target;
      sq_distance = projector.approx.DistanceSquared(right_most_projected_point);
      projected_point = right_most_projected_point;
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

    Interpolation interp{projected_point, segment->edgeid, sq_distance, route_distance,
                         route_time,      offset,          segment};

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
                                                 const StateId& next_stateid,
                                                 const MatchResult& first_result,
                                                 const MatchResult& last_result) {
  // Nothing to do here
  if (measurements.empty()) {
    return {};
  }

  // can't interpolate these because they don't happen between two valid states or
  // we weren't able to get a downstream route
  std::vector<MatchResult> results;
  results.reserve(measurements.size());
  if (!stateid.IsValid() || !next_stateid.IsValid()) {
    for (const auto& measurement : measurements) {
      results.push_back(CreateMatchResult(measurement));
    }
    return results;
  }

  // get the path between the two uninterpolated states
  std::vector<EdgeSegment> route;
  MergeRoute(mapmatcher.state_container().state(stateid),
             mapmatcher.state_container().state(next_stateid), route, last_result);

  // for each point that needs interpolated
  std::vector<EdgeSegment>::const_iterator left_most_segment = route.begin();
  float left_most_offset = route.empty() ? 0.f : route.begin()->source;
  midgard::PointLL left_most_projection = first_result.lnglat;

  for (const auto& measurement : measurements) {
    // we need some info about the measurement to figure out what the best interpolation would be
    const auto& match_measurement = mapmatcher.state_container().measurement(stateid.time());
    const auto match_measurement_distance = GreatCircleDistance(measurement, match_measurement);
    const auto match_measurement_time = ClockDistance(measurement, match_measurement);

    // interpolate this point along the route
    const auto interp =
        InterpolateMeasurement(mapmatcher, measurement, left_most_segment, left_most_offset,
                               route.end(), match_measurement_distance, match_measurement_time,
                               left_most_projection, last_result.lnglat);

    // shift the point at which we are allowed to start interpolating from to the right
    // so that its on the interpolation point this way the next interpolation happens after
    if (interp.edgeid.Is_Valid()) {
      left_most_segment = interp.segment;
      left_most_offset = interp.edge_distance;
      left_most_projection = interp.projected;
    }

    // if it was able to do the interpolation
    if (interp.edgeid.Is_Valid()) {
      // keep the interpolated match result
      results.push_back(CreateMatchResult(measurement, interp));
    } // couldnt interpolate this point
    else {
      results.push_back(CreateMatchResult(measurement));
    }
  }

  return results;
}

// Find the match result of a state, given its previous state and next state
MatchResult FindMatchResult(const MapMatcher& mapmatcher,
                            const std::vector<StateId>& stateids,
                            StateId::Time time,
                            baldr::GraphReader& graph_reader) {
  // Either the time is invalid because of discontinuity or it matches the index
  const auto& state_id = stateids[time];
  assert(!state_id.IsValid() || state_id.time() == time);

  // If we have a discontinuity on either side of this point
  const auto& measurement = mapmatcher.state_container().measurement(time);
  if (!state_id.IsValid()) {
    return CreateMatchResult(measurement, state_id);
  }

  bool begins_discontinuity = false;
  bool ends_discontinuity = false;

  // Find the last edge of the path from the previous state to the current state
  baldr::GraphId prev_edge;
  baldr::GraphId node_id;
  // Because of node routing in meili we must loop back over the previous path. In most cases the loop
  // is a single iteration but in rare cases (node to node trivial routes) we need to explore states
  // that are older than the immediate previous state
  for (StateId::Time t = time; t > 0 && !prev_edge.Is_Valid(); --t) {
    // If there is no path from t - 1
    const auto& prev_state_id = stateids[t - 1];
    if (!prev_state_id.IsValid()) {
      // Mark the discontinuity if this is the current time and the previous state was not routable
      if (t == time) {
        ends_discontinuity = true;
      }
      // Give up on finding a previous edge
      break;
    }
    const auto& prev_state = mapmatcher.state_container().state(prev_state_id);
    const auto& state_id = stateids[t];
    const auto& state = mapmatcher.state_container().state(state_id);
    // Normally the last label of the route from previous to current state has a valid edge id. But it
    // wont if the destination (current state candidate) was a node. In this case it will have a valid
    // node id, but match results need edges so we keep looking backwards until we find an edge
    auto rbegin = prev_state.RouteBegin(state), rend = prev_state.RouteEnd();
    if (rbegin == rend)
      break;
    node_id = rbegin->nodeid();
    while (rbegin != rend && !prev_edge.Is_Valid()) {
      const auto& label = *rbegin;
      prev_edge = label.edgeid();
      rbegin++;
    }
  }

  // Find the first edge of the path from the current state to the next state
  baldr::GraphId next_edge;
  // Because of node routing in meili we must loop over the next sets of paths. In most cases the loop
  // is a single iteration but in rare cases (node to node trivial routes) we need to explore states
  // that are newer than the immediate next state
  for (StateId::Time t = time; t + 1 < stateids.size() && !next_edge.Is_Valid(); ++t) {
    // If there is no path to t + 1
    const auto& next_state_id = stateids[t + 1];
    if (!next_state_id.IsValid()) {
      // Mark the discontinuity if this is the current time and the next state was not routable
      if (t == time) {
        begins_discontinuity = true;
      }
      // Give up on finding a next edge
      break;
    }
    const auto& next_state = mapmatcher.state_container().state(next_state_id);
    const auto& state_id = stateids[t];
    const auto& state = mapmatcher.state_container().state(state_id);
    // Normally we need to loop back to the first label of the route from current to the next state
    // and get its edge id. But its possible the origin (current state candidate) was a node which
    // means the first label will only be a nodeid.  If we didnt find any edges after that (which is
    // the case in a trivial node to node route) then we need to keep looking at more states into the
    // future or until we hit a discontinuity
    auto rbegin = state.RouteBegin(next_state), rend = state.RouteEnd();
    if (rbegin == rend)
      break;
    while (rbegin != rend) {
      const auto& label = *rbegin;
      if (label.nodeid().Is_Valid())
        node_id = label.nodeid();
      if (label.edgeid().Is_Valid())
        next_edge = label.edgeid();
      rbegin++;
    }
  }

  // if we got here without finding any edges one of two things could have happened. the first is that
  // we had some candidates on this state but there were no paths to/from these states. the second is
  // that there was a path but it was one where you route from a node in the graph to itself. in that
  // case we dont really care what edge we use to go from the node to itself so just use any one that
  // was a candidate on it
  const auto& state = mapmatcher.state_container().state(state_id);
  if (!prev_edge.Is_Valid() && !next_edge.Is_Valid()) {
    // it wasnt a trivial node route
    if (!node_id.Is_Valid()) {
      return CreateMatchResult(measurement, state_id);
    }
    assert(!state.candidate().edges.empty());
    const auto& candidate = state.candidate().edges.front();
    return {candidate.projected,          std::sqrt(candidate.distance), candidate.id,
            candidate.percent_along,      measurement.epoch_time(),      state_id,
            measurement.is_break_point(), begins_discontinuity,          ends_discontinuity};
  }

  // find which candidate was used for this state
  graph_tile_ptr tile;
  for (const auto& edge : state.candidate().edges) {
    // if it matches either end of the path coming into this state or the beginning of the
    // path leaving this state, then we are good to go and have found the match
    if (edge.id == prev_edge || edge.id == next_edge) {
      return {edge.projected,
              std::sqrt(edge.distance),
              edge.id,
              edge.percent_along,
              measurement.epoch_time(),
              state_id,
              measurement.is_break_point(),
              begins_discontinuity,
              ends_discontinuity};
    }

    // the only matches we can make where the ids arent the same are at intersections
    if (edge.percent_along > 0.f && edge.percent_along < 1.f) {
      continue;
    }

    // we are at an intersection so we need the node from the candidate where the route would be
    auto candidate_nodes = graph_reader.GetDirectedEdgeNodes(edge.id, tile);
    const auto& candidate_node =
        edge.percent_along == 0.f ? candidate_nodes.first : candidate_nodes.second;

    // if the last edge of the previous route ends at this candidate node
    const auto* prev_de = graph_reader.directededge(prev_edge, tile);
    if (prev_de && prev_de->endnode() == candidate_node) {
      return {edge.projected,
              std::sqrt(edge.distance),
              prev_edge,
              1.f,
              measurement.epoch_time(),
              state_id,
              measurement.is_break_point(),
              begins_discontinuity,
              ends_discontinuity};
    }

    // if the first edge of the next route starts at this candidate node
    const auto* next_opp_de = graph_reader.GetOpposingEdge(next_edge, tile);
    if (next_opp_de && next_opp_de->endnode() == candidate_node) {
      return {edge.projected,
              std::sqrt(edge.distance),
              next_edge,
              0.f,
              measurement.epoch_time(),
              state_id,
              measurement.is_break_point(),
              begins_discontinuity,
              ends_discontinuity};
    }

    // when the intersection match fails, we do a more labor intensive search at transitions
    // for both prev_edge's ending node and next_edge's starting node and see if we could do a
    // node snap match. We collect the info together and search both prev edge end node and next
    // edge start node in the below loop
    std::array<std::tuple<const baldr::DirectedEdge*, baldr::GraphId, float>, 2> transition_infos{
        {std::tuple<const baldr::DirectedEdge*, baldr::GraphId, float>{prev_de, prev_edge, 1.f},
         std::tuple<const baldr::DirectedEdge*, baldr::GraphId, float>{next_opp_de, next_edge, 0.f}}};

    for (const auto& trans_info : transition_infos) {
      const auto* target_de = std::get<0>(trans_info);
      auto target_edge_id = std::get<1>(trans_info);
      float distance_along = std::get<2>(trans_info);

      if (target_de && edge.id.level() != target_de->endnode().level()) {
        baldr::GraphId end_node = target_de->endnode();
        for (const auto& trans : tile->GetNodeTransitions(end_node)) {
          // we only care about if the nodes are in the same level
          if (trans.endnode().level() != candidate_node.level()) {
            continue;
          }

          // bail when nodes are the same level node is not a sibling of the target node
          end_node = trans.endnode();
          tile = graph_reader.GetGraphTile(end_node);
          if (tile == nullptr || end_node != candidate_node) {
            break;
          }

          return {edge.projected,
                  std::sqrt(edge.distance),
                  target_edge_id,
                  distance_along,
                  measurement.epoch_time(),
                  state_id,
                  measurement.is_break_point(),
                  begins_discontinuity,
                  ends_discontinuity};
        }
      }
    }
  }

  // we should never reach here, the above early exit checks whether there is no path on either side
  // of the given state we are finding a match for. perhaps we should throw?
  return CreateMatchResult(measurement, state_id);
}

// Find the corresponding match results of a list of states
std::vector<MatchResult> FindMatchResults(const MapMatcher& mapmatcher,
                                          const std::vector<StateId>& stateids,
                                          baldr::GraphReader& graph_reader) {
  std::vector<MatchResult> results;
  for (StateId::Time time = 0; time < stateids.size(); time++) {
    results.push_back(FindMatchResult(mapmatcher, stateids, time, graph_reader));
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

MapMatcher::MapMatcher(const Config& config,
                       baldr::GraphReader& graphreader,
                       CandidateQuery& candidatequery,
                       const sif::mode_costing_t& mode_costing,
                       sif::TravelMode travelmode)
    : config_(config), graphreader_(graphreader), candidatequery_(candidatequery),
      mode_costing_(mode_costing), travelmode_(travelmode), interrupt_(nullptr), vs_(), ts_(vs_),
      container_(), emission_cost_model_(graphreader_, container_, config_.emission_cost),
      transition_cost_model_(graphreader_,
                             vs_,
                             ts_,
                             container_,
                             mode_costing_,
                             travelmode_,
                             config_.transition_cost) {
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

void MapMatcher::RemoveRedundancies(const std::vector<StateId>& result,
                                    const std::vector<MatchResult>& results) {
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
          MergeRoute(left_used_candidate, right_candidate, edges, results[time + 1])) {
        paths_from_winner.emplace(right_candidate.stateid(), std::move(edges));
      }
    }

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
      for (const auto& right_candidate : container_.column(time + 1)) {
        // If there is no route its not really unique since we dont need discontinuities
        std::vector<EdgeSegment> edges;
        if (ts_.IsRemoved(right_candidate.stateid()) ||
            !MergeRoute(left_unused_candidate, right_candidate, edges, results[time + 1])) {
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
  best_paths.reserve(k);

  // Nothing to do
  if (measurements.empty()) {
    best_paths.emplace_back(std::vector<MatchResult>{}, std::vector<EdgeSegment>{}, 0);
    return best_paths;
  }

  bool found_discontinuity = false;

  // Separate the measurements we are using for matching from the ones we'll just interpolate
  auto interpolated = AppendMeasurements(measurements);
  // Without minimum number of edge candidates, throw a 443 - NoSegment error code.
  if (!container_.HasMinimumCandidates()) {
    throw valhalla_exception_t{443};
  }

  // For k paths
  std::vector<StateId> state_ids;
  state_ids.reserve(container_.size());
  while (best_paths.size() < k && !found_discontinuity) {
    // Get the states for the kth best path in reversed order then fix the order
    state_ids.clear();
    double accumulated_cost = 0.f;
    while (state_ids.size() < container_.size()) {
      // Get the time at the last column of states
      const auto time = container_.size() - state_ids.size() - 1;
      // Find the most probable path
      std::copy(vs_.SearchPathVS(time, false), vs_.PathEnd(), std::back_inserter(state_ids));
      // See what the last state was that we reached
      const auto& winner = vs_.SearchWinner(time);
      // If we got all the way to the end there were no discontinuities and the cost is a normal value
      if (winner.IsValid()) {
        accumulated_cost += vs_.AccumulatedCost(winner);
      } // We got a discontinuity before reaching the last state
      else {
        // TODO need a sane constant cost for invalid state
        accumulated_cost += MAX_ACCUMULATED_COST;
        found_discontinuity = true;
      }

      // if we need to match more we add a penalty for connecting over the discontinuity
      if (state_ids.size() < container_.size()) {
        found_discontinuity = true;
        accumulated_cost += MAX_ACCUMULATED_COST;
      }
    }

    // we dont return additional paths (alternatives) if the have discontinuities
    if (!best_paths.empty() && found_discontinuity) {
      break;
    }

    // Get back the real state ids in order
    std::vector<StateId> original_state_ids;
    original_state_ids.reserve(state_ids.size());
    for (auto s_itr = state_ids.rbegin(); s_itr != state_ids.rend(); ++s_itr) {
      original_state_ids.push_back(ts_.GetOrigin(*s_itr, *s_itr));
    }

    // Get the match result for each of the states
    auto results = FindMatchResults(*this, original_state_ids, graphreader_);

    // Insert the interpolated results into the result list
    std::vector<MatchResult> best_path;
    best_path.reserve(measurements.size());
    for (StateId::Time time = 0; time < original_state_ids.size(); time++) {
      // Add in this states result
      best_path.emplace_back(results[time]);

      // See if there were any interpolated points with this state move on if not
      const auto it = interpolated.find(time);
      if (it == interpolated.end()) {
        continue;
      }

      // Interpolate the points between this and the next state
      const auto& this_stateid = original_state_ids[time];
      const auto& next_stateid =
          time + 1 < original_state_ids.size() ? original_state_ids[time + 1] : StateId();

      const auto& first_result = results[time];
      const auto& last_result = results[time + 1];
      const auto interpolated_results =
          InterpolateMeasurements(*this, it->second, this_stateid, next_stateid, first_result,
                                  last_result);

      // Copy the interpolated match results into the final set
      best_path.insert(best_path.cend(), interpolated_results.cbegin(), interpolated_results.cend());
    }

    // Construct a result
    auto segments = ConstructRoute(*this, best_path);
    MatchResults match_results(std::move(best_path), std::move(segments), accumulated_cost);

    // We'll keep it if we don't have a duplicate already
    auto found_path = std::find(best_paths.rbegin(), best_paths.rend(), match_results);
    if (found_path == best_paths.rend()) {
      LOG_TRACE(static_cast<std::stringstream&&>(std::stringstream() << match_results).str());
      LOG_TRACE("Result " + std::to_string(best_paths.size()));
      LOG_TRACE(print_result(container_, original_state_ids));
      best_paths.emplace_back(std::move(match_results));
    }

    // RemoveRedundancies doesn't work with broken paths yet,
    // also we want to avoid removing path for the last best path
    if (!found_discontinuity && best_paths.size() < k) {
      // Remove all the candidates pairs whose paths are redundant with this one
      RemoveRedundancies(original_state_ids, results);
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
  const float sq_max_search_radius = config_.candidate_search.max_search_radius_meters *
                                     config_.candidate_search.max_search_radius_meters;
  const float sq_interpolation_distance =
      config_.routing.interpolation_distance_meters * config_.routing.interpolation_distance_meters;
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
      candidatequery_.Query(measurement.lnglat(), measurement.stop_type(), sq_radius, costing());

  const auto time = container_.AppendMeasurement(measurement);

  for (const auto& candidate : candidates) {
    const auto& stateid = container_.AppendCandidate(candidate);
    vs_.AddStateId(stateid);
  }

  return time;
}

} // namespace meili
} // namespace valhalla
