// -*- mode: c++ -*-

#include <algorithm>

#include <valhalla/midgard/pointll.h>

#include "viterbi_search.h"
#include "edge_search.h"
#include "sp.h"

using namespace valhalla;


class Measurement {
 public:
  Measurement(const PointLL& lnglat)
      : lnglat_(lnglat) {}

  const PointLL& lnglat() const
  { return lnglat_; };

 private:
  PointLL lnglat_;
};


class State
{
 public:
  State(const StateId id,
        const Time time,
        const Candidate& candidate)
      : id_(id),
        time_(time),
        candidate_(candidate),
        labelset_(nullptr),
        label_idx_() {}

  State(const Time time, const Candidate& candidate)
      : id_(kInvalidStateId),
        time_(time),
        candidate_(candidate),
        labelset_(nullptr),
        label_idx_() {}

  const StateId id() const
  { return id_; }

  const Time time() const
  { return time_; }

  const Candidate& candidate() const
  { return candidate_; }

  bool interpolated() const
  { return id_ == kInvalidStateId; }

  bool routed() const
  { return labelset_ != nullptr; }

  void route(const std::vector<const State*>& states,
             GraphReader& graphreader,
             float max_route_distance) const
  {
    // TODO disable routing to interpolated states

    // Prepare locations
    std::vector<PathLocation> locations;
    locations.reserve(1 + states.size());
    locations.push_back(candidate_.pathlocation());
    for (const auto state : states) {
      locations.push_back(state->candidate().pathlocation());
    }

    // Route
    labelset_ = std::make_shared<LabelSet>(std::ceil(max_route_distance));
    // TODO pass labelset_ as shared_ptr
    const auto& results = find_shortest_path(graphreader, locations, 0, *labelset_);

    // Cache results
    label_idx_.clear();
    uint16_t dest = 1;  // dest at 0 is remained for the origin
    for (const auto state : states) {
      const auto it = results.find(dest);
      if (it != results.end()) {
        label_idx_[state->id()] = it->second;
      }
      dest++;
    }
  }

  float route_distance(const State& state) const
  {
    const auto it = label_idx_.find(state.id());
    if (it != label_idx_.end()) {
      return labelset_->label(it->second).cost;
    }
    return -1.f;
  }

  RoutePathIterator RouteBegin(const State& state) const
  {
    const auto it = label_idx_.find(state.id());
    if (it != label_idx_.end()) {
      return RoutePathIterator(labelset_.get(), it->second);
    }
    return RoutePathIterator(labelset_.get());
  }

  // TODO remove it
  RoutePathIterator RouteEnd(const State& state) const
  { return RoutePathIterator(labelset_.get()); }

  RoutePathIterator RouteEnd() const
  { return RoutePathIterator(labelset_.get()); }

 private:
  const StateId id_;

  const Time time_;

  const Candidate candidate_;

  mutable std::shared_ptr<LabelSet> labelset_;

  mutable std::unordered_map<StateId, uint32_t> label_idx_;
};


inline float GreatCircleDistance(const State& left,
                                 const State& right)
{
  const auto &left_pt = left.candidate().pathlocation().vertex(),
            &right_pt = right.candidate().pathlocation().vertex();
  return left_pt.Distance(right_pt);
}


inline float GreatCircleDistanceSquared(const State& left,
                                        const State& right)
{
  const auto &left_pt = left.candidate().pathlocation().vertex(),
            &right_pt = right.candidate().pathlocation().vertex();
  return left_pt.DistanceSquared(right_pt);
}


inline float GreatCircleDistance(const Measurement& left,
                                 const Measurement& right)
{ return left.lnglat().Distance(right.lnglat()); }


inline float GreatCircleDistanceSquared(const Measurement& left,
                                        const Measurement& right)
{ return left.lnglat().DistanceSquared(right.lnglat()); }


constexpr float kBreakageDistance = 2000.f;  // meters
constexpr float kProximateDistance = 10.f;   // meters
constexpr float kSquaredProximateDistance = kProximateDistance * kProximateDistance; // meters^2


class MapMatching: public ViterbiSearch<State>
{
 public:
  MapMatching(float sigma_z,
              float beta,
              baldr::GraphReader& graphreader,
              const std::shared_ptr<sif::DynamicCost>* mode_costing,
              const sif::TravelMode mode)
      : sigma_z_(sigma_z),
        inv_double_sq_sigma_z_(1.f / (sigma_z_ * sigma_z_ * 2.f)),
        beta_(beta),
        inv_beta_(1.f / beta_),
        measurements_(),
        graphreader_(graphreader),
        mode_costing_(mode_costing),
        mode_(mode),
        states_()
  {
    if (sigma_z_ <= 0.f) {
      throw std::invalid_argument("expect sigma_z to be positive");
    }

    if (beta_ <= 0.f) {
      throw std::invalid_argument("expect beta to be positive");
    }
  }

  ~MapMatching()
  { Clear(); }

  void Clear()
  {
    measurements_.clear();
    states_.clear();
    ViterbiSearch<State>::Clear();
  }

  template <typename candidate_iterator_t>
  Time AppendState(const Measurement& measurement,
                   candidate_iterator_t begin,
                   candidate_iterator_t end)
  {
    Time time = states_.size();

    // Append to base class
    std::vector<const State*> column;
    for (candidate_iterator_t it = begin; it != end; it++) {
      StateId id = state_.size();
      state_.push_back(new State(id, time, *it));
      column.push_back(state_.back());
    }
    unreached_states_.push_back(column);

    states_.push_back(column);
    measurements_.push_back(measurement);

    return time;
  }

  const std::vector<const State*>&
  states(Time time) const
  { return states_[time]; }

  const std::shared_ptr<sif::DynamicCost> costing() const
  { return mode_costing_[static_cast<uint32_t>(mode_)]; }

  baldr::GraphReader& graphreader() const
  { return graphreader_; }

  const Measurement& measurement(Time time) const
  { return measurements_[time]; }

  const Measurement& measurement(const State& state) const
  { return measurements_[state.time()]; }

  std::vector<Measurement>::size_type size() const
  { return measurements_.size(); }

 private:
  float sigma_z_;
  double inv_double_sq_sigma_z_;  // equals to 1.f / (sigma_z_ * sigma_z_ * 2.f)
  float beta_;
  float inv_beta_;  // equals to 1.f / beta_
  std::vector<Measurement> measurements_;
  baldr::GraphReader& graphreader_;
  const std::shared_ptr<DynamicCost>* mode_costing_;
  const TravelMode mode_;
  std::vector<std::vector<const State*>> states_;

 protected:
  virtual float MaxRouteDistance(const State& left, const State& right) const
  {
    auto mmt_distance = GreatCircleDistance(measurement(left), measurement(right));
    return std::min(mmt_distance * 3, kBreakageDistance);
  }

  float TransitionCost(const State& left, const State& right) const override
  {
    auto mmt_distance = GreatCircleDistance(measurement(left), measurement(right));
    if (!left.routed()) {
      left.route(unreached_states_[right.time()], graphreader_, MaxRouteDistance(left, right));
    }
    assert(left.routed());

    auto route_distance = left.route_distance(right);
    if (route_distance >= 0.f) {
      return std::abs(route_distance - mmt_distance) * inv_beta_;
    }

    return -1.f;
  }

  inline float EmissionCost(const State& state) const override
  { return state.candidate().sq_distance() * inv_double_sq_sigma_z_; }

  inline double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override
  { return prev_costsofar + transition_cost + emission_cost; }
};


std::vector<Candidate>::const_iterator
interpolate_along_route(const std::unordered_map<GraphId, std::vector<Candidate>::const_iterator>& candidate_map,
                        const std::vector<Candidate>::const_iterator candidate_end,
                        const RoutePathIterator route_begin,
                        const RoutePathIterator route_end)
{
  auto closest_candidate = candidate_end;
  float closest_sq_distance = std::numeric_limits<float>::infinity();

  for (auto label = route_begin; label != route_end; label++) {
    auto it = candidate_map.find(label->edgeid);
    if (it != candidate_map.end()) {
      auto candidate = it->second;
      auto sq_distance = candidate->sq_distance();
      if (sq_distance < closest_sq_distance) {
        closest_candidate = candidate;
        closest_sq_distance = sq_distance;
      }
    }

    it = candidate_map.find(label->nodeid);
    if (it != candidate_map.end()) {
      auto candidate = it->second;
      auto sq_distance = candidate->sq_distance();
      if (sq_distance < closest_sq_distance) {
        closest_candidate = candidate;
        closest_sq_distance = sq_distance;
      }
    }
  }

  return closest_candidate;
}


// Collect a nodeid set of a path location
std::unordered_set<GraphId>
collect_nodes(GraphReader& reader, const PathLocation& location)
{
  std::unordered_set<GraphId> results;

  for (const auto& edge : location.edges()) {
    if (!edge.id.Is_Valid()) continue;
    if (edge.dist == 0.f) {
      const auto opp_edge = reader.GetOpposingEdge(edge.id);
      if (opp_edge) {
        results.insert(opp_edge->endnode());
      }
    } else if (edge.dist == 1.f) {
      const auto tile = reader.GetGraphTile(edge.id);
      if (tile) {
        const auto directededge = tile->directededge(edge.id);
        if (directededge) {
          results.insert(directededge->endnode());
        }
      }
    }
  }

  return results;
}


const State*
interpolate(MapMatching& mm,
            const CandidateQuery& cq,
            float max_sq_search_radius,
            const MapMatching::iterator target_state,
            const Measurement& measurement)
{
  auto source_state = std::next(target_state);
  if (!source_state.IsValid()) {
    // TODO return the closest one
    return nullptr;
  }

  const auto& candidates = cq.Query(measurement.lnglat(),
                                    max_sq_search_radius,
                                    mm.costing()->GetFilter());
  if (candidates.empty()) {
    return nullptr;
  }

  // Reverse map from GraphId to its canidate
  std::unordered_map<GraphId, std::vector<Candidate>::const_iterator> candidate_map;
  for (auto it = candidates.begin(); it != candidates.end(); it++) {
    const auto& location = it->pathlocation();
    if (!location.IsNode()) {
      for (const auto& edge : it->pathlocation().edges()) {
        if (edge.id.Is_Valid()) {
          candidate_map[edge.id] = it;
        }
      }
    } else {
      for (const auto nodeid : collect_nodes(mm.graphreader(), location)) {
        if (nodeid.Is_Valid()) {
          candidate_map[nodeid] = it;
        }
      }
    }
  }
  if (candidate_map.empty()) {
    return nullptr;
  }

  auto closest_candidate = candidates.cend();
  if (target_state.IsValid()) {
    // Find the closest candidate along the route path from the source
    // state to the target state
    closest_candidate = interpolate_along_route(
        candidate_map, candidates.cend(),
        source_state->RouteBegin(*target_state),
        source_state->RouteEnd(*target_state));
  } else {
    // If target state is not found, we simply interpolate it on the
    // same edge where the source state's candidate stays
    const auto& location = source_state->candidate().pathlocation();
    for (const auto& edge : location.edges()) {
      const auto it = candidate_map.find(edge.id);
      if (it != candidate_map.end()) {
        closest_candidate = it->second;
        break;
      }
    }
    // TODO check nodes as well
  }

  if (closest_candidate != candidates.end()) {
    return new State(target_state.time(), *closest_candidate);
  } else {
    // TODO return the closest candidate and mark it as new start
    return nullptr;
  }
}


std::vector<const State*>
OfflineMatch(MapMatching& mm,
             const CandidateQuery& cq,
             const std::vector<Measurement>& measurements,
             float max_sq_search_radius)
{
  mm.Clear();

  if (measurements.empty()) {
    return {};
  }

  using mmt_size_t = std::vector<Measurement>::size_type;
  Time time = 0;
  std::unordered_map<Time, std::vector<mmt_size_t>> proximate_measurements;

  // Load states
  for (mmt_size_t idx = 0,
             last_idx = 0,
              end_idx = measurements.size() - 1;
       idx <= end_idx; idx++) {
    const auto& measurement = measurements[idx];
    auto sq_distance = GreatCircleDistanceSquared(measurements[last_idx], measurement);
    // Always match the first and the last measurement
    if (kSquaredProximateDistance <= sq_distance || idx == 0 || idx == end_idx) {
      const auto& candidates = cq.Query(measurement.lnglat(),
                                        max_sq_search_radius,
                                        mm.costing()->GetFilter());
      time = mm.AppendState(measurement, candidates.begin(), candidates.end());
      last_idx = idx;
    } else {
      proximate_measurements[time].push_back(idx);
    }
  }

  // Search viterbi path
  std::vector<MapMatching::iterator> path;
  for (auto state = mm.SearchPath(time); state != mm.PathEnd(); state++) {
    path.push_back(state);
  }
  std::reverse(path.begin(), path.end());
  assert(path.size() == mm.size());

  // Interpolate proximate measurements and merge their states into
  // the results
  std::vector<const State*> results;
  for (Time time = 0; time < mm.size(); time++) {
    const auto& state = path[time];
    results.push_back(state.IsValid()? &(*state) : nullptr);
    auto it = proximate_measurements.find(time);
    if (it != proximate_measurements.end()) {
      assert (time + 1 < mm.size());  // Won't be the last time
      for (const auto idx : it->second) {
        results.push_back(interpolate(mm, cq, max_sq_search_radius, path[time + 1], measurements[idx]));
      }
    }
  }
  assert(results.size() == measurements.size());

  return results;
}


// Interpolated states won't be deleted in the destructor of
// ViterbiSearch, so you have to manually call this helper function to
// delete them

// TODO it's a workaround maybe we should use shared_ptr
void DeleteInterpolatedStates(const std::vector<const State*> states)
{
  for (const auto state : states) {
    if (state && state->interpolated()) {
      delete state;
    }
  }
}
