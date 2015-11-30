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
  { return lnglat_; }

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

  const StateId id() const
  { return id_; }

  const Time time() const
  { return time_; }

  const Candidate& candidate() const
  { return candidate_; }

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

  float EmissionCost(const State& state) const override
  { return state.candidate().sq_distance() * inv_double_sq_sigma_z_; }

  double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override
  { return prev_costsofar + transition_cost + emission_cost; }
};


enum class GraphType: uint8_t
{ kUnknown = 0, kEdge, kNode };


class MatchResult
{
 public:
  MatchResult(const Point& lnglat,
              float distance,
              const GraphId graphid,
              GraphType graphtype,
              const State* state = nullptr)
      : lnglat_(lnglat),
        distance_(distance),
        graphid_(graphid),
        graphtype_(graphtype),
        state_(state) {}

  MatchResult(const Point& lnglat)
      : lnglat_(lnglat),
        distance_(0.f),
        graphid_(),
        graphtype_(GraphType::kUnknown),
        state_(nullptr)
  { assert(!graphid_.Is_Valid()); }

  // Coordinate of the matched point
  const PointLL& lnglat() const
  { return lnglat_; }

  // Distance from measurement to the matched point
  float distance() const
  { return distance_; }

  // Which edge/node this matched point stays
  const GraphId graphid() const
  { return graphid_; }

  GraphType graphtype() const
  { return graphtype_; }

  // Attach the state pointer for other information (e.g. reconstruct
  // the route path) and debugging
  const State* state() const
  { return state_; }

 private:
  PointLL lnglat_;
  float distance_;
  GraphId graphid_;
  GraphType graphtype_;
  const State* state_;
};


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


MatchResult
guess_source_result(const MapMatching::iterator source,
                    const MapMatching::iterator target,
                    const Measurement& source_measurement)
{
  if (source.IsValid() && target.IsValid()) {
    GraphId last_valid_id;
    GraphType last_valid_type = GraphType::kUnknown;
    for (auto label = source->RouteBegin(*target);
         label != source->RouteEnd(); label++) {
      if (label->nodeid.Is_Valid()) {
        last_valid_id = label->nodeid;
        last_valid_type = GraphType::kNode;
      } else if (label->edgeid.Is_Valid()) {
        last_valid_id = label->edgeid;
        last_valid_type = GraphType::kEdge;
      }
    }
    const auto& c = source->candidate();
    return {c.pathlocation().vertex(), c.distance(), last_valid_id, last_valid_type, &(*source)};
  } else if (source.IsValid()) {
    return {source_measurement.lnglat(), 0.f, GraphId(), GraphType::kUnknown, &(*source)};
  }

  return {source_measurement.lnglat()};
}


MatchResult
guess_target_result(const MapMatching::iterator source,
                    const MapMatching::iterator target,
                    const Measurement& target_measurement)
{
  if (source.IsValid() && target.IsValid()) {
    auto label = source->RouteBegin(*target);
    GraphId graphid;
    GraphType graphtype = GraphType::kUnknown;
    if (label != source->RouteEnd()) {
      if (label->nodeid.Is_Valid()) {
        graphid = label->nodeid;
        graphtype = GraphType::kNode;
      } else if (label->edgeid.Is_Valid()) {
        graphid = label->edgeid;
        graphtype = GraphType::kEdge;
      }
    }
    const auto& c = target->candidate();
    return {c.pathlocation().vertex(), c.distance(), graphid, graphtype, &(*target)};
  } else if (target.IsValid()) {
    return {target_measurement.lnglat(), 0.f, GraphId(), GraphType::kUnknown, &(*target)};
  }

  return {target_measurement.lnglat()};
}


MatchResult
interpolate(GraphReader& reader,
            const std::unordered_set<GraphId>& graphset,
            const std::vector<Candidate>::const_iterator begin,
            const std::vector<Candidate>::const_iterator end,
            const Measurement& measurement)
{
  auto closest_candidate = end;
  float closest_sq_distance = std::numeric_limits<float>::infinity();
  GraphId closest_graphid;
  GraphType closest_graphtype = GraphType::kUnknown;

  for (auto candidate = begin; candidate != end; candidate++) {
    if (candidate->sq_distance() < closest_sq_distance) {
      const auto& location = candidate->pathlocation();

      if (!location.IsNode()) {
        for (const auto& edge : location.edges()) {
          const auto it = graphset.find(edge.id);
          if (it != graphset.end()) {
            closest_candidate = candidate;
            closest_sq_distance = candidate->sq_distance();
            closest_graphid = edge.id;
            closest_graphtype = GraphType::kEdge;
          }
        }
      } else {
        for (const auto nodeid : collect_nodes(reader, location)) {
          const auto it = graphset.find(nodeid);
          if (it != graphset.end()) {
            closest_candidate = candidate;
            closest_sq_distance = candidate->sq_distance();
            closest_graphid = nodeid;
            closest_graphtype = GraphType::kNode;
          }
        }
      }
    }
  }

  if (closest_candidate != end) {
    return {closest_candidate->pathlocation().vertex(), closest_candidate->distance(), closest_graphid, closest_graphtype};
  }

  return {measurement.lnglat()};
}


std::unordered_set<GraphId>
collect_graphset(GraphReader& reader,
                 const MapMatching::iterator source,
                 const MapMatching::iterator target)
{
  std::unordered_set<GraphId> graphset;
  if (source.IsValid() && target.IsValid()) {
    for (auto label = source->RouteBegin(*target);
         label != source->RouteEnd();
         ++label) {
      if (label->edgeid.Is_Valid()) {
        graphset.insert(label->edgeid);
      }
      if (label->nodeid.Is_Valid()) {
        graphset.insert(label->nodeid);
      }
    }
  } else if (source.IsValid()) {
    const auto& location = source->candidate().pathlocation();
    if (!location.IsNode()) {
      for (const auto& edge : location.edges()) {
        if (edge.id.Is_Valid()) {
          graphset.insert(edge.id);
        }
      }
    } else {
      for (const auto nodeid : collect_nodes(reader, location)) {
        if (nodeid.Is_Valid()) {
          graphset.insert(nodeid);
        }
      }
    }
  }

  return graphset;
}


std::vector<MatchResult>
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
  std::vector<MapMatching::iterator> iterpath;
  iterpath.reserve(mm.size());
  for (auto it = mm.SearchPath(time); it != mm.PathEnd(); it++) {
    iterpath.push_back(it);
  }
  std::reverse(iterpath.begin(), iterpath.end());
  assert(iterpath.size() == mm.size());

  // Interpolate proximate measurements and merge their states into
  // the results
  std::vector<MatchResult> results;
  results.reserve(measurements.size());
  results.emplace_back(measurements.front().lnglat());
  assert(!results.back().graphid().Is_Valid());

  for (Time time = 1; time < mm.size(); time++) {
    const auto &source_state = iterpath[time - 1],
               &target_state = iterpath[time];

    if (!results.back().graphid().Is_Valid()) {
      results.pop_back();
      const auto& measurement = measurements[results.size()];
      results.push_back(guess_source_result(source_state, target_state, measurement));
    }

    auto it = proximate_measurements.find(time - 1);
    if (it != proximate_measurements.end()) {
      const auto& graphset = collect_graphset(mm.graphreader(), source_state, target_state);
      for (const auto idx : it->second) {
        const auto& candidates = cq.Query(measurements[idx].lnglat(),
                                          max_sq_search_radius,
                                          mm.costing()->GetFilter());
        results.push_back(interpolate(mm.graphreader(), graphset,
                                      candidates.begin(), candidates.end(),
                                      measurements[idx]));
      }
    }

    const auto& measurement = measurements[results.size()];
    results.push_back(guess_target_result(source_state, target_state, measurement));
  }
  assert(results.size() == measurements.size());

  return results;
}


struct EdgeSegment
{
  EdgeSegment(GraphId the_edgeid,
              float the_source,
              float the_target)
      : edgeid(the_edgeid),
        source(the_source),
        target(the_target) {}

  GraphId edgeid;
  float source;
  float target;
};


void MergeRoute(std::vector<EdgeSegment>& route,
                const std::vector<EdgeSegment>& segments)
{
  for (auto segment = segments.rbegin(); segment != segments.rend(); segment++) {
    if (segment->edgeid.Is_Valid()) {
      if (!route.empty()) {
        auto& last_segment = route.back();
        if (last_segment.edgeid == segment->edgeid) {
          // TODO assert(segment->target >= last_segment.target)
          last_segment.target = std::max(last_segment.target, segment->target);
        } else {
          route.push_back(*segment);
        }
      } else {
        route.push_back(*segment);
      }
    }
  }
}


template <typename iterator_t>
std::vector<EdgeSegment>
ConstructRoute(iterator_t begin, iterator_t end)
{
  std::vector<EdgeSegment> route;
  iterator_t previous_match = end;

  for (auto match = begin; match != end; match++) {
    if (!match->state()) {
      continue;
    }
    if (previous_match != end) {
      std::vector<EdgeSegment> segments;
      auto previous_state = previous_match->state(),
            current_state = match->state();
      for (auto label = previous_state->RouteBegin(*current_state),
                  end = previous_state->RouteEnd(); label != end; label++) {
        if (label->edgeid.Is_Valid()) {
          assert(label->source <= label->target);
          segments.emplace_back(label->edgeid, label->source, label->target);
        }
      }
      MergeRoute(route, segments);
    }
    previous_match = match;
  }

  return route;
}
