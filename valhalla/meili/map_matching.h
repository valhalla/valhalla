// -*- mode: c++ -*-
#ifndef MMP_MAP_MATCHING_H_
#define MMP_MAP_MATCHING_H_

#include <cmath>
#include <cstdint>

#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/costconstants.h>

#include <valhalla/baldr/pathlocation.h>

#include <valhalla/meili/measurement.h>
#include <valhalla/meili/stateid.h>
#include <valhalla/meili/viterbi_search.h>
#include <valhalla/meili/routing.h>


namespace valhalla{
namespace meili {


class State
{
 public:
  State(const StateId& stateid, const baldr::PathLocation& candidate)
      : stateid_(stateid),
        candidate_(candidate),
        labelset_(nullptr),
        label_idx_() {}

  const StateId& stateid() const
  { return stateid_; }

  const baldr::PathLocation& candidate() const
  { return candidate_; }

  bool routed() const
  { return labelset_ != nullptr; }

  void route(const std::vector<State>& states,
             baldr::GraphReader& graphreader,
             float max_route_distance,
             float max_route_time,
             const midgard::DistanceApproximator& approximator,
             const float search_radius,
             sif::cost_ptr_t costing,
             const Label* edgelabel,
             const float turn_cost_table[181]) const;

  const Label* last_label(const State& state) const;

  RoutePathIterator RouteBegin(const State& state) const
  {
    const auto it = label_idx_.find(state.stateid());
    if (it != label_idx_.end()) {
      return RoutePathIterator(labelset_.get(), it->second);
    }
    return RoutePathIterator(labelset_.get());
  }

  RoutePathIterator RouteEnd() const
  { return RoutePathIterator(labelset_.get()); }

 private:
  StateId stateid_;

  const baldr::PathLocation candidate_;

  mutable std::shared_ptr<LabelSet> labelset_;

  mutable std::unordered_map<StateId, uint32_t> label_idx_;
};

class MapMatching: public ViterbiSearch
{
 private:
  using Column = std::vector<State>;

 public:
  MapMatching(baldr::GraphReader& graphreader,
              const sif::cost_ptr_t* mode_costing,
              const sif::TravelMode mode,
              float sigma_z,
              float beta,
              float breakage_distance,
              float max_route_distance_factor,
              float max_route_time_factor,
              float turn_penalty_factor);

  MapMatching(baldr::GraphReader& graphreader,
              const sif::cost_ptr_t* mode_costing,
              const sif::TravelMode mode,
              const boost::property_tree::ptree& config);

  virtual ~MapMatching();

  void Clear();

  baldr::GraphReader& graphreader() const
  { return graphreader_; }

  sif::cost_ptr_t costing() const
  { return mode_costing_[static_cast<size_t>(mode_)]; }

  const Column&
  states(StateId::Time time) const
  { return columns_[time]; }

  const State&
  state(const StateId& stateid) const
  { return columns_[stateid.time()][stateid.id()]; }

  const Measurement& measurement(StateId::Time time) const
  { return measurements_[time]; }

  const Measurement& measurement(const State& state) const
  { return measurements_[state.stateid().time()]; }

  std::vector<Measurement>::size_type size() const
  { return measurements_.size(); }

  void SetMeasurementLeaveTime(StateId::Time time, double leave_time)
  { measurements_[time].SetLeaveTime(leave_time); }

  template <typename candidate_iterator_t>
  StateId::Time AppendState(const Measurement& measurement,
                            candidate_iterator_t begin,
                            candidate_iterator_t end)
  {
    measurements_.push_back(measurement);

    StateId::Time time = columns_.size();
    Column column;
    uint32_t idx = 0;
    for (auto it = begin; it != end; it++, idx++) {
      const StateId stateid(time, idx);
      AddStateId(stateid);
      column.emplace_back(stateid, *it);
    }
    columns_.push_back(column);

    return time;
  }

  // given the *squared* great circle distance between a measurement and its candidate,
  // return the emission cost of the candidate
  float
  CalculateEmissionCost(float sq_distance) const
  { return sq_distance * inv_double_sq_sigma_z_; }

  // we use the difference between the original two measurements and the distance along the route
  // network to compute a transition cost of a given candidate, turn_cost may be added if
  // the turn_penalty_table_ is enabled, one could make use of time in this computation but
  // this is not advisable as traffic at the time may make readings unreliable and time information
  // is not strictly required to perform the matching
  float
  CalculateTransitionCost(float turn_cost, float route_distance, float measurement_distance,
      float route_time, float measurement_time) const
  { return (turn_cost + std::abs(route_distance - measurement_distance)) * inv_beta_; }

 protected:
  float TransitionCost(const StateId& lhs, const StateId& rhs) const override;

  float EmissionCost(const StateId& stateid) const override;

  double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override;

 private:
  baldr::GraphReader& graphreader_;

  const sif::cost_ptr_t* mode_costing_;

  const sif::TravelMode mode_;

  std::vector<Measurement> measurements_;

  std::vector<Column> columns_;

  float sigma_z_;
  double inv_double_sq_sigma_z_;  // equals to 1.f / (sigma_z_ * sigma_z_ * 2.f)

  float beta_;
  float inv_beta_;  // equals to 1.f / beta_

  float breakage_distance_;

  float max_route_distance_factor_;

  float max_route_time_factor_;

  float turn_penalty_factor_;

  // Cost for each degree in [0, 180]
  float turn_cost_table_[181];
};

}
}
#endif // MMP_MAP_MATCHING_H_
