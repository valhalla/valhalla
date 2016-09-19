// -*- mode: c++ -*-
#ifndef MMP_MAP_MATCHING_H_
#define MMP_MAP_MATCHING_H_
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/baldr/pathlocation.h>

#include <valhalla/meili/measurement.h>
#include <valhalla/meili/viterbi_search.h>
#include <valhalla/meili/routing.h>


namespace valhalla{
namespace meili {


class State
{
 public:
  State(const StateId id, const Time time, const baldr::PathLocation& candidate);

  StateId id() const
  { return id_; }

  Time time() const
  { return time_; }

  const baldr::PathLocation& candidate() const
  { return candidate_; }

  bool routed() const
  { return labelset_ != nullptr; }

  void route(const std::vector<const State*>& states,
             baldr::GraphReader& graphreader,
             float max_route_distance,
             const midgard::DistanceApproximator& approximator,
             float search_radius,
             sif::cost_ptr_t costing,
             std::shared_ptr<const sif::EdgeLabel> edgelabel,
             const float turn_cost_table[181]) const;

  const Label* last_label(const State& state) const;

  RoutePathIterator RouteBegin(const State& state) const
  {
    const auto it = label_idx_.find(state.id());
    if (it != label_idx_.end()) {
      return RoutePathIterator(labelset_.get(), it->second);
    }
    return RoutePathIterator(labelset_.get());
  }

  RoutePathIterator RouteEnd() const
  { return RoutePathIterator(labelset_.get()); }

 private:
  const StateId id_;

  const Time time_;

  const baldr::PathLocation candidate_;

  mutable std::shared_ptr<LabelSet> labelset_;

  mutable std::unordered_map<StateId, uint32_t> label_idx_;
};


class MapMatching: public ViterbiSearch<State>
{
 public:
  MapMatching(baldr::GraphReader& graphreader,
              const sif::cost_ptr_t* mode_costing,
              const sif::TravelMode mode,
              float sigma_z,
              float beta,
              float breakage_distance,
              float max_route_distance_factor,
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

  const std::vector<const State*>&
  states(Time time) const
  { return states_[time]; }

  const Measurement& measurement(Time time) const
  { return measurements_[time]; }

  const Measurement& measurement(const State& state) const
  { return measurements_[state.time()]; }

  std::vector<Measurement>::size_type size() const
  { return measurements_.size(); }

  template <typename candidate_iterator_t>
  Time AppendState(const Measurement& measurement,
                   candidate_iterator_t begin,
                   candidate_iterator_t end)
  {
    Time time = states_.size();

    // Append to base class
    std::vector<const State*> column;
    for (auto it = begin; it != end; it++) {
      StateId id = state_.size();
      state_.push_back(new State(id, time, *it));
      column.push_back(state_.back());
    }
    unreached_states_.push_back(column);

    states_.push_back(column);
    measurements_.push_back(measurement);

    return time;
  }

  float
  CalculateEmissionCost(float sq_distance) const
  { return sq_distance * inv_double_sq_sigma_z_; }

  float
  CalculateTransitionCost(float turncost, float route_distance, float measurement_distance) const
  { return (turncost + std::abs(route_distance - measurement_distance)) * inv_beta_; }

 protected:
  virtual float MaxRouteDistance(const State& left, const State& right) const;

  float TransitionCost(const State& left, const State& right) const override;

  float EmissionCost(const State& state) const override;

  double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override;

 private:

  baldr::GraphReader& graphreader_;

  const sif::cost_ptr_t* mode_costing_;

  const sif::TravelMode mode_;

  std::vector<Measurement> measurements_;

  std::vector<std::vector<const State*>> states_;

  float sigma_z_;
  double inv_double_sq_sigma_z_;  // equals to 1.f / (sigma_z_ * sigma_z_ * 2.f)

  float beta_;
  float inv_beta_;  // equals to 1.f / beta_

  float breakage_distance_;

  float max_route_distance_factor_;

  float turn_penalty_factor_;

  // Cost for each degree in [0, 180]
  float turn_cost_table_[181];
};

}
}
#endif // MMP_MAP_MATCHING_H_
