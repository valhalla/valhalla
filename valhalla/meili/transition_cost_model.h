#ifndef MMP_TRANSITION_COST_MODEL_H_
#define MMP_TRANSITION_COST_MODEL_H_

#include <functional>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/meili/config.h>
#include <valhalla/meili/measurement.h>
#include <valhalla/meili/state.h>
#include <valhalla/meili/topk_search.h>
#include <valhalla/meili/viterbi_search.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace meili {

class TransitionCostModel {
public:
  TransitionCostModel(baldr::GraphReader& graphreader,
                      const IViterbiSearch& vs,
                      const TopKSearch& ts,
                      const StateContainer& container,
                      const sif::mode_costing_t& mode_costing,
                      const sif::TravelMode travelmode,
                      float beta,
                      float breakage_distance,
                      float max_route_distance_factor,
                      float max_route_time_factor,
                      float turn_penalty_factor);

  TransitionCostModel(baldr::GraphReader& graphreader,
                      const IViterbiSearch& vs,
                      const TopKSearch& ts,
                      const StateContainer& container,
                      const sif::mode_costing_t& mode_costing,
                      const sif::TravelMode travelmode,
                      const Config::TransitionCost& config);

  // we use the difference between the original two measurements and the distance along the route
  // network to compute a transition cost of a given candidate, transition_time may be added if
  // the turn_penalty_table_ is enabled, one could make use of time in this computation but
  // this is not advisable as traffic at the time may make readings unreliable and time information
  // is not strictly required to perform the matching
  float CalculateTransitionCost(float turn_cost,
                                float route_distance,
                                float measurement_distance,
                                float,
                                float) const {
    return (turn_cost + std::abs(route_distance - measurement_distance)) * inv_beta_;
  }

  float operator()(const StateId& lhs, const StateId& rhs) const;

private:
  void UpdateRoute(const StateId& lhs, const StateId& rhs) const;

  float ClockDistance(const StateId::Time& lhs, const StateId::Time& rhs) const {
    double clk_dist = -1.0;

    const auto lhs_leave_time = container_.leave_time(lhs);
    const auto rhs_epoch = container_.measurement(rhs).epoch_time();
    if (0 <= lhs_leave_time && 0 <= rhs_epoch) {
      clk_dist = rhs_epoch - lhs_leave_time;
    }

    return clk_dist;
  }

  baldr::GraphReader& graphreader_;

  const IViterbiSearch& vs_;

  const TopKSearch& ts_;

  const StateContainer& container_;

  const sif::mode_costing_t& mode_costing_;

  const sif::TravelMode travelmode_;

  float beta_;
  float inv_beta_; // equals to 1.f / beta_

  float breakage_distance_;

  float max_route_distance_factor_;

  float max_route_time_factor_;

  float turn_penalty_factor_;

  // Cost for each degree in [0, 180]
  float turn_cost_table_[181];

  bool match_on_restrictions_{false};
};

} // namespace meili
} // namespace valhalla
#endif /* MMP_TRANSITION_COST_MODEL_H_ */
