#ifndef MMP_TRANSITION_COST_MODEL_H_
#define MMP_TRANSITION_COST_MODEL_H_

#include <functional>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/meili/map_matching.h>
#include <valhalla/meili/measurement.h>
#include <valhalla/meili/viterbi_search.h>

namespace valhalla {
namespace meili {

class TransitionCostModel
{
 private:
  using ColumnGetter = std::function<const std::vector<State>&(const StateId::Time&)>;
  using MeasurementGetter = std::function<const Measurement&(const StateId::Time&)>;

 public:
  TransitionCostModel(
      baldr::GraphReader& graphreader,
      const IViterbiSearch& vs,
      const ColumnGetter& get_column,
      const MeasurementGetter& get_measurement,
      const sif::cost_ptr_t* mode_costing,
      const sif::TravelMode mode,
      float beta,
      float breakage_distance,
      float max_route_distance_factor,
      float max_route_time_factor,
      float turn_penalty_factor);

  TransitionCostModel(
      baldr::GraphReader& graphreader,
      const IViterbiSearch& vs,
      const ColumnGetter& get_column,
      const MeasurementGetter& get_measurement,
      const sif::cost_ptr_t* mode_costing,
      const sif::TravelMode mode,
      const boost::property_tree::ptree& config);

  float
  CalculateTransitionCost(float turn_cost, float route_distance, float measurement_distance,
                          float route_time, float measurement_time) const
  { return (turn_cost + std::abs(route_distance - measurement_distance)) * inv_beta_; }

  float operator()(const StateId& lhs, const StateId& rhs) const;

 private:
  void UpdateRoute(const StateId& lhs, const StateId& rhs) const;

  baldr::GraphReader& graphreader_;

  const IViterbiSearch& vs_;

  ColumnGetter get_column_;

  MeasurementGetter get_measurement_;

  const sif::cost_ptr_t* mode_costing_;

  const sif::TravelMode mode_;

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
#endif /* MMP_TRANSITION_COST_MODEL_H_ */
