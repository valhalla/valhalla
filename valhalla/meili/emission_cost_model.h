#ifndef MMP_EMISSION_COST_MODEL_H_
#define MMP_EMISSION_COST_MODEL_H_

#include <functional>
#include <valhalla/meili/map_matching.h>

namespace valhalla {
namespace meili {

class EmissionCostModel
{
 private:
  using StateGetter = std::function<const State&(const StateId& stateid)>;

 public:
  EmissionCostModel(
      baldr::GraphReader& graphreader,
      const StateGetter& get_state,
      float sigma_z)
      : graphreader_(graphreader),
        get_state_(get_state),
        sigma_z_(sigma_z)
  {
    if (sigma_z_ <= 0.f) {
      throw std::invalid_argument("Expect sigma_z to be positive");
    }
  }

  EmissionCostModel(
      baldr::GraphReader& graphreader,
      const StateGetter& get_state,
      const boost::property_tree::ptree& config)
      : EmissionCostModel(graphreader, get_state, config.get<float>("sigma_z")) {}

  // given the *squared* great circle distance between a measurement and its candidate,
  // return the emission cost of the candidate
  float
  CalculateEmissionCost(float sq_distance) const
  { return sq_distance * inv_double_sq_sigma_z_; }

  float operator()(const StateId& stateid) const
  { return CalculateEmissionCost(get_state_(stateid).candidate().edges.front().score); }

 private:
  baldr::GraphReader& graphreader_;

  StateGetter get_state_;

  float sigma_z_;
  double inv_double_sq_sigma_z_;  // equals to 1.f / (sigma_z_ * sigma_z_ * 2.f)
};
} // namespace meili
}

#endif /* MMP_EMISSION_COST_MODEL_H_ */
