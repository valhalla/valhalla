#ifndef MMP_EMISSION_COST_MODEL_H_
#define MMP_EMISSION_COST_MODEL_H_

#include <functional>
#include <valhalla/meili/state.h>

namespace valhalla {
namespace meili {

class EmissionCostModel {
private:
  using StateGetter = std::function<const State&(const StateId& stateid)>;

public:
  EmissionCostModel(baldr::GraphReader& graphreader, const StateContainer& container, float sigma_z)
      : graphreader_(graphreader), container_(container), sigma_z_(sigma_z),
        inv_double_sq_sigma_z_(1.f / (sigma_z_ * sigma_z_ * 2.f)) {
    if (sigma_z_ <= 0.f) {
      throw std::invalid_argument("Expect sigma_z to be positive");
    }
  }

  EmissionCostModel(baldr::GraphReader& graphreader,
                    const StateContainer& container,
                    const boost::property_tree::ptree& config)
      : EmissionCostModel(graphreader, container, config.get<float>("sigma_z")) {
  }

  // given the *squared* great circle distance between a measurement and its candidate,
  // return the emission cost of the candidate
  float CalculateEmissionCost(float sq_distance) const {
    return sq_distance * inv_double_sq_sigma_z_;
  }

  float operator()(const StateId& stateid) const {
    return CalculateEmissionCost(container_.state(stateid).candidate().edges.front().distance);
  }

private:
  baldr::GraphReader& graphreader_;

  const StateContainer& container_;

  float sigma_z_;
  double inv_double_sq_sigma_z_; // equals to 1.f / (sigma_z_ * sigma_z_ * 2.f)
};
} // namespace meili
} // namespace valhalla

#endif /* MMP_EMISSION_COST_MODEL_H_ */
