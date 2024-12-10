#include "sif/hierarchylimits.h"

using namespace valhalla::sif;

namespace valhalla {
namespace sif {

bool StopExpanding(const valhalla::HierarchyLimits& hierarchy_limits, const float dist) {
  if (hierarchy_limits.max_up_transitions() > 0) {
    auto mup = hierarchy_limits.max_up_transitions();
    auto s = hierarchy_limits.expansion_within_dist();
  }
  return (hierarchy_limits.up_transition_count() > hierarchy_limits.max_up_transitions() &&
          dist > hierarchy_limits.expansion_within_dist());
}
bool StopExpanding(const valhalla::HierarchyLimits& hierarchy_limits) {
  return hierarchy_limits.up_transition_count() > hierarchy_limits.max_up_transitions();
}
void RelaxHierarchyLimits(valhalla::HierarchyLimits& hierarchy_limits,
                          const float factor,
                          const float expansion_within_factor) {

  if (hierarchy_limits.max_up_transitions() != kUnlimitedTransitions) {
    hierarchy_limits.set_max_up_transitions(hierarchy_limits.max_up_transitions() * factor);
    hierarchy_limits.set_expansion_within_dist(hierarchy_limits.expansion_within_dist() *
                                               expansion_within_factor);
  }
}
} // namespace sif
} // namespace valhalla
