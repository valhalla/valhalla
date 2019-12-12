#include "sif/hierarchylimits.h"

using namespace valhalla::sif;

bool HierarchyLimits::StopExpanding(const float dist) const {
  return (dist > expansion_within_dist && up_transition_count > max_up_transitions);
}
