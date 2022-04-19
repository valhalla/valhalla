#include "sif/hierarchylimits.h"

using namespace valhalla::sif;

bool HierarchyLimits::StopExpanding(const float dist) const {
  return (up_transition_count > max_up_transitions && dist > expansion_within_dist);
}
