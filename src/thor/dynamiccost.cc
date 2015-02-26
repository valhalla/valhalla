#include "thor/dynamiccost.h"

using namespace valhalla::baldr;

namespace valhalla{
namespace thor{

DynamicCost::DynamicCost(const boost::property_tree::ptree& pt)
    : not_thru_distance_(5000.0f) {
  // Parse property tree to get hierarchy limits
  // TODO - get the number of levels
  for (uint32_t level = 0; level <= 8; level++) {
    hierarchy_limits_.emplace_back(HierarchyLimits(pt, level));
  }
}

DynamicCost::~DynamicCost() {
}

// Does the costing allow hierarchy transitions? Defaults to false. Costing
// methods that wish to use hierarchy transitions must override this method.
bool DynamicCost::AllowTransitions() const {
  return false;
}

// Gets the hierarchy limits.
std::vector<HierarchyLimits>& DynamicCost::GetHierarchyLimits() {
  return hierarchy_limits_;
}

// Set the distance from the destination where "not_thru" edges are allowed.
void DynamicCost::set_not_thru_distance(const float d) {
  not_thru_distance_ = d;
}

}
}
