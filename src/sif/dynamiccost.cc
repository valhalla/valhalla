#include "sif/dynamiccost.h"
#include <valhalla/thor/hierarchylimits.h>

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

// Does the costing method allow multiple passes (with relaxed hierarchy
// limits). Defaults to false. Costing methods that wish to allow multiple
// passes with relaxed hierarchy transitions must override this method.
bool DynamicCost::AllowMultiPass() const {
  return false;
}

// Returns the cost to make the transition from the predecessor edge.
// Defaults to 0. Costing models that wish to include edge transition
// costs (i.e., intersection/turn costs) must override this method.
Cost DynamicCost::TransitionCost(const DirectedEdge* edge,
                                 const NodeInfo* node,
                                 const EdgeLabel& pred,
                                 const uint32_t to_idx) const {
  return Cost(0.0f, 0.0f);
}

// Get the general unit size that can be considered as equal for sorting
// purposes. Defaults to 1 (second).
uint32_t DynamicCost::UnitSize() const {
  return kDefaultUnitSize;
}

// Gets the hierarchy limits.
std::vector<HierarchyLimits>& DynamicCost::GetHierarchyLimits() {
  return hierarchy_limits_;
}

// Relax hierarchy limits.
void DynamicCost::RelaxHierarchyLimits(const float factor) {
  for (auto& hierarchy : hierarchy_limits_) {
    hierarchy.Relax(factor);
  }
}

// Do not transition up to highway level - remain on arterial. Used as last
// resort.
void DynamicCost::DisableHighwayTransitions() {
  hierarchy_limits_[1].DisableHighwayTransitions();
}

// Set the distance from the destination where "not_thru" edges are allowed.
void DynamicCost::set_not_thru_distance(const float d) {
  not_thru_distance_ = d;
}

}
}
