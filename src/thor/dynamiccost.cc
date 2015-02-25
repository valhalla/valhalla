#include "thor/dynamiccost.h"

using namespace valhalla::baldr;

namespace valhalla{
namespace thor{

DynamicCost::DynamicCost()
    : not_thru_distance_(5000.0f) {
}

DynamicCost::~DynamicCost() {
}

// Does the costing allow hierarchy transitions? Defaults to false. Costing
// methods that wish to use hierarchy transitions must override this method.
bool DynamicCost::AllowTransitions() const {
  return false;
}

// Set the distance from the destination where "not_thru" edges are allowed.
void DynamicCost::set_not_thru_distance(const float d) {
  not_thru_distance_ = d;
}

}
}
