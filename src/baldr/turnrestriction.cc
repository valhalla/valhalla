#include "../../valhalla/baldr/turnrestriction.h"

namespace valhalla {
namespace baldr {

// Constructor
TurnRestriction::TurnRestriction()
    : data_{},
      restriction_mask_(0) {
}

// Get the index of the "from" directed edge for this turn restriction.
uint32_t TurnRestriction::edgeindex() const {
  return data_.edgeindex;
}

// Get the turn restriction type.
RestrictionType TurnRestriction::type() const {
  return static_cast<RestrictionType>(data_.type);
}

// Get the restriction mask. This is a bit mask where bits set to 1 indicate
// a restricted turn based on index of the outbound directed edge at the
// end node of this directed edge.
uint32_t TurnRestriction::restriction_mask() const {
  return restriction_mask_;
}

}
}
