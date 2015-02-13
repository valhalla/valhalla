#include "mjolnir/turnrestrictionbuilder.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Constructor with arguments
TurnRestrictionBuilder::TurnRestrictionBuilder(const uint32_t idx,
                                 const RestrictionType type,
                                 const uint32_t mask) {
  data_.edgeindex = idx;
  data_.type = static_cast<uint32_t>(type);
  restriction_mask_ = mask;
}

// Set the directed edge index.
void TurnRestrictionBuilder::set_edgeindex(const uint32_t idx) {
  data_.edgeindex = idx;
}

}
}
