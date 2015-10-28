#include <string.h>
#include "baldr/accessrestriction.h"

namespace valhalla {
namespace baldr {

// Constructor with arguments
AccessRestriction::AccessRestriction(const uint32_t edgeid,
                                     const AccessType type,
                                     const uint32_t modes,
                                     const uint32_t dow,
                                     const uint64_t value)
  : edgeid_(edgeid),
    type_(static_cast<uint32_t>(type)),
    modes_(modes),
    days_of_week_(dow),
    value_(value) {
}

// Get the internal edge Id.
uint32_t AccessRestriction::edgeid() const {
  return edgeid_;
}

// Get the type
AccessType AccessRestriction::type() const {
  return static_cast<AccessType>(type_);
}

// Get the modes impacted by access restriction.
uint32_t AccessRestriction::modes() const {
  return modes_;
}

// Gets the days of the week for this access restriction.
uint32_t AccessRestriction::days_of_week() const {
  return days_of_week_;
}

// Get the value
uint64_t AccessRestriction::value() const {
  return value_;
}

// operator < - for sorting. Sort by route Id.
bool AccessRestriction::operator < (const AccessRestriction& other) const {
  return edgeid() < other.edgeid();
}

}
}
