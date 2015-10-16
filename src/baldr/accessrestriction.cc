#include <string.h>
#include "baldr/accessrestriction.h"

namespace valhalla {
namespace baldr {

// Constructor with arguments
AccessRestriction::AccessRestriction(const uint32_t edgeid,
                                     const AccessType type,
                                     const uint32_t value)
: edgeid_(edgeid),
  type_(type),
  value_(value) {
}

// Get the internal edge Id.
uint32_t AccessRestriction::edgeid() const {
  return edgeid_;
}

// Get the type
AccessType AccessRestriction::type() const {
  return type_;
}

// Get the value
uint32_t AccessRestriction::value() const {
  return value_;
}

// operator < - for sorting. Sort by route Id.
bool AccessRestriction::operator < (const AccessRestriction& other) const {
  return edgeid() < other.edgeid();
}

}
}
