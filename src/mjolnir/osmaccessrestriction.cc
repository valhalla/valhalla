#include "mjolnir/osmaccessrestriction.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

OSMAccessRestriction::OSMAccessRestriction()
    : attributes_{} {
}

OSMAccessRestriction::~OSMAccessRestriction() {
}

// Set the restriction type
void OSMAccessRestriction::set_type(AccessType type) {
  attributes_.type_ = static_cast<uint32_t>(type);
}

// Get the restriction type
AccessType OSMAccessRestriction::type() const {
  return static_cast<AccessType>(attributes_.type_);
}

// Set the hour off
void OSMAccessRestriction::set_value(uint32_t value) {
  attributes_.value_ = value;
}

// Get the hour off
uint32_t OSMAccessRestriction::value() const {
  return attributes_.value_;
}

}
}
