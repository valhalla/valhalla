#include "mjolnir/osmaccessrestriction.h"

#include <cstring>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

OSMAccessRestriction::OSMAccessRestriction() {
  memset(this, 0, sizeof(OSMAccessRestriction));
}

OSMAccessRestriction::~OSMAccessRestriction() {
}

// Set the restriction type
void OSMAccessRestriction::set_type(AccessType type) {
  attributes_.type_ = static_cast<uint16_t>(type);
}

// Get the restriction type
AccessType OSMAccessRestriction::type() const {
  return static_cast<AccessType>(attributes_.type_);
}

// Set the value
void OSMAccessRestriction::set_value(uint64_t value) {
  value_ = value;
}

// Get the value
uint64_t OSMAccessRestriction::value() const {
  return value_;
}

// Set the modes for the restriction
void OSMAccessRestriction::set_modes(uint16_t modes) {
  attributes_.modes_ = modes;
}

// Get the modes for the restriction
uint16_t OSMAccessRestriction::modes() const {
  return attributes_.modes_;
}

AccessRestrictionDirection OSMAccessRestriction::direction() const {
  return static_cast<AccessRestrictionDirection>(direction_);
}

void OSMAccessRestriction::set_direction(AccessRestrictionDirection direction) {
  direction_ = direction;
};

} // namespace mjolnir
} // namespace valhalla
