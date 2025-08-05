#ifndef VALHALLA_MJOLNIR_OSMACCESSRESTRICTION_H
#define VALHALLA_MJOLNIR_OSMACCESSRESTRICTION_H

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>

#include <cstdint>

namespace valhalla {
namespace mjolnir {

// Used for access restrictions. Conveys the direction in which the access restriction applies
enum class AccessRestrictionDirection : uint8_t { kBoth = 0, kForward = 1, kBackward = 2 };

/**
 * OSM Access restriction information. Access Restrictions are stored in a
 * multimap keyed by the Id of the "from" way of the restriction.
 */
class OSMAccessRestriction {
public:
  /**
   * Constructor
   */
  OSMAccessRestriction() : except_destination_(0){};

  /**
   * Destructor.
   */
  ~OSMAccessRestriction() = default;

  /**
   * Set the restriction type
   */
  void set_type(baldr::AccessType type);

  /**
   * Get the restriction type
   */
  baldr::AccessType type() const;

  /**
   * Set the value for the restriction
   */
  void set_value(uint64_t value);

  /**
   * Get the value
   */
  uint64_t value() const;

  /**
   * Get the modes for the restriction
   * @return  Returns a bit field of affected modes.
   */
  uint16_t modes() const;

  /**
   * Set the modes for the restriction
   */
  void set_modes(uint16_t modes);

  /**
   * Get the direction the access restriction applies to.
   */
  AccessRestrictionDirection direction() const;

  /**
   * Set the direction the access restriction applies to.
   */
  void set_direction(AccessRestrictionDirection direction);

  /**
   * Whether or not the restriction applies to local traffic
   */
  bool except_destination() const;

  /**
   * Set flag for whether or not the restriction applies to local traffic
   */
  void set_except_destination(const bool except_destination);

protected:
  struct Attributes {
    uint16_t type_ : 4;
    uint16_t modes_ : 12;
  };

  uint64_t value_ = 0;

  Attributes attributes_ = {0, 0};
  uint16_t except_destination_ : 1;
  uint16_t spare_ : 15;
  AccessRestrictionDirection direction_ = AccessRestrictionDirection::kBoth;
  uint8_t spare1_ = 0;
  uint16_t spare2_ = 0;
};
static_assert(sizeof(OSMAccessRestriction) == 16);

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMACCESSRESTRICTION_H
