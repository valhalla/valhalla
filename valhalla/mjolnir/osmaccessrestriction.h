#ifndef VALHALLA_MJOLNIR_OSMACCESSRESTRICTION_H
#define VALHALLA_MJOLNIR_OSMACCESSRESTRICTION_H

#include <cstdint>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace mjolnir {

/**
 * OSM Access restriction information. Access Restrictions are stored in a
 * multimap keyed by the Id of the "from" way of the restriction.
 */
class OSMAccessRestriction {
 public:
  /**
   * Constructor
   */
  OSMAccessRestriction();

  /**
   * Destructor.
   */
  ~OSMAccessRestriction();

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
  void set_value(uint32_t value);

  /**
   * Get the value
   */
  uint32_t value() const;

  /**
   * Get the modes for the restriction
   * @return  Returns a bit field of affected modes.
   */
  uint32_t modes() const;

  /**
   * Set the modes for the restriction
   */
  void set_modes(uint32_t modes);

 protected:

  struct Attributes {
    uint32_t type_        : 4;
    uint32_t value_       : 14;
    uint64_t modes_       : 12;
    uint32_t spare_       : 2;
  };
  Attributes attributes_;
};

}
}

#endif  // VALHALLA_MJOLNIR_OSMACCESSRESTRICTION_H
