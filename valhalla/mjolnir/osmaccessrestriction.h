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

 protected:

  struct Attributes {
    uint32_t type_        : 4;
    uint32_t value_       : 14;
    uint32_t spare_       : 14;
  };
  Attributes attributes_;
};

}
}

#endif  // VALHALLA_MJOLNIR_OSMACCESSRESTRICTION_H
