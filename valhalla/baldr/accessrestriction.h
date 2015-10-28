#ifndef VALHALLA_BALDR_ACCESSRESTRICTION_H_
#define VALHALLA_BALDR_ACCESSRESTRICTION_H_

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Information held for each access restriction.
 */
class AccessRestriction {
 public:
  // Constructor with arguments
  AccessRestriction(const uint32_t edgeid,
                    const AccessType type,
                    const uint32_t modes,
                    const uint32_t dow,
                    const uint64_t value);

  /**
   * Get the internal edge Id.
   * @return  Returns the internal edge Id.
   */
  uint32_t edgeid() const;

  /**
   * Get the type of the restriction.  See graphconstants.h
   * @return  Returns the type of the restriction
   */
  AccessType type() const;

  /**
   * Get the modes impacted by access restriction.
   * @return  Returns a bit field of affected modes.
   */
  uint32_t modes() const;

  /**
   * Gets the days of the week for this access restriction.
   * @return  Returns the days of the week bit mask.
   */
  uint32_t days_of_week() const;

  /**
   * Get the value for this restriction.
   * @return  Returns the value
   */
  uint64_t value() const;

  /**
   * operator < - for sorting. Sort by edge Id.
   * @param  other  Other access restriction to compare to.
   * @return  Returns true if edgeid < other edgeid.
   */
  bool operator < (const AccessRestriction& other) const;

 protected:

  uint64_t edgeid_      : 22;  // Directed edge Id.
  uint64_t type_        : 6;   // Access type
  uint64_t modes_       : 12;  // Mode(s) this access restriction applies to
  uint64_t days_of_week_ : 7;  // Days of week this access restriction applies

  uint64_t value_;        // Value for this restriction. Can take on
                          // different meanings per type
};

}
}

#endif  // VALHALLA_BALDR_ACCESSRESTRICTION_H_
