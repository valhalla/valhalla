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
                    const uint32_t value);

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
   * Get the value for this restriction.
   * @return  Returns the value
   */
  uint32_t value() const;

  /**
   * operator < - for sorting. Sort by edge Id.
   * @param  other  Other access restriction to compare to.
   * @return  Returns true if edgeid < other edgeid.
   */
  bool operator < (const AccessRestriction& other) const;

 protected:
  // Internal edge Id. Used to lookup access restrictions
  uint32_t edgeid_;

  // type of access restrictions.
  AccessType type_;

  // value for this restriction.
  uint32_t value_;
};

}
}

#endif  // VALHALLA_BALDR_ACCESSRESTRICTION_H_
