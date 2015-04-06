#ifndef VALHALLA_BALDR_ADMIN_H_
#define VALHALLA_BALDR_ADMIN_H_

#include <stdint.h>

#include <valhalla/midgard/util.h>


namespace valhalla {
namespace baldr {

constexpr size_t kDst = 8;
constexpr size_t kStateIso = 3;
constexpr size_t kCountryIso = 2;

/**
 * Holds a generic admin with state and country iso and text. Also, contains
 * start and end dst.  Text is stored in the GraphTile text list and the offset
 * is stored within the admin.  This is a read only base class.
 */
class Admin {
 public:

  // country ISO3166-1
  const std::string country_iso() const;

  // country ISO + dash + state ISO will give you ISO3166-2 for state.
  const std::string state_iso() const;

  // When does daylight saving time start?
  const std::string start_dst() const;

  // When does daylight saving time end?
  const std::string end_dst() const;

  /**
   * Get the offset into the GraphTile text list for the state text associated
   * with the admin.
   * @return  Returns the text offset.
   */
  uint32_t state_offset() const;

  /**
   * Get the offset into the GraphTile text list for the country text associated
   * with the admin.
   * @return  Returns the text offset.
   */
  uint32_t country_offset() const;

 protected:
  // Constructor
  Admin(const uint32_t country_offset, const uint32_t state_offset,
        const std::string& country_iso, const std::string& state_iso,
        const std::string& start_dst, const std::string& end_dst);

  // country ISO3166-1
  char country_iso_[kCountryIso];

  // state ISO3166-2
  char state_iso_[kStateIso];

  // DST start date and time.
  char start_dst_[kDst];

  // DST end date and time.
  char end_dst_[kDst];

  // country name offset
  uint32_t country_offset_;

  // state name offset
  uint32_t state_offset_;

};

}
}

#endif  // VALHALLA_BALDR_ADMIN_H_
