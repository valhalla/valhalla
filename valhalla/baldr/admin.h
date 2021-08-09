#ifndef VALHALLA_BALDR_ADMIN_H_
#define VALHALLA_BALDR_ADMIN_H_

#include <cstdint>
#include <stdint.h>

#include <valhalla/midgard/util.h>

namespace valhalla {
namespace baldr {

constexpr size_t kStateIso = 3;
constexpr size_t kCountryIso = 2;

// Returns the 3-char equivalent of the 2-char country code (iso_3166_1_alpha2) or an empty string
// if the 2-char code is unknown
std::string get_iso_3166_1_alpha3(const std::string& iso_3166_1_alpha2);

/**
 * Holds a generic admin with state and country iso and text. Text is stored
 * in the GraphTile text list and the offset is stored within the admin.
 */
class Admin {
public:
  /**
   * Constructor.
   * @param  country_offset  Offset to country name in text records.
   * @param  state_offset    Offset to state name in text records.
   * @param  country_iso     Country ISO string.
   * @param  state_iso       State ISO string.
   */
  Admin(const uint32_t country_offset,
        const uint32_t state_offset,
        const std::string& country_iso,
        const std::string& state_iso);

  /**
   * Get the country ISO3166-1 code.
   * @return  Returns the ISO country code.
   */
  std::string country_iso() const;

  /**
   * Get the state ISO code. Country ISO + dash + state ISO will give
   * you ISO3166-2 for state.
   * @return  Returns the state ISO code.
   */
  std::string state_iso() const;

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
  uint32_t country_offset_;         // country name offset
  uint32_t state_offset_;           // state name offset
  char country_iso_[kCountryIso]{}; // country ISO3166-1
  char state_iso_[kStateIso]{};     // state ISO3166-2
  char spare_[3]{};                 // spare for byte alignment
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_ADMIN_H_
