#ifndef VALHALLA_MJOLNIR_OSMLINGUISTIC_H
#define VALHALLA_MJOLNIR_OSMLINGUISTIC_H

#include <cstdint>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

// OSM Linguistic.  IPA/Nt-sampa/Katakana/Jeita phonems/pronunciations/languages
struct OSMLinguistic {

  enum class Type : uint8_t {
    kName,
    kNameLeft,
    kNameRight,
    kNameForward,
    kNameBackward,
    kNodeName,
    kAltName,
    kAltNameLeft,
    kAltNameRight,
    kOfficialName,
    kOfficialNameLeft,
    kOfficialNameRight,
    kTunnelName,
    kTunnelNameLeft,
    kTunnelNameRight,
    kRef,
    kRefLeft,
    kRefRight,
    kNodeRef,
    kIntRef,
    kIntRefLeft,
    kIntRefRight,
    kDestination,
    kDestinationForward,
    kDestinationBackward,
    kDestinationRef,
    kDestinationRefTo,
    kDestinationStreet,
    kDestinationStreetTo,
    kJunctionRef,
    kJunctionName
  };
  enum class DiffType : uint8_t { kLeft, kRight, kForward, kBackward };

  /**
   * Constructor
   */
  OSMLinguistic() {
    memset(this, 0, sizeof(OSMLinguistic));
  }

  /**
   * Set way id.
   * @param   id  way id
   */
  void set_name_offset(const uint32_t id) {
    name_offset_ = id;
  }

  /**
   * Get the name offset
   * @return  Returns name_offset_.
   */
  uint64_t name_offset() const {
    return name_offset_;
  }

  /**
   * Set the key
   * @param   	type Type of pronunciation/language
   * @param		alpha Type of alphabet
   */
  void set_key(const uint8_t type, const uint8_t alpha) {
    key_.type_ = type;
    key_.alpha_ = alpha;
  }

  // OSM way Id
  uint32_t name_offset_;
  struct Key {
    uint16_t type_ : 8;
    uint16_t alpha_ : 8;
  };
  Key key_;
  uint16_t spare_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMLINGUISTIC_H
