#ifndef VALHALLA_BALDR_SIGN_H_
#define VALHALLA_BALDR_SIGN_H_

#include <cstdint>
#include <stdint.h>
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace baldr {

/**
 * Holds a generic sign with type and text. Text is stored in the GraphTile
 * text list and the offset is stored within the sign. The directed edge index
 * within the tile is also stored so that signs can be found via the directed
 * edge index.
 */
class Sign {
public:
  enum class Type : uint8_t { kExitNumber, kExitBranch, kExitToward, kExitName };

  /**
   * Constructor given arguments.
   * @param  idx  Directed edge index to which this sign applies.
   * @param  type Sign type.
   * @param  rn   Boolean indicating whether this sign indicates a route number.
   * @param  text_offset  Offset to text in the names/text table.
   */
  Sign(const uint32_t idx, const Sign::Type& type, const bool rn, const uint32_t text_offset)
      : edgeindex_(idx), type_(static_cast<uint32_t>(type)), is_route_num_(rn), tagged_(0),
        text_offset_(text_offset) {
  }

  /**
   * Get the index of the directed edge this sign applies to.
   * @return  Returns the directed edge index (within the same tile
   *          as the sign information).
   */
  uint32_t edgeindex() const {
    return edgeindex_;
  }

  /**
   * Set the directed edge index.
   * @param  idx  Directed edge index.
   */
  void set_edgeindex(const uint32_t idx) {
    edgeindex_ = idx;
  }

  /**
   * Get the sign type.
   * @return  Returns the sign type.
   */
  Sign::Type type() const {
    return static_cast<Sign::Type>(type_);
  }

  /**
   * Does this sign record indicate a route number.
   * @return  Returns true if the sign record is a route number.
   */
  bool is_route_num() const {
    return is_route_num_;
  }

  /**
   * Is the sign text tagged (Future use for special tagging such as language code)
   * @return Returns true if the sign text is tagged.
   */
  bool tagged() const {
    return tagged_;
  }

  /**
   * Get the offset into the GraphTile text list for the text associated
   * with the sign.
   * @return  Returns the text offset.
   */
  uint32_t text_offset() const {
    return text_offset_;
  }

protected:
  uint32_t edgeindex_ : 22; // kMaxTileEdgeCount in nodeinfo.h: 22 bits
  uint32_t type_ : 8;
  uint32_t is_route_num_ : 1;
  uint32_t tagged_ : 1; // For future use to support "tagged" text strings.
                        // Similar to EdgeInfo, for compatibility any tagged strings
                        // will be skipped until code is available to properly use them.

  uint32_t text_offset_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_SIGN_H_
