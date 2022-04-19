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
 * edge or node index.
 */
class Sign {
public:
  enum class Type : uint8_t {
    kExitNumber,
    kExitBranch,
    kExitToward,
    kExitName,
    kGuideBranch,
    kGuideToward,
    kJunctionName,
    kGuidanceViewJunction,
    kGuidanceViewSignboard,
    kPronunciation = 255
  };

  /**
   * Constructor given arguments.
   * @param  idx  Directed edge or node index to which this sign applies.
   * @param  type Sign type.
   * @param  rn_type   Boolean indicating whether this sign indicates a route number or the guidance
   * view type.
   * @param  text_offset  Offset to text in the names/text table.
   */
  Sign(const uint32_t idx,
       const Sign::Type& type,
       const bool rn_type,
       const bool tagged,
       const uint32_t text_offset)
      : index_(idx), type_(static_cast<uint32_t>(type)), route_num_type_(rn_type), tagged_(tagged),
        text_offset_(text_offset) {
  }

  /**
   * Get the index of the directed edge or node this sign applies to.
   * @return  Returns the directed edge or node index (within the same tile
   *          as the sign information).
   */
  uint32_t index() const {
    return index_;
  }

  /**
   * Set the directed edge or node index.
   * @param  idx  Directed edge or node index.
   */
  void set_index(const uint32_t idx) {
    index_ = idx;
  }

  /**
   * Get the sign type.
   * @return  Returns the sign type.
   */
  Sign::Type type() const {
    return static_cast<Sign::Type>(type_);
  }

  /**
   * Does this sign record indicate a route number, phoneme for a node, or the guidance view type
   * @return  Returns true if the sign record is a route number, phoneme for a node, or if this is a
   * guidance view sign returning true indicates that we are a base image and false
   * if we are a overlay image
   */
  bool is_route_num_type() const {
    return route_num_type_;
  }

  /**
   * Is the sign text tagged
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

  // operator < - for sorting. Sort by edge or node index and then by type.
  bool operator<(const Sign& other) const {
    if (index() == other.index()) {
      return type() < other.type();
    } else {
      return index() < other.index();
    }
  }

protected:
  uint32_t index_ : 22; // kMaxTileEdgeCount in nodeinfo.h: 22 bits
  uint32_t type_ : 8;
  uint32_t route_num_type_ : 1;
  uint32_t tagged_ : 1;

  uint32_t text_offset_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_SIGN_H_
