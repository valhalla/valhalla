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
  enum class Type : uint8_t {
    kExitNumber,
    kExitBranch,
    kExitToward,
    kExitName
  };

  /**
   * Constructor given arguments.
   * @param  idx  Directed edge index to which this sign applies.
   * @param  type Sign type.
   * @param  text_offset  Offset to text in the names/text table.
   */
  Sign(const uint32_t idx, const Sign::Type& type, const uint32_t text_offset);

  /**
   * Get the index of the directed edge this sign applies to.
   * @return  Returns the directed edge index (within the same tile
   *          as the sign information).
   */
  uint32_t edgeindex() const;

  /**
   * Set the directed edge index.
   * @param  idx  Directed edge index.
   */
  void set_edgeindex(const uint32_t idx);

  /**
   * Get the sign type.
   * @return  Returns the sign type.
   */
  Sign::Type type() const;

  /**
   * Get the offset into the GraphTile text list for the text associated
   * with the sign.
   * @return  Returns the text offset.
   */
  uint32_t text_offset() const;

 protected:
  uint32_t edgeindex_  : 22;     // kMaxTileEdgeCount in nodeinfo.h: 22 bits
  uint32_t type_       :  8;
  uint32_t spare_      :  2;

  uint32_t text_offset_;
};

}
}

#endif  // VALHALLA_BALDR_SIGN_H_
