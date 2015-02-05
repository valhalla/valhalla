#ifndef VALHALLA_BALDR_SIGN_H_
#define VALHALLA_BALDR_SIGN_H_

#include <stdint.h>

#include <valhalla/midgard/util.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

/**
 * Holds a generic sign with type and text. Text is stored in the GraphTile
 * text list and the offset is stored within the sign. The directed edge index
 * within the tile is also stored so that signs can be found via the directed
 * edge index. This is a read only base class.
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
   * Get the index of the directed edge this sign applies to.
   * @return  Returns the directed edge index (within the same tile
   *          as the sign information).
   */
  uint32_t edgeindex() const;

  /**
   * Get the sign type.
   * @return  Returns the sign type
   */
  Sign::Type type() const;

  /**
   * Get the offset into the GraphTile text list for the text associated
   * with the sign.
   * @return  Returns the text offset.
   */
  uint32_t text_offset() const;

 protected:
  // Constructor
  Sign(const uint32_t idx, const Sign::Type& type,
           const uint32_t text_offset);

  struct IndexAndType {
    uint32_t   edgeindex  : 22;     // kMaxTileEdgeCount in nodeinfo.h: 22 bits
    Sign::Type type       :  8;
    uint32_t   spare      :  2;
  };
  IndexAndType data_;
  uint32_t text_offset_;
};

}
}

#endif  // VALHALLA_BALDR_SIGN_H_
