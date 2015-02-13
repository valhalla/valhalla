#ifndef VALHALLA_BALDR_TURNRESTRICTION_H_
#define VALHALLA_BALDR_TURNRESTRICTION_H_

#include <unordered_map>
#include <string>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Simple turn restriction. From one directed edge via a node to
 * other edges from that via node.
 */
class TurnRestriction {
 public:
  /**
   * Constructor
   */
  TurnRestriction();

  /**
   * Get the index of the "from" directed edge for this turn restriction.
   * @return  Returns the directed edge index (within the same tile
   *          as the turn restriction information).
   */
   uint32_t edgeindex() const;

   /**
    * Get the turn restriction type.
    * @return  Returns the turn restriction type
    */
   RestrictionType type() const;

   /**
    * Get the restriction mask. This is a bit mask where bits set to 1 indicate
    * a restricted turn based on index of the outbound directed edge at the
    * end node of this directed edge.
    * @return  Returns the restriction mask.
    */
   uint32_t restriction_mask() const;

 protected:

  struct IndexAndType {
    uint32_t edgeindex  : 22;     // kMaxTileEdgeCount in nodeinfo.h: 22 bits
    uint32_t type       :  4;
    uint32_t spare      :  6;
  };
  IndexAndType data_;

  // Restriction mask. A bit mask where bits set to 1 indicate a restricted
  // turn based on index of the outbound directed edge at the end node of
  // this directed edge.
  uint32_t restriction_mask_;
};

}
}


#endif  // VALHALLA_BALDR_TURNRESTRICTION_H_
