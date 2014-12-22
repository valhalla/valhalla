#ifndef VALHALLA_BALDR_DIRECTEDEDGE_H_
#define VALHALLA_BALDR_DIRECTEDEDGE_H_

#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace baldr {

/**
 * Directed edge within the graph.
 * @author  David W. Nesbitt
 */
class DirectedEdge {
 public:
  /**
   * Constructor
   */
  DirectedEdge();

  /**
   * Gets the length of the edge in kilometers.
   * @return  Returns the length in kilometers.
   */
  float length() const;

  /**
   * Gets the end node of this directed edge.
   * @return  Returns the end node.
   */
  GraphId endnode() const;

  // TODO - methods for access

  /**
   * Gets the speed in KPH. TODO - cast to float instead?
   * @return  Returns the speed in KPH.
   */
  float speed() const;

  /**
   * Offset to the common edge data. The offset is from the start
   * of the common edge data within a tile.
   * @return  Returns offset from the start of the edge data within a tile.
   */
  uint32_t edgedataoffset() const;

 protected:
  // Length of the link in miles
  float length_;

  // End node
  GraphId endnode_;

  // Offset to the common edge data within the tile
  uint32_t edgedataoffset_;

  // Legal access to the directed link ((also include reverse direction access).
  // TODO - come up with final set of values!
  struct Access {
    unsigned char car_          : 1;
    unsigned char taxi_         : 1;
    unsigned char truck_        : 1;
    unsigned char pedestrian_   : 1;
    unsigned char bicycle_      : 1;
    unsigned char emergency_    : 1;
    unsigned char horse_        : 1;  // ???
    unsigned char spare_        : 1;
  };
  Access forwardaccess_;
  Access reverseaccess_;

  // Attributes. Can be used in edge costing methods to favor or avoid edges.
  struct Attributes {
    uint32_t ferry_          : 1;
    uint32_t railferry_      : 1;  // ???
    uint32_t toll_           : 1;
    uint32_t private_        : 1;
    uint32_t unpaved_        : 1;
    uint32_t tunnel_         : 1;
    uint32_t spare           : 22;
    uint32_t elevation_      : 4;  // Elevation factor
  };
  Attributes attributes_;

  // Classification and use information
  struct Classification {
    uint32_t class_   : 3;     // Road class (importance)
    uint32_t link_    : 1;     // *link tag - Ramp or turn channel
    uint32_t use_     : 4;     // Something like "form of way"
    uint32_t spare_   : 16;
  };
  Classification classification_;

  // Speed in kilometers. Range 0-250 KPH. Applies to "normal vehicles".
  // Save values above 250 as special cases (closures, construction,
  // etc.)
  // TODO - do we need or does OSM support "truck speed"
  unsigned char speed_;

  // TODO - byte alignment / sizing

};

}
}

#endif  // VALHALLA_BALDR_DIRECTEDEDGE_H_
