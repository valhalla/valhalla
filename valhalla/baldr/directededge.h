#ifndef VALHALLA_BALDR_DIRECTEDEDGE_H_
#define VALHALLA_BALDR_DIRECTEDEDGE_H_

#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

// Access constants. Bit constants.
constexpr uint8_t kAutoAccess       = 1;
constexpr uint8_t kPedestrianAccess = 2;
constexpr uint8_t kBicycleAccess    = 4;
constexpr uint8_t kTruckAccess      = 8;
constexpr uint8_t kEmergencyAccess  = 16;
constexpr uint8_t kTaxiAccess       = 32;
constexpr uint8_t kHorseAccess      = 64;

// Bike Network constants. Bit constants.
constexpr uint8_t kNcn = 1;
constexpr uint8_t kRcn = 2;
constexpr uint8_t kLcn = 4;

/**
 * Directed edge within the graph.
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

  /**
   * Get the access modes in the forward direction (bit field).
   */
  uint8_t forwardaccess() const;

  /**
   * Get the access modes in the reverse direction (bit field).
   */
  uint8_t reverseaccess() const;

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

  /**
   * Is this edge part of a ferry?
   */
  bool ferry() const;

  /**
   * Is this edge part of a rail ferry?
   */
  bool railferry() const;

  /**
   * Does this edge have a toll or is it part of a toll road?
   */
  bool toll() const;

  /**
   * Is this edge part of a private or no through road that allows access
   * only if required to get to a destination?
   */
  bool destonly() const;

  /**
   * Is this edge unpaved or bad surface?
   */
  bool unpaved() const;

  /**
   * Is this edge a tunnel?
   */
  bool tunnel() const;

  /**
   * Is this edge a bridge?
   */
  bool bridge() const;

  /**
   * Is this edge a roundabout?
   */
  bool roundabout() const;

  /**
   * Edge leads to a "no thru" region where there are no exits other than
   * the incoming edge. This flag is populated by processing the graph to
   * identify such edges. This is used to speed pedestrian routing.
   * @return  Returns true if the edge leads into a no thru region.
   */
  bool not_thru() const;

  /**
   * Get the index of the opposing directed edge at the end node of this
   * directed edge. Can be used to find the start node of this directed edge
   * and to detect U-turns.
   * @return  Returns the index of the opposing directed edge at the end node
   *          of this directed edge.
   */
  uint32_t opp_index() const;

  /**
   * Get the bike network mask for this directed edge.
   * @return  Returns the bike network mask for this directed edge.
   */
  uint32_t bikenetwork() const;

  /**
   * Get the number of lanes for this directed edge.
   * @return  Returns the number of lanes for this directed edge.
   */
  uint32_t lanes() const;

  /**
   * Get the importance of the road/path
   * @return  Returns importance / RoadClass
   */
  RoadClass importance() const;

  /**
   * Get the use of this edge.
   */
  Use use() const;

 protected:
  // End node
  GraphId endnode_;

  // Offset to the common edge data within the tile
  uint32_t edgedataoffset_;

  // Length of the link in kilometers. TODO - change to meters? perhaps use
  // 20 or so bits? Would that preclude building routing?
  float length_;

  // Legal access to the directed link ((also include reverse direction access).
  // TODO - come up with final set of values!
  union Access {
    struct Fields {
      uint8_t car          : 1;
      uint8_t pedestrian   : 1;
      uint8_t bicycle      : 1;
      uint8_t truck        : 1;
      uint8_t emergency    : 1;
      uint8_t taxi         : 1;
      uint8_t horse        : 1;  // ???
      uint8_t spare_       : 1;
    } fields;
    uint64_t v;
  };
  Access forwardaccess_;
  Access reverseaccess_;

  // Speed in kilometers. Range 0-250 KPH. Applies to "normal vehicles".
  // Save values above 250 as special cases (closures, construction,
  // etc.)
  // TODO - do we need or does OSM support "truck speed"
  uint8_t speed_;

  // Classification and use information
  struct Classification {
    uint8_t importance  : 3;     // Importance of the road/path
    uint8_t link        : 1;     // *link tag - Ramp or turn channel
    uint8_t use         : 4;     // Use / form
  };
  Classification classification_;

  // Attributes. Can be used in edge costing methods to favor or avoid edges.
  struct Attributes {
    uint32_t ferry          : 1;
    uint32_t railferry      : 1;
    uint32_t toll           : 1;
    uint32_t dest_only      : 1;
    uint32_t unpaved        : 1;
    uint32_t tunnel         : 1;
    uint32_t bridge         : 1;
    uint32_t roundabout     : 1;
    uint32_t spare          : 6;
    uint32_t not_thru       : 1;  // Edge leads to "no-through" region
    uint32_t opp_index      : 5;  // Opposing directed edge index
    uint32_t bikenetwork    : 4;
    uint32_t lanecount      : 4;
    uint32_t elevation      : 4;  // Elevation factor
  };
  Attributes attributes_;

  // TODO - walkway/path

  // TODO - byte alignment / sizing
};

}
}

#endif  // VALHALLA_BALDR_DIRECTEDEDGE_H_
