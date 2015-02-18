#ifndef VALHALLA_BALDR_DIRECTEDEDGE_H_
#define VALHALLA_BALDR_DIRECTEDEDGE_H_

#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

// Bike Network constants. Bit constants.
constexpr uint8_t kNcn = 1;   // Part of national bike network
constexpr uint8_t kRcn = 2;   // Part of regional bike network
constexpr uint8_t kLcn = 4;   // Part of local bike network
constexpr uint8_t kMcn = 8;   // Part of mountain bike network

// Maximum offset to edge information
constexpr uint32_t kMaxEdgeInfoOffset = 16777215;   // 2^24 bytes

// Maximum length of an edge
constexpr uint32_t kMaxEdgeLength = 16777215;   // 2^24 meters

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
   * Gets the end node of this directed edge.
   * @return  Returns the end node.
   */
  GraphId endnode() const;

  /**
   * Offset to the common edge data. The offset is from the start
   * of the common edge information  within a tile.
   * @return  Returns offset from the start of the edge info within a tile.
   */
  uint32_t edgeinfo_offset() const;

  /**
   * Does this directed edge have exit signs?
   * @return  Returns true if the directed edge has exit signs,
   *          false if not.
   */
  bool exitsign() const;

  /**
   * Gets the length of the edge in meters.
   * @return  Returns the length in meters.
   */
  uint32_t length() const;

  /**
   * Get the elevation factor.  TODO
   * @return  Returns the elevation factor (0-15).
   */
  uint32_t elevation() const;

  /**
   * Get the number of lanes for this directed edge.
   * @return  Returns the number of lanes for this directed edge.
   */
  uint32_t lanecount() const;

  /**
   * Get the access modes in the forward direction (bit field).
   */
  uint8_t forwardaccess() const;

  /**
   * Get the access modes in the reverse direction (bit field).
   */
  uint8_t reverseaccess() const;

  /**
   * Gets the speed in KPH.
   * @return  Returns the speed in KPH.
   */
  uint8_t speed() const;

  /**
   * Get the importance of the road/path.
   * TODO - rename to classification or roadclass since it returns RoadClass?!
   * @return  Returns importance / RoadClass
   */
  RoadClass importance() const;

  /**
   * Is this edge a link/ramp?
   */
  bool link() const;

  /**
   * Get the use of this edge.
   */
  Use use() const;

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
   * Get the smoothness.
   */
  Surface surface() const;

  /**
   * Get the cycle lane.
   */
  CycleLane cyclelane() const;

  /**
   * Does this edge represent a transition up one level in the hierarchy.
   * Transition edges move between nodes in different levels of the
   * hierarchy but have no length or other attribution. An upward transition
   * is a transition from a minor road hierarchy (local) to more major
   * (arterial).
   * @return  Returns true if the edge is a transition from a lower level
   *          to a higher (false if not).
   */
  bool trans_up() const;

  /**
   * Does this edge represent a transition down one level in the hierarchy.
   * Transition edges move between nodes in different levels of the
   * hierarchy but have no length or other attribution. An downward transition
   * is a transition from a major road hierarchy (highway) to more minor
   * (arterial).
   * @return  Returns true if the edge is a transition from an upper level
   *          to a lower (false if not).
   */
  bool trans_down() const;

  /**
   * Does this edge represent a shortcut between 2 nodes? Shortcuts bypass
   * nodes that only connect to lower levels in the hierarchy (other than the
   * 1-2 higher level edges that superseded by the shortcut).
   * @return  Returns true if this edge is a shortcut edge, false if not.
   */
  bool shortcut() const;

  /**
   * Is this edge superseded by a shortcut edge? Superseded edges can be
   * skipped unless downward transitions are allowed.
   * @return  Returns true if this edge is part of a shortcut edge, false
   *          if not.
   */
  bool superseded() const;

  /**
   * Is this directed edge stored forward in edgeinof (true) or
   * reverse (false).
   * @return  Returns true if stored forward, false if reverse.
   */
  bool forward() const;

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
   * Gets the intersection internal flag.
   * @return  Returns true if the edge is internal to an intersection. This
   *          is derived from OSM and used for doubly digitized intersections.
   */
  bool internal() const;

  /**
   * Get the index of the directed edge on the local level of the graph
   * hierarchy. This is used for turn restrictions so the edges can be
   * identified on the different levels.
   * @return  Returns the index of the edge on the local level.
   */
  uint32_t localedgeidx() const;

  /**
   * Simple turn restrictions from the end of this directed edge.
   * These are turn restrictions from one edge to another that apply to
   * all vehicles, at all times.
   * @return  Returns a bit mask that indicates the local edge indexes
   *          of outbound directed edges that are restricted.
   */
  uint32_t restrictions() const;

  /**
   * Get the computed version of DirectedEdge attributes.
   * @return   Returns internal version.
   */
  static const uint64_t internal_version();

 protected:
  // End node
  GraphId endnode_;

  // Data offsets and flags for extended data.
  struct DataOffsets {
    uint32_t edgeinfo_offset : 24; // Offset to edge data.
    uint32_t spare           :  5;
    uint32_t start_mer       :  1; // Directed edge starts a multi-edge
                                   // restriction
    uint32_t end_mer         :  1; // Directed edge ends a multi-edge
                                   // restriction
    uint32_t exitsign        :  1; // Does this directed edge have exit signs
  };
  DataOffsets dataoffsets_;

  // Geometric attributes: length, elevation factor, lane count.
  struct GeoAttributes {
    uint32_t length        : 24;  // Length in meters
    uint32_t elevation     :  4;  // Elevation factor
    uint32_t lanecount     :  4;  // Number of lanes
  };
  GeoAttributes geoattributes_;

  // Attributes. Can be used in edge costing methods to favor or avoid edges.
  // TODO: add carriage type (LOCAL, EXPRESS, etc.) if we have it
  // TODO - 64 bit or should we have 2x32 bit?
  struct Attributes {
    uint64_t ferry          : 1;
    uint64_t railferry      : 1;
    uint64_t toll           : 1;
    uint64_t dest_only      : 1;
    uint64_t spare          : 1;
    uint64_t tunnel         : 1;
    uint64_t bridge         : 1;
    uint64_t roundabout     : 1;
    uint64_t surface        : 3;  // representation of smoothness
    uint64_t cycle_lane     : 2;
    uint64_t trans_up       : 1;  // Edge represents a transition up one
                                  // level in the hierarchy
    uint64_t trans_down     : 1;  // Transition down one level
    uint64_t shortcut       : 1;  // Shortcut edge
    uint64_t superseded     : 1;  // Edge is superseded by a shortcut
    uint64_t forward        : 1;  // Is the edge info forward or reverse
    uint64_t not_thru       : 1;  // Edge leads to "no-through" region
    uint64_t opp_index      : 5;  // Opposing directed edge index
    uint64_t bikenetwork    : 4;  // Edge that is part of a bicycle network
    uint64_t internal       : 1;  // Edge that is internal to an intersection
    uint64_t localedgeidx   : 7;  // Index of the edge on the local level
    uint64_t restrictions   : 7;  // Restrictions - mask of local edge indexes
                                  // at the end node that are restricted.
    uint64_t spare2         : 21;
  };
  Attributes attributes_;

  // Legal access to the directed link (also include reverse direction access).
  // See graphconstants.h.
  Access forwardaccess_;
  Access reverseaccess_;

  // Speed in kilometers per hour. Range 0-250 KPH. Applies to
  // "normal vehicles". Save values above 250 as special cases
  // (closures, construction, // etc.)
  // TODO - do we need or does OSM support "truck speed"
  uint8_t speed_;

  // Classification and use information
  struct Classification {
    uint8_t importance  : 3;     // Importance of the road/path
    uint8_t link        : 1;     // *link tag - Ramp or turn channel
    uint8_t use         : 4;     // Use / form
  };
  Classification classification_;

  // TODO - turn restrictions, costs,
  struct IntersectionTransition {
    uint32_t spare:         32;
  };
  IntersectionTransition transitions_;
};

}
}

#endif  // VALHALLA_BALDR_DIRECTEDEDGE_H_
