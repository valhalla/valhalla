#ifndef VALHALLA_BALDR_DIRECTEDEDGE_H_
#define VALHALLA_BALDR_DIRECTEDEDGE_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/turn.h>
#include <valhalla/baldr/json.h>

namespace valhalla {
namespace baldr {

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
  uint64_t edgeinfo_offset() const;

  /**
   * General restriction or access condition (per mode) for this directed edge.
   * @return  Returns the restriction for the directed edge.
   */
  uint64_t access_restriction() const;

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
   * Get the weighted grade factor
   * @return  Returns the weighted grade factor (0-15).
   *          where 0 is a 10% grade and 15 is 15%
   */
  uint32_t weighted_grade() const;

  /**
   * Get the road curvature factor. TODO
   * @return  Returns the curvature factor (0-15).
   */
  uint32_t curvature() const;

  /**
   * Is driving on the right hand side of the road along this edge?
   * @return  Returns true if this edge uses right-side driving, false if
   *          left-side driving.
   */
  bool drive_on_right() const;

  /**
   * Is this edge part of a ferry?
   * @return  Returns true if this edge is part of a ferry, false if not.
   */
  bool ferry() const;

  /**
   * Is this edge part of a rail ferry?
   * @return  Returns true if this edge is part of a rail ferry, false if not.
   */
  bool railferry() const;

  /**
   * Does this edge have a toll or is it part of a toll road?
   * @return  Returns true if this edge is part of a toll road, false if not.
   */
  bool toll() const;

  /**
   * Does this edge have a seasonal access (e.g., closed in the winter)?
   * @return  Returns true if this edge has seasonal access, false if not.
   */
  bool seasonal() const;

  /**
   * Is this edge part of a private or no through road that allows access
   * only if required to get to a destination?
   * @return  Returns true if the edge is destination only / private access.
   */
  bool destonly() const;

  /**
   * Is this edge part of a tunnel?
   * @return  Returns true if this edge is part of a tunnel, false if not.
   */
  bool tunnel() const;

  /**
   * Is this edge part of a bridge?
   * @return  Returns true if this edge is part of a bridge, false if not.
   */
  bool bridge() const;

  /**
   * Is this edge part of a roundabout?
   * @return  Returns true if this edge is part of a roundabout, false if not.
   */
  bool roundabout() const;

  /**
   * Is this edge is unreachable by driving. This can happen if a driveable
   * edge is surrounded by pedestrian only edges (e.g. in a city center) or
   * is not properly connected to other edges.
   * @return  Returns true if this edge is unreachable by auto.
   */
  bool unreachable() const;

  /**
   * A traffic signal occurs at the end of this edge.
   * @return  Returns true if a traffic signal is present at the end of the
   *          directed edge.
   */
  bool traffic_signal() const;

  /**
   * Is this directed edge stored forward in edgeinfo (true) or
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
   * directed edge. Can be used to find the start node of this directed edge.
   * @return  Returns the index of the opposing directed edge at the end node
   *          of this directed edge.
   */
  uint32_t opp_index() const;

  /**
   * Get the cycle lane type along this edge.
   * @returns   Returns the type (if any) of bicycle lane along this edge.
   */
  CycleLane cyclelane() const;

  /**
   * Get the bike network mask for this directed edge.
   * @return  Returns the bike network mask for this directed edge.
   */
  uint32_t bike_network() const;

  /**
   * Get the truck route flag for this directed edge.
   * @return  Returns true if part of a truck network/route, false otherwise.
   */
  bool truck_route() const;

  /**
   * Get the number of lanes for this directed edge.
   * @return  Returns the number of lanes for this directed edge.
   */
  uint32_t lanecount() const;

  /**
   * Simple turn restrictions from the end of this directed edge.
   * These are turn restrictions from one edge to another that apply to
   * all vehicles, at all times.
   * @return  Returns a bit mask that indicates the local edge indexes
   *          of outbound directed edges that are restricted.
   */
  uint32_t restrictions() const;

  /**
   * Get the specialized use of this edge.
   * @return  Returns the use type of this edge.
   */
  Use use() const;

  /**
   * Is this edge a transit line (bus or rail)?
   * @return  Returns true if this edge is a transit line.
   */
  bool IsTransitLine() const;

  /**
   * Get the speed type (see graphconstants.h)
   * @return  Returns the speed type.
   */
  SpeedType speed_type() const;

  /**
   * Get the country crossing flag.
   * @return  Returns true if the edge crosses into a new country.
   */
  bool ctry_crossing() const;

  /**
   * Get the access modes in the forward direction (bit field).
   * @return  Returns the access modes in the forward direction.
   */
  uint32_t forwardaccess() const;

  /**
   * Get the access modes in the reverse direction (bit field).
   * @return  Returns the access modes in the reverse direction.
   */
  uint32_t reverseaccess() const;

  /**
   * Gets the speed in KPH.
   * @return  Returns the speed in KPH.
   */
  uint32_t speed() const;

  /**
   * Gets the truck speed in KPH.
   * @return  Returns the truck speed in KPH.
   */
  uint32_t truck_speed() const;

  /**
   * Get the classification (importance) of the road/path.
   * @return  Returns road classification / importance.
   */
  RoadClass classification() const;

  /**
   * Is this edge unpaved or bad surface?
   */
  bool unpaved() const;

  /**
   * Get the surface type (see graphconstants.h). This is a general indication
   * of smoothness.
   */
  Surface surface() const;

  /**
   * Is this edge a link/ramp?
   */
  bool link() const;

  /**
   * Gets the intersection internal flag. This indicates the edge is "internal"
   * to an intersection. This is derived from OSM based on geometry of an
   * of nearby edges and is used for routing behavior on doubly digitized
   * intersections.
   * @return  Returns true if the edge is internal to an intersection.
   */
  bool internal() const;

  /**
   * Gets the turn type given the prior edge's local index
   * (index of the inbound edge).
   * @param  localidx  Local index at the node of the inbound edge.
   * @return  Returns the turn type (see turn.h)
   */
  Turn::Type turntype(const uint32_t localidx) const;

  /**
   * Is there an edge to the left, in between the from edge and this edge.
   * @param localidx  Local index at the node of the inbound edge.
   * @return  Returns true if there is an edge to the left, false if not.
   */
  bool edge_to_left(const uint32_t localidx) const;

  /**
   * Get the stop impact when transitioning from the prior edge (given
   * by the local index of the corresponding inbound edge at the node).
   * @param  localidx  Local index at the node of the inbound edge.
   * @return  Returns the relative stop impact from low (0) to high (7).
   */
  uint32_t stopimpact(const uint32_t localidx) const;

  /**
   * Get the transit line Id (for departure lookups along an edge)
   * @return  Returns the transit line Id.
   */
  uint32_t lineid() const;

  /**
   * Is there an edge to the right, in between the from edge and this edge.
   * @param localidx  Local index at the node of the inbound edge.
   * @return  Returns true if there is an edge to the right, false if not.
   */
  bool edge_to_right(const uint32_t localidx) const;

  /**
   * Get the index of the directed edge on the local level of the graph
   * hierarchy. This is used for turn restrictions so the edges can be
   * identified on the different levels.
   * @return  Returns the index of the edge on the local level.
   */
  uint32_t localedgeidx() const;

  /**
   * Get the index of the opposing directed edge on the local hierarchy level
   * at the end node of this directed edge. Only stored for the first 8 edges
   * so it can be used for edge transition costing.
   * @return  Returns the index of the opposing directed edge on the local
   *          hierarchy level at end node of this directed edge.
   */
  uint32_t opp_local_idx() const;

  /**
   * Indicates the mask of the superseded edge bypassed by a shortcut.
   * Shortcuts bypass nodes that only connect to lower levels in the hierarchy
   * (other than the 1-2 higher level edges that superseded by the shortcut).
   * @return  Returns the shortcut mask of the matching superseded edge
   *          outbound from the node. 0 indicates the edge is not a shortcut.
   */
  uint32_t shortcut() const;

  /**
   * Mask indicating the shortcut that supersedes this directed edge.
   * Superseded edges can be skipped unless downward transitions are allowed.
   * @return  Returns the mask of the matching shortcut edge that supersedes this
   *          directed edge outbound from the node. 0 indicates the edge is not
   *          superseded by a shortcut edge.
   */
  uint32_t superseded() const;

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
   * Is this edge a shortcut edge. If there are more than kMaxShortcutsFromNode
   * shortcuts no mask is set but this flag is set to true.
   * @return  Returns true if this edge is a shortcut.
   */
  bool is_shortcut() const;

  /**
   * Create a json object representing this edge
   * @return  Returns the json object
   */
  json::MapPtr json() const;

 protected:
  // End node
  GraphId endnode_;

  // Data offsets and flags for extended data. Where a flag exists the actual
  // data can be indexed by the directed edge Id within the tile.
  uint64_t edgeinfo_offset_           : 25; // Offset to edge data.
  uint64_t access_restriction_        : 12; // General restriction or access
                                            // condition (per mode)
  uint64_t start_complex_restriction_ : 12; // Complex restriction (per mode)
                                            // starts on this directed edge
  uint64_t end_complex_restriction_   : 12; // Complex restriction (per mode)
                                            // ends on this directed edge
  uint64_t exitsign_                  : 1;  // Exit signs exist for this edge
  uint64_t spare1_                    : 2;

  // Attributes. Can be used in edge costing methods to favor or avoid edges.
  // Speed values above 250 used for special cases (closures, construction)
  uint64_t speed_          : 8; // Speed (kph)
  uint64_t truck_speed_    : 8; // Truck speed (kph)
  uint64_t restrictions_   : 8; // Restrictions - mask of local edge indexes
                                // at the end node that are restricted.
  uint64_t lanecount_      : 4; // Number of lanes
  uint64_t bike_network_   : 4; // Edge that is part of a bicycle network
  uint64_t use_            : 6; // Specific use types
  uint64_t speed_type_     : 2; // Speed type (tagged vs. categorized)
  uint64_t opp_index_      : 7; // Opposing directed edge index
  uint64_t drive_on_right_ : 1; // Driving side. Right if true (false=left)
  uint64_t ferry_          : 1; // Ferry (boat)
  uint64_t railferry_      : 1; // Ferry (rail) - e.g. Euro Rail Tunnel
  uint64_t toll_           : 1; // Edge is part of a toll road.
  uint64_t seasonal_       : 1; // Seasonal access (ex. no access in winter)
  uint64_t dest_only_      : 1; // Access allowed to destination only
                                //  (private or no through traffic)
  uint64_t tunnel_         : 1; // Is this edge part of a tunnel
  uint64_t bridge_         : 1; // Is this edge part of a bridge?
  uint64_t roundabout_     : 1; // Edge is part of a roundabout
  uint64_t unreachable_    : 1; // Edge that is unreachable by driving
  uint64_t traffic_signal_ : 1; // Traffic signal at end of the directed edge
  uint64_t forward_        : 1; // Is the edge info forward or reverse
  uint64_t not_thru_       : 1; // Edge leads to "no-through" region
  uint64_t cycle_lane_     : 2; // Does this edge have bicycle lanes?
  uint64_t truck_route_    : 1; // Edge that is part of a truck route/network
  uint64_t ctry_crossing_  : 1; // Does the edge cross into new country

  // Legal access to the directed link (also include reverse direction access).
  // See graphconstants.h.
  uint64_t forwardaccess_  : 12;
  uint64_t reverseaccess_  : 12;
  uint64_t classification_ : 3;  // Classification/importance of the road/path
  uint64_t surface_        : 3;  // representation of smoothness
  uint64_t link_           : 1;  // *link tag - Ramp or turn channel
  uint64_t internal_       : 1;  // Edge that is internal to an intersection
  uint64_t spare1          : 32;

  // Geometric attributes: length, weighted grade, curvature factor.
  // Turn types between edges.
  uint64_t turntype_        : 24;   // Turn type (see graphconstants.h)
  uint64_t edge_to_left_    :  8;   // Is there an edge to the left (between
                                    // the "from edge" and this edge)
  uint64_t length_          : 24;   // Length in meters
  uint64_t weighted_grade_  :  4;   // Weighted estimate of grade
  uint64_t curvature_       :  4;   // Curvature factor

  // Stop impact among edges
  struct StopImpact {
    uint32_t stopimpact      : 24; // Stop impact between edges
    uint32_t edge_to_right   :  8; // Is there an edge to the right (between
                                   // "from edge" and this edge)
  };

  // Store either the stop impact or the transit line identifier. Since
  // transit lines are schedule based they have no need for edge transition
  // logic so we can safely share this field.
  union StopOrLine {
    StopImpact s;
    uint32_t   lineid;
  };
  StopOrLine stopimpact_;

  // Hierarchy transitions and shortcut information
  uint32_t localedgeidx_  : 7;  // Index of the edge on the local level
  uint32_t opp_local_idx_ : 7;  // Opposing local edge index (for costing
                                // and Uturn detection)
  uint32_t shortcut_      : 7;  // Shortcut edge (mask)
  uint32_t superseded_    : 7;  // Edge is superseded by a shortcut (mask)
  uint32_t trans_up_      : 1;  // Edge represents a transition up one
                                // level in the hierarchy
  uint32_t trans_down_    : 1;  // Transition down one level
  uint32_t is_shortcut_   : 1;  // True if this edge is a shortcut.
  uint32_t spare2_        : 1;
};

}
}

#endif  // VALHALLA_BALDR_DIRECTEDEDGE_H_
