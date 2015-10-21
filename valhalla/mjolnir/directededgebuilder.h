#ifndef VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
#define VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/mjolnir/osmway.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

/**
 * Directed edge within the graph.
 * @author  David W. Nesbitt
 */
class DirectedEdgeBuilder : public baldr::DirectedEdge {
 public:
  /**
   * Constructor
   */
  DirectedEdgeBuilder();

  /**
   * Constructor with arguments.
   * @param  way            OSM way info generated from parsing OSM tags with Lua.
   * @param  endnode        GraphId of the end node of this directed edge.
   * @param  length         Length in meters.
   * @param  speed          Speed in kph.
   * @param  truck_speed    Truck speed in kph.
   * @param  use            Use of the edge.
   * @param  rc             Road class / importance
   * @param  localidx       Index of the edge (from the node) on the local level
   * @param  restrictions   Mask of simple turn restrictions at the end node
   *                        of this directed edge.
   * @param  bike_network   Mask of bike_networks from relations.
   */
  DirectedEdgeBuilder(const OSMWay& way, const baldr::GraphId& endnode,
                      const bool forward, const uint32_t length,
                      const uint32_t speed, const uint32_t truck_speed,
                      const baldr::Use use, const baldr::RoadClass rc,
                      const uint32_t localidx, const bool signal,
                      const uint32_t restrictions, const uint32_t bike_network);

  /**
   * Set the end node of this directed edge.
   * @param  endnode  End node of the directed link.
   */
  void set_endnode(const baldr::GraphId& endnode);

  /**
   * Set the offset to the common edge info. The offset is from the start
   * of the common edge info within a tile.
   * @param  offset  Offset from the start of the edge info within a tile.
   */
  void set_edgeinfo_offset(const uint32_t offset);

  /**
   * Set the restriction or access condition per mode
   * @param  access  General access conditions
   */
  void set_access(const uint32_t access);

  /**
   * Sets the exit sign flag.
   * @param  exit  True if this directed edge has exit signs, false if not.
   */
  void set_exitsign(const bool exit);

  /**
   * Sets the length of the edge in meters.
   * @param  length  Length of the edge in meters.
   */
  void set_length(const uint32_t length);

  /**
   * Sets the weighted grade factor (0-15) for the edge.
   * @param  factor  Weighted grade factor.
   */
  void set_weighted_grade(const uint32_t factor);

  /**
   * Sets the curvature factor (0-16) for the edge. TODO.
   * @param  factor  Curvature factor.
   */
  void set_curvature(const uint32_t factor);

  /**
   * Set the flag indicating driving is on the right hand side of the road
   * along this edge?
   * @param rsd  True if this edge uses right-side driving, false if
   *             left-side driving.
   */
  void set_drive_on_right(const bool rsd);

  /**
   * Sets the flag indicating this edge is a ferry (or part of a ferry).
   * @param  ferry  True if this edge is a ferry, false if not.
   */
  void set_ferry(const bool ferry);

  /**
   * Sets the flag indicating this edge is a rail ferry (or part of a
   * rail ferry). Example is the EuroTunnel (Channel Tunnel).
   * @param  ferry  True if this edge is a rail ferry, false if not.
   */
  void set_railferry(const bool railferry);

  /**
   * Sets the flag indicating this edge has a toll or is it part of
   * a toll road.
   * @param  toll  True if this edge is part of a toll road, false if not.
   */
  void set_toll(const bool toll);

  /**
   * Sets the flag indicating this edge has seasonal access.
   * @param  seasonal  True if this edge has seasonal access, false if not.
   */
  void set_seasonal(const bool seasonal);

  /**
   * Sets the destination only (private) flag. This indicates the edge should
   * allow access only to locations that are destinations and not allow
   * "through" traffic
   * @param  destonly  True if the edge is private (allows access to
   *                   destination only), false if not.
   */
  void set_dest_only(const bool destonly);

  /**
   * Sets the flag indicating this edge has is a tunnel of part of a tunnel.
   * @param  tunnel   True if the edge is a tunnel, false if not.
   */
  void set_tunnel(const bool tunnel);

  /**
   * Sets the flag indicating this edge has is a bridge of part of a bridge.
   * @param  bridge   True if the edge is a bridge, false if not.
   */
  void set_bridge(const bool bridge);

  /**
   * Sets the flag indicating the edge is part of a roundabout.
   * @param  roundabout  True if the edge is part of a roundabout, false if not.
   */
  void set_roundabout(const bool roundabout);

  /**
   * Sets the flag indicating the edge is unreachable by driving. This can
   * happen if a driveable edge is surrounded by pedestrian only edges (e.g.
   * in a city center) or is not properly connected to other edges.
   * @param  unreachable  True if the edge is unreachable by driving,
   *                      false if not.
   */
  void set_unreachable(const bool unreachable);

  /**
   * Sets the flag indicating a traffic signal is present at the end of
   * this edge.
   * @param  signal  True if a traffic signal exists at the end of this edge,
   *                 false if not.
   */
  void set_traffic_signal(const bool signal);

  /**
   * Set the forward flag. Tells if this directed edge is stored forward
   * in edgeinfo (true) or reverse (false).
   * @param  forward  Forward flag.
   * */
  void set_forward(const bool forward);

  /**
   * Set the not_thru flag. If an edge leads to a "no thru" region where
   * there are no exits other than the incoming edge. This flag is populated
   * by processing the graph toidentify such edges.
   * @param  not_thru True if the edge leads into a no thru region.
   */
  void set_not_thru(const bool not_thru);

  /**
   * Set the index of the opposing directed edge at the end node of this
   * directed edge.
   * @param  opp_index  Opposing directed edge index at the end node.
   * */
  void set_opp_index(const uint32_t opp_index);

  /**
    * Sets the type of cycle lane (if any) present on this edge.
    * @param  cyclelane   Type of cycle lane.
    */
  void set_cyclelane(const CycleLane cyclelane);

  /**
   * Sets the bike network mask indicating which (if any) bicycle networks are
   * along this edge. See baldr/directededge.h for definitions.
   * @param  bikenetwork  Bicycle network mask.
  */
  void set_bike_network(const uint32_t bike_network);

  /**
   * Set the truck route flag for this directed edge.
   * @param  truck_route  Truck route flag.
   */
  void set_truck_route(const bool truck_route);

  /**
   * Sets the number of lanes
   * @param  lanecount  Number of lanes
   */
  void set_lanecount(const uint32_t lanecount);

  /**
   * Set simple turn restrictions from the end of this directed edge.
   * These are turn restrictions from one edge to another that apply to
   * all vehicles, at all times.
   * @param  mask A bit mask that indicates the local edge indexes
   *          of outbound directed edges that are restricted.
   */
  void set_restrictions(const uint32_t mask);

  /**
   * Sets the specialized use type of this edge.
   * @param  use  Use of this edge.
   */
  void set_use(const Use use);

  /**
   * Set the speed type (see graphconstants.h)
   * @param  speed_type  Speed type.
   */
  void set_speed_type(const SpeedType speed_type);

  /**
   * Set the country crossing flag.
   * @param  crossing  True if this edge crosses into a new country.
   */
  void set_ctry_crossing(const bool crossing);

  /**
   * Set all forward access modes to true (used for transition edges)
   */
  void set_all_forward_access();

  /**
   * Sets the car access of the edge in the specified direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  car      Is car access allowed?
   */
  void set_caraccess(const bool forward, const bool car);

  /**
   * Sets the taxi access of the edge in the specified direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  taxi     Is taxi access allowed?
   */
  void set_taxiaccess(const bool forward, const bool taxi);

  /**
   * Sets the truck access of the edge in the specified direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  truck    Is truck access allowed?
   */
  void set_truckaccess(const bool forward, const bool truck);

  /**
   * Sets the pedestrian access of the edge in the specified direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  pedestrian  Is pedestrian access allowed?
   */
  void set_pedestrianaccess(const bool forward, const bool pedestrian);

  /**
   * Sets the bicycle access of the edge in the specified direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  bicycle  Is bicycle access allowed?
   */
  void set_bicycleaccess(const bool forward, const bool bicycle);

  /**
   * Sets the emergency access of the edge in the specified direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  emergency  Is emergency access allowed?
   */
  void set_emergencyaccess(const bool forward, const bool emergency);

  /**
   * Sets the bus access of the edge in the specified direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  bus    Is bus access allowed?
   */
  void set_busaccess(const bool forward, const bool bus);

  /**
   * Sets the high occupancy vehicle (HOV) access of the edge in the specified
   * direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  hov    Is HOV access allowed?
   */
  void set_hovaccess(const bool forward, const bool hov);

  /**
   * Sets the speed in KPH.
   * @param  speed  Speed in KPH.
  */
  void set_speed(const uint32_t speed);

  /**
   * Sets the truck speed in KPH.
   * @param  truck speed  Speed in KPH.
  */
  void set_truck_speed(const uint32_t speed);

  /**
   * Sets the classification (importance) of this edge.
   * @param  roadclass  Road class.
   */
  void set_classification(const RoadClass roadclass);

  /**
   * Sets the surface type (see baldr/graphconstants.h). This is a general
   * indication of smoothness.
   * @param  surface   Surface type.
   */
  void set_surface(const Surface surface);

  /**
   * Sets the link flag indicating the edge is part of a link or connection
   * (ramp or turn channel).
   * @param  link  True if the edge is part of a link.
   */
  void set_link(const bool link);

  /**
   * Sets the intersection internal flag indicating the edge is "internal"
   * to an intersection. This is derived from OSM based on geometry of an
   * of nearby edges and is used for routing behavior on doubly digitized
   * intersections.
   * @param  internal  True if the edge is internal to an intersection.
   */
  void set_internal(const bool internal);

  /**
   * Sets the turn type given the prior edge's local index
   * (index of the inbound edge).
   * @param  localidx  Local index at the node of the inbound edge.
   * @param  turntype  Turn type (see TODO)
   */
  void set_turntype(const uint32_t localidx, const Turn::Type turntype);

  /**
   * Set the flag indicating there is an edge to the left, in between
   * the from edge and this edge.
   * @param  localidx  Local index at the node of the inbound edge.
   * @param  left      True if there is an edge to the left, false if not.
   */
  void set_edge_to_left(const uint32_t localidx, const bool left);

  /**
   * Set the stop impact when transitioning from the prior edge (given
   * by the local index of the corresponding inbound edge at the node).
   * @param  localidx    Local index at the node of the inbound edge.
   * @param  stopimpact  Relative stop impact from low (0) to high (7).
   */
  void set_stopimpact(const uint32_t localidx, const uint32_t stopimpact);

  /**
   * Set the unique transit line Id.
   * @param  lineid  Line Id indicating a unique stop-pair and route.
   */
  void set_lineid(const uint32_t lineid);

  /**
   * Set the flag indicating there is an edge to the right, in between
   * the from edge and this edge.
   * @param  localidx  Local index at the node of the inbound edge.
   * @param  right     True if there is an edge to the right, false if not.
   */
  void set_edge_to_right(const uint32_t localidx, const bool right);

  /** Set the index of the directed edge on the local level of the graph
   * hierarchy. This is used for turn restrictions so the edges can be
   * identified on the different levels.
   * @param idx The index of the edge on the local level.
   */
  void set_localedgeidx(const uint32_t idx);

  /**
   * Set the index of the opposing directed edge on the local hierarchy level
   * at the end node of this directed edge. Only stored for the first 8 edges
   * so it can be used for edge transition costing.
   * @param localidx  The index of the opposing directed edge on the local
   *                  hierarchy level at end node of this directed edge.
   */
  void set_opp_local_idx(const uint32_t localidx);

  /**
   * Set the mask for whether this edge represents a shortcut between 2 nodes.
   * Shortcuts bypass nodes that only connect to lower levels in the hierarchy
   * (other than the 1-2 higher level edges that superseded by the shortcut).
   * @param  shortcut  Mask indicating the edge that is superseded by
   *                   the shortcut. 0 if not a shortcut.
   */
  void set_shortcut(const uint32_t shortcut);

  /**
   * Set the mask for whether this edge is superseded by a shortcut edge.
   * Superseded edges can be skipped unless downward transitions are allowed.
   * @param  superseded  Mask that matches the shortcut that supersedes this
   *                     directed edge. 0 if not superseded by a shortcut.
   */
  void set_superseded(const uint32_t superseded);

  /**
   * Set the flag for whether this edge represents a transition up one level
   * in the hierarchy. Transition edges move between nodes in different levels
   * of the hierarchy but have no length or other attribution. An upward
   * transition is a transition from a minor road hierarchy (local) to more
   * major (arterial).
   * @param  trans_up  True if the edge is a transition from a lower level
   *          to a higher (false if not).
   */
  void set_trans_up(const bool trans_up);

  /**
   * Set the flag for whether this edge represents a transition down one level
   * in the hierarchy. Transition edges move between nodes in different levels
   * of the hierarchy but have no length or other attribution. A downward
   * transition is a transition from a major road hierarchy (highway) to more
   * minor (arterial).
   * @param   trans_down  True if the edge is a transition from an upper level
   *          to a lower (false if not).
   */
  void set_trans_down(const bool trans_down);

};

}
}

#endif  // VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

