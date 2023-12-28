#ifndef VALHALLA_BALDR_DIRECTEDEDGE_H_
#define VALHALLA_BALDR_DIRECTEDEDGE_H_

#include <cstdint>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/json.h>
#include <valhalla/baldr/turn.h>

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
  GraphId endnode() const {
    return GraphId(endnode_);
  }

  /**
   * Set the end node of this directed edge.
   * @param  endnode  End node of the directed link.
   */
  void set_endnode(const GraphId& endnode);

  /**
   * Gets the free flow speed in KPH.
   * @return  Returns the free flow speed in KPH.
   */
  uint32_t free_flow_speed() const {
    return free_flow_speed_;
  }

  /**
   * Sets the free flow speed in KPH.
   * @param free flow Speed in KPH.
   */
  void set_free_flow_speed(const uint32_t speed);

  /**
   * Gets the constrained flow speed in KPH.
   * @return  Returns the constrained flow speed in KPH.
   */
  uint32_t constrained_flow_speed() const {
    return constrained_flow_speed_;
  }

  /**
   * Sets the constrained flow speed in KPH.
   * @param constrained flow Speed in KPH.
   */
  void set_constrained_flow_speed(const uint32_t speed);

  /**
   * Flag indicating the edge has predicted speed records.
   * @return  Returns true if this edge has predicted speed records.
   */
  bool has_predicted_speed() const {
    return has_predicted_speed_;
  }

  /**
   * Indicates whether the edge has either predicted, free or constrained flow speeds.
   * @return  Returns true if this edge has any flow speeds.
   */
  bool has_flow_speed() const {
    return free_flow_speed_ > 0 || constrained_flow_speed_ > 0 || has_predicted_speed_;
  }

  /**
   * Set the flag indicating the edge has turn lanes at the end of the edge.
   * @param lanes  True if this edge has turn lane records.
   */
  void set_turnlanes(const bool lanes) {
    turnlanes_ = lanes;
  }

  /**
   * Flag indicating the edge has turn lanes at the end of the edge.
   * @return  Returns true if this edge has turn lane records.
   */
  bool turnlanes() const {
    return turnlanes_;
  }

  /**
   * Set the flag indicating the edge has predicted speed records.
   * @param p  True if this edge has predicted speed records
   */
  void set_has_predicted_speed(const bool p);

  /**
   * Offset to the common edge data. The offset is from the start
   * of the common edge information  within a tile.
   * @return  Returns offset from the start of the edge info within a tile.
   */
  uint64_t edgeinfo_offset() const {
    return edgeinfo_offset_;
  }

  /**
   * Set the offset to the common edge info. The offset is from the start
   * of the common edge info within a tile.
   * @param  offset  Offset from the start of the edge info within a tile.
   */
  void set_edgeinfo_offset(const uint32_t offset);

  /**
   * General restriction or access condition (per mode) for this directed edge.
   * @return  Returns the restriction for the directed edge.
   */
  uint64_t access_restriction() const {
    return access_restriction_;
  }

  /**
   * Set the modes which have access restrictions on this edge.
   * @param  access  Modes with access restrictions.
   */
  void set_access_restriction(const uint32_t access);

  /**
   * Does this directed edge have signs?
   * @return  Returns true if the directed edge has signs,
   *          false if not.
   */
  bool sign() const {
    return sign_;
  }

  /**
   * Sets the sign flag.
   * @param  sign  True if this directed edge has signs, false if not.
   */
  void set_sign(const bool sign);

  /**
   * Does this directed edge have the lane connectivity?
   * @return  Returns true if the directed edge has the lane connectivity,
   *          false if not.
   */
  bool laneconnectivity() const {
    return lane_conn_;
  }

  /**
   * Sets the lane connectivity flag.
   * @param  lc  True if this directed edge has lane connectivity.
   */
  void set_laneconnectivity(const bool lc);

  /**
   * Gets the length of the edge in meters.
   * @return  Returns the length in meters.
   */
  uint32_t length() const {
    return length_;
  }

  /**
   * Sets the length of the edge in meters.
   * @param  length  Length of the edge in meters.
   * @param  should_error  Bool indicating whether or not to error or warn on length > kMaxEdgeLength
   *
   */
  void set_length(const uint32_t length, bool should_error = true);

  /**
   * Get the weighted grade factor
   * @return  Returns the weighted grade factor (0-15).
   *          where 0 is a -10% grade and 15 is 15%
   */
  uint32_t weighted_grade() const {
    return weighted_grade_;
  }

  /**
   * Sets the weighted grade factor (0-15) for the edge.
   * @param  factor  Weighted grade factor.
   */
  void set_weighted_grade(const uint32_t factor);

  /**
   * Gets the maximum upward slope. Uses 1 degree precision for slopes to 16
   * degrees and 4 degree precision afterwards (up to a max of 76 degrees).
   * @return  Returns the maximum upward slope (0 to 76 degrees).
   */
  int max_up_slope() const {
    return ((max_up_slope_ & 0x10) == 0) ? max_up_slope_ : 16 + ((max_up_slope_ & 0xf) * 4);
  }

  /**
   * Sets the maximum upward slope. If slope is negative, 0 is set.
   * @param  slope  Maximum upward slope (degrees).
   */
  void set_max_up_slope(const float slope);

  /**
   * Gets the maximum downward slope. Uses 1 degree precision for slopes to
   * -16 degrees, and 4 degree precision afterwards (up to a max of -76 degs).
   * @return  Returns the maximum downward slope (0 to -76 degrees).
   */
  int max_down_slope() const {
    return ((max_down_slope_ & 0x10) == 0) ? -static_cast<int>(max_down_slope_)
                                           : -static_cast<int>(16 + ((max_down_slope_ & 0xf) * 4));
  }

  /**
   * Sets the maximum downward slope. If slope is positive, 0 is set.
   * @param  slope  Maximum downward slope (degrees).
   */
  void set_max_down_slope(const float slope);

  /**
   * Get the road curvature factor.
   * @return  Returns the curvature factor (0-15).
   */
  uint32_t curvature() const {
    return curvature_;
  }

  /**
   * Sets the curvature factor (0-16) for the edge.
   * @param  factor  Curvature factor.
   */
  void set_curvature(const uint32_t factor);

  /**
   * Flag indicating the edge is a dead end (no other drivable
   * roads at the end node of this edge).
   * @return  Returns true if this edge is a dead end.
   */
  bool deadend() const {
    return deadend_;
  }

  /**
   * Set the flag indicating the edge is a dead end (no other drivable
   * roads at the end node of this edge).
   * @param d  True if this edge is a dead end.
   */
  void set_deadend(const bool d);

  /**
   * Does this edge have a toll or is it part of a toll road?
   * @return  Returns true if this edge is part of a toll road, false if not.
   */
  bool toll() const {
    return toll_;
  }

  /**
   * Sets the flag indicating this edge has a toll or is it part of
   * a toll road.
   * @param  toll  True if this edge is part of a toll road, false if not.
   */
  void set_toll(const bool toll);

  /**
   * Does this edge have a seasonal access (e.g., closed in the winter)?
   * @return  Returns true if this edge has seasonal access, false if not.
   */
  bool seasonal() const {
    return seasonal_;
  }

  /**
   * Sets the flag indicating this edge has seasonal access.
   * @param  seasonal  True if this edge has seasonal access, false if not.
   */
  void set_seasonal(const bool seasonal);

  /**
   * Is this edge part of a private or no through road that allows access
   * only if required to get to a destination?
   * @return  Returns true if the edge is destination only / private access.
   */
  bool destonly() const {
    return dest_only_;
  }

  /**
   * Sets the destination only (private) flag. This indicates the edge should
   * allow access only to locations that are destinations and not allow
   * "through" traffic
   * @param  destonly  True if the edge is private (allows access to
   *                   destination only), false if not.
   */
  void set_dest_only(const bool destonly);

  /**
   * Is this edge part of a private or no through road for HGV that allows access
   * only if required to get to a destination? If destonly() is true, this is true.
   * @return  Returns true if the edge is destination only / private access for HGV.
   */
  bool destonly_hgv() const {
    return dest_only_hgv_;
  }

  /**
   * Sets the destination only (private) flag for HGV. This indicates the
   * edge should allow access only to locations that are destinations and
   * not allow HGV "through" traffic
   * @param  destonly_hgv  True if the edge is private for HGV (allows access to
   *                       destination only), false if not.
   */
  void set_dest_only_hgv(const bool destonly_hgv);

  /**
   * Is this edge part of a tunnel?
   * @return  Returns true if this edge is part of a tunnel, false if not.
   */
  bool tunnel() const {
    return tunnel_;
  }

  /**
   * Sets the flag indicating this edge has is a tunnel of part of a tunnel.
   * @param  tunnel   True if the edge is a tunnel, false if not.
   */
  void set_tunnel(const bool tunnel);

  /**
   * Is this edge part of a bridge?
   * @return  Returns true if this edge is part of a bridge, false if not.
   */
  bool bridge() const {
    return bridge_;
  }

  /**
   * Sets the flag indicating this edge has is a bridge of part of a bridge.
   * @param  bridge   True if the edge is a bridge, false if not.
   */
  void set_bridge(const bool bridge);

  /**
   * Is this edge indoor?
   * @return  Returns true if this edge is indoor, false if not (outdoor).
   */
  bool indoor() const {
    return indoor_;
  }

  /**
   * Sets the flag indicating this edge is indoor.
   * @param  indoor   True if the edge is indoor, false if not (outdoor).
   */
  void set_indoor(const bool indoor);

  /**
   * Get the HOV type (see graphconstants.h).
   */
  HOVEdgeType hov_type() const {
    return static_cast<baldr::HOVEdgeType>(hov_type_);
  }

  /**
   * Sets the hov type (see baldr/graphconstants.h)
   * @param  hov_type   HOV type.
   */
  void set_hov_type(const HOVEdgeType hov_type);

  /**
   * Returns t/f if this edge is HOV only.
   * @return
   */
  bool is_hov_only() const;

  /**
   * Is this edge part of a roundabout?
   * @return  Returns true if this edge is part of a roundabout, false if not.
   */
  bool roundabout() const {
    return roundabout_;
  }

  /**
   * Sets the flag indicating the edge is part of a roundabout.
   * @param  roundabout  True if the edge is part of a roundabout, false if not.
   */
  void set_roundabout(const bool roundabout);

  /**
   * A traffic signal occurs at the end of this edge.
   * @return  Returns true if a traffic signal is present at the end of the
   *          directed edge.
   */
  bool traffic_signal() const {
    return traffic_signal_;
  }

  /**
   * Sets the flag indicating a traffic signal is present at the end of
   * this edge.
   * @param  signal  True if a traffic signal exists at the end of this edge,
   *                 false if not.
   */
  void set_traffic_signal(const bool signal);

  /**
   * A stop sign occurs at the end of this edge.
   * @return  Returns true if a stop sign is present at the end of the
   *          directed edge.
   */
  bool stop_sign() const {
    return stop_sign_;
  }

  /**
   * Sets the flag indicating a stop sign is present at the end of
   * this edge.
   * @param  sign  True if a stop sign exists at the end of this edge,
   *               false if not.
   */
  void set_stop_sign(const bool sign);

  /**
   * A yield/give way sign occurs at the end of this edge.
   * @return  Returns true if a yield/give way sign is present at the end of the
   *          directed edge.
   */
  bool yield_sign() const {
    return yield_sign_;
  }

  /**
   * Sets the flag indicating a yield/give way sign is present at the end of
   * this edge.
   * @param  sign  True if a yield/give way sign exists at the end of this edge,
   *               false if not.
   */
  void set_yield_sign(const bool sign);

  /**
   * Is this directed edge stored forward in edgeinfo (true) or
   * reverse (false).
   * @return  Returns true if stored forward, false if reverse.
   */
  bool forward() const {
    return forward_;
  }

  /**
   * Set the forward flag. Tells if this directed edge is stored forward
   * in edgeinfo (true) or reverse (false).
   * @param  forward  Forward flag.
   * */
  void set_forward(const bool forward);

  /**
   * Edge leads to a "no thru" region where there are no exits other than
   * the incoming edge. This flag is populated by processing the graph to
   * identify such edges. This is used to speed pedestrian routing.
   * @return  Returns true if the edge leads into a no thru region.
   */
  bool not_thru() const {
    return not_thru_;
  }

  /**
   * Set the not_thru flag. If an edge leads to a "no thru" region where
   * there are no exits other than the incoming edge. This flag is populated
   * by processing the graph toidentify such edges.
   * @param  not_thru True if the edge leads into a no thru region.
   */
  void set_not_thru(const bool not_thru);

  /**
   * Get the index of the opposing directed edge at the end node of this
   * directed edge. Can be used to find the start node of this directed edge.
   * @return  Returns the index of the opposing directed edge at the end node
   *          of this directed edge.
   */
  uint32_t opp_index() const {
    return opp_index_;
  }

  /**
   * Set the index of the opposing directed edge at the end node of this
   * directed edge.
   * @param  opp_index  Opposing directed edge index at the end node.
   * */
  void set_opp_index(const uint32_t opp_index);

  /**
   * Get the cycle lane type along this edge. If the edge's use is cycleway, footway or path,
   * then cyclelane holds information based on how separated cyclists are from pedestrians
   * @returns   Returns the type (if any) of bicycle lane along this edge.
   */
  CycleLane cyclelane() const {
    return static_cast<CycleLane>(cycle_lane_);
  }

  /**
   * Sets the type of cycle lane (if any) present on this edge.
   * If edge use is cycleway, footway or path, then cyclelane should represent
   * segregation from pedestrians
   * @param  cyclelane   Type of cycle lane.
   */
  void set_cyclelane(const CycleLane cyclelane);

  /**
   * Get the bike network flag for this directed edge.
   * @return  Returns the bike network flag. This is true if the edge is part of any
   *          bike network. They type(s) of bike networks are stored in EdgeInfo.
   */
  bool bike_network() const {
    return bike_network_;
  }

  /**
   * Sets the bike network flag.
   * @param  bike_network  Bicycle network flag. True if edge is part of any bike network.
   */
  void set_bike_network(const bool bike_network);

  /**
   * Get the truck route flag for this directed edge.
   * @return  Returns true if part of a truck network/route, false otherwise.
   */
  bool truck_route() const {
    return truck_route_;
  }

  /**
   * Set the truck route flag for this directed edge.
   * @param  truck_route  Truck route flag.
   */
  void set_truck_route(const bool truck_route);

  /**
   * Get the number of lanes for this directed edge.
   * @return  Returns the number of lanes for this directed edge.
   */
  uint32_t lanecount() const {
    return lanecount_;
  }

  /**
   * Sets the number of lanes
   * @param  lanecount  Number of lanes
   */
  void set_lanecount(const uint32_t lanecount);

  /**
   * Simple turn restrictions from the end of this directed edge.
   * These are turn restrictions from one edge to another that apply to
   * all vehicles, at all times.
   * @return  Returns a bit mask that indicates the local edge indexes
   *          of outbound directed edges that are restricted.
   */
  uint32_t restrictions() const {
    return restrictions_;
  }

  /**
   * Set simple turn restrictions from the end of this directed edge.
   * These are turn restrictions from one edge to another that apply to
   * all vehicles, at all times.
   * @param  mask A bit mask that indicates the local edge indexes
   *          of outbound directed edges that are restricted.
   */
  void set_restrictions(const uint32_t mask);

  /**
   * Get the specialized use of this edge.
   * @return  Returns the use type of this edge.
   */
  Use use() const {
    return static_cast<Use>(use_);
  }

  /**
   * Evaluates a basic set of conditions to determine if this directed edge is a valid potential
   * member of a shortcut. This is used while forming and resolving shortcuts.
   * @return true if the edge is not a shortcut, not related to transit and not under construction
   */
  bool can_form_shortcut() const {
    return !is_shortcut() && !bss_connection() && use() != Use::kTransitConnection &&
           use() != Use::kEgressConnection && use() != Use::kPlatformConnection &&
           use() != Use::kConstruction;
  }

  /**
   * Sets the specialized use type of this edge.
   * @param  use  Use of this edge.
   */
  void set_use(const Use use);

  /**
   * Is the edge a road (includes generic service roads)
   * @return Returns true if the edge is a road use or service road.
   */
  bool is_road() const {
    return use() == Use::kRoad || use() == Use::kServiceRoad;
  }

  /**
   * Is this edge a transit line (bus or rail)?
   * @return  Returns true if this edge is a transit line.
   */
  bool IsTransitLine() const {
    return (use() == Use::kRail || use() == Use::kBus);
  }

  /**
   * Get the speed type (see graphconstants.h)
   * @return  Returns the speed type.
   */
  SpeedType speed_type() const {
    return static_cast<SpeedType>(speed_type_);
  }

  /**
   * Set the speed type (see graphconstants.h)
   * @param  speed_type  Speed type.
   */
  void set_speed_type(const SpeedType speed_type);

  /**
   * Get the country crossing flag.
   * @return  Returns true if the edge crosses into a new country.
   */
  bool ctry_crossing() const {
    return ctry_crossing_;
  }

  /**
   * Set the country crossing flag.
   * @param  crossing  True if this edge crosses into a new country.
   */
  void set_ctry_crossing(const bool crossing);

  /**
   * Get the access modes in the forward direction (bit field).
   * @return  Returns the access modes in the forward direction.
   */
  uint32_t forwardaccess() const {
    return forwardaccess_;
  }

  /**
   * Set the access modes in the forward direction (bit field).
   * @param  modes  Allowed access modes in the forward direction.
   */
  void set_forwardaccess(const uint32_t modes);

  /**
   * Set all forward access modes to true (used for transition edges)
   */
  void set_all_forward_access();

  /**
   * Get the access modes in the reverse direction (bit field).
   * @return  Returns the access modes in the reverse direction.
   */
  uint32_t reverseaccess() const {
    return reverseaccess_;
  }

  /**
   * Set the access modes in the reverse direction (bit field).
   * @param  modes  Allowed access modes in the reverse direction.
   */
  void set_reverseaccess(const uint32_t modes);

  /**
   * Gets the average speed in KPH.
   * @return  Returns the speed in KPH.
   */
  uint32_t speed() const {
    return speed_;
  }

  /**
   * Sets the average speed in KPH.
   * @param  speed  Speed in KPH.
   */
  void set_speed(const uint32_t speed);

  /**
   * Gets the truck speed in KPH.
   * @return  Returns the truck speed in KPH.
   */
  uint32_t truck_speed() const {
    return truck_speed_;
  }

  /**
   * Sets the truck speed in KPH.
   * @param speed Speed in KPH.
   */
  void set_truck_speed(const uint32_t speed);

  /**
   * Get the classification (importance) of the road/path.
   * @return  Returns road classification / importance.
   */
  RoadClass classification() const {
    return static_cast<RoadClass>(classification_);
  }

  /**
   * Sets the classification (importance) of this edge.
   * @param  roadclass  Road class.
   */
  void set_classification(const RoadClass roadclass);

  /**
   * Gets the sac scale. Shows if edge is meant for hiking, and if so how difficult
   * of a hike it is.
   * @return  sac_scale
   */
  SacScale sac_scale() const {
    return static_cast<SacScale>(sac_scale_);
  }

  /**
   * Sets the sac scale. Shows if edge is meant for hiking, and if so how difficult
   * of a hike it is.
   * @param  sac_scale SacScale type
   */
  void set_sac_scale(const SacScale sac_scale);

  /**
   * Are names consistent with the from edge (local edge index at the start node).
   * @param idx  Local edge index at the start.
   */
  bool name_consistency(const uint32_t idx) const {
    return name_consistency_ & (1 << idx);
  }

  /**
   * Set the name consistency given the other edge's local index. This is limited
   * to the first 8 local edge indexes.
   * @param  from  Local index of the from edge.
   * @param  c     Are names consistent between the 2 edges?
   */
  void set_name_consistency(const uint32_t from, const bool c);

  /**
   * Is this edge unpaved or bad surface?
   */
  bool unpaved() const {
    return (surface() >= Surface::kCompacted);
  }

  /**
   * Get the surface type (see graphconstants.h). This is a general indication
   * of smoothness.
   */
  Surface surface() const {
    return static_cast<Surface>(surface_);
  }

  /**
   * Sets the surface type (see baldr/graphconstants.h). This is a general
   * indication of smoothness.
   * @param  surface   Surface type.
   */
  void set_surface(const Surface surface);

  /**
   * Is this edge a link/ramp?
   */
  bool link() const {
    return link_;
  }

  /**
   * Sets the link flag indicating the edge is part of a link or connection
   * (ramp or turn channel).
   * @param  link  True if the edge is part of a link.
   */
  void set_link(const bool link);

  /**
   * Gets the intersection internal flag. This indicates the edge is "internal"
   * to an intersection. This is derived from OSM based on geometry of an
   * of nearby edges and is used for routing behavior on doubly digitized
   * intersections.
   * @return  Returns true if the edge is internal to an intersection.
   */
  bool internal() const {
    return internal_;
  }

  /**
   * Sets the intersection internal flag indicating the edge is "internal"
   * to an intersection. This is derived from OSM based on geometry of an
   * of nearby edges and is used for routing behavior on doubly digitized
   * intersections.
   * @param  internal  True if the edge is internal to an intersection.
   */
  void set_internal(const bool internal);

  /**
   * Complex restriction (per mode) for this directed edge at the start.
   * @return  Returns the starting mode for this complex restriction for the
   *          directed edge.
   */
  uint32_t start_restriction() const {
    return start_restriction_;
  }

  /**
   * Set the modes which have a complex restriction starting on this edge.
   * @param  modes  Modes with access restrictions.
   */
  void set_start_restriction(const uint32_t modes);

  /**
   * Complex restriction (per mode) for this directed edge at the end.
   * @return  Returns the ending mode for this complex restriction for the
   *          directed edge.
   */
  uint32_t end_restriction() const {
    return end_restriction_;
  }

  /**
   * Set the modes which have a complex restriction ending on this edge.
   * @param  modes  Modes with access restrictions.
   */
  void set_end_restriction(const uint32_t modes);

  /**
   * Is this edge part of a complex restriction?
   * @return Returns true if this edge is part of a complex restriction.
   */
  bool part_of_complex_restriction() const {
    return complex_restriction_;
  }

  /**
   * Sets the part of complex restriction flag indicating the edge is part
   * of a complex restriction or really a via
   * @param  part_of  true if the edge is part of a complex restriction.
   */
  void complex_restriction(const bool part_of);

  /**
   * Get if edge has a shoulder
   * @return  Returns if edge has a shoulder
   */
  bool shoulder() const {
    return shoulder_;
  }

  /**
   * Set if edge has a shoulder
   * @param  shoulder  True if edge has shoulder
   */
  void set_shoulder(const bool shoulder);

  /**
   * Get if cyclists should dismount their bikes along this edge
   * @return  Returns true if edge is a dismount edge, false if it is not.
   */
  bool dismount() const {
    return dismount_;
  }

  /**
   * Set if cyclists should dismount their bikes along this edge
   * @param  dismount  true if the edge is a dismount edge, false if not.
   */
  void set_dismount(const bool dismount);

  /**
   * Get if a sidepath for bicycling should be preferred instead of this edge
   * @return  Returns if a sidepath should be preferred for cycling
   */
  bool use_sidepath() const {
    return use_sidepath_;
  }

  /**
   * Set if a sidepath for bicycling should be preferred instead of this edge
   * @param  use_sidepath  true if sidepath should be preferred for cycling over this edge
   */
  void set_use_sidepath(const bool use_sidepath);

  /**
   * Get the density along the edges.
   * @return  Returns relative density along the edge.
   */
  uint32_t density() const {
    return density_;
  }

  /**
   * Set the density along the edges.
   * @param  density  Relative density along the edge.
   */
  void set_density(const uint32_t density);

  /**
   * Is this edge named?
   * @return  Returns true if the edge is named, false if unnamed.
   */
  bool named() const {
    return named_;
  }

  /**
   * Sets the named flag.
   * @param  named  true if the edge has names, false if unnamed.
   */
  void set_named(const bool named);

  /**
   * Is there a sidewalk to the left of this directed edge?
   * @return  Returns true if there is a sidewalk to the left of this edge.
   */
  bool sidewalk_left() const {
    return sidewalk_left_;
  }

  /**
   * Set the flag for a sidewalk to the left of this directed edge.
   * @param  sidewalk True if a sidewalk is on the left of this directed edge.
   */
  void set_sidewalk_left(const bool sidewalk);

  /**
   * Is there a sidewalk to the right of this directed edge?
   * @return  Returns true if there is a sidewalk to the right of this edge.
   */
  bool sidewalk_right() const {
    return sidewalk_right_;
  }

  /**
   * Set the flag for a sidewalk to the right of this directed edge.
   * @param  sidewalk True if a sidewalk is on the right of this directed edge.
   */
  void set_sidewalk_right(const bool sidewalk);

  /**
   * Gets the turn type given the prior edge's local index
   * (index of the inbound edge).
   * @param  localidx  Local index at the node of the inbound edge.
   * @return  Returns the turn type (see turn.h)
   */
  Turn::Type turntype(const uint32_t localidx) const {
    // Turn type is 3 bits per index
    uint32_t shift = localidx * 3;
    return static_cast<Turn::Type>(((turntype_ & (7 << shift))) >> shift);
  }

  /**
   * Sets the turn type given the prior edge's local index
   * (index of the inbound edge).
   * @param  localidx  Local index at the node of the inbound edge.
   * @param  turntype  Turn type
   */
  void set_turntype(const uint32_t localidx, const Turn::Type turntype);

  /**
   * Is there an edge to the left, in between the from edge and this edge.
   * @param localidx  Local index at the node of the inbound edge.
   * @return  Returns true if there is an edge to the left, false if not.
   */
  bool edge_to_left(const uint32_t localidx) const {
    return (edge_to_left_ & (1 << localidx));
  }

  /**
   * Set the flag indicating there is an edge to the left, in between
   * the from edge and this edge.
   * @param  localidx  Local index at the node of the inbound edge.
   * @param  left      True if there is an edge to the left, false if not.
   */
  void set_edge_to_left(const uint32_t localidx, const bool left);

  /**
   * Get the stop impact when transitioning from the prior edge (given
   * by the local index of the corresponding inbound edge at the node).
   * @param  localidx  Local index at the node of the inbound edge.
   * @return  Returns the relative stop impact from low (0) to high (7).
   */
  uint32_t stopimpact(const uint32_t localidx) const {
    // Stop impact is 3 bits per index
    uint32_t shift = localidx * 3;
    return (stopimpact_.s.stopimpact & (7 << shift)) >> shift;
  }

  /**
   * Set the stop impact when transitioning from the prior edge (given
   * by the local index of the corresponding inbound edge at the node).
   * @param  localidx    Local index at the node of the inbound edge.
   * @param  stopimpact  Relative stop impact from low (0) to high (7).
   */
  void set_stopimpact(const uint32_t localidx, const uint32_t stopimpact);

  /**
   * Get the transit line Id (for departure lookups along an edge)
   * @return  Returns the transit line Id.
   */
  uint32_t lineid() const {
    return stopimpact_.lineid;
  }

  /**
   * Set the unique transit line Id.
   * @param  lineid  Line Id indicating a unique stop-pair and route.
   */
  void set_lineid(const uint32_t lineid);

  /**
   * Is there an edge to the right, in between the from edge and this edge.
   * @param localidx  Local index at the node of the inbound edge.
   * @return  Returns true if there is an edge to the right, false if not.
   */
  bool edge_to_right(const uint32_t localidx) const {
    return (stopimpact_.s.edge_to_right & (1 << localidx));
  }

  /**
   * Set the flag indicating there is an edge to the right, in between
   * the from edge and this edge.
   * @param  localidx  Local index at the node of the inbound edge.
   * @param  right     True if there is an edge to the right, false if not.
   */
  void set_edge_to_right(const uint32_t localidx, const bool right);

  /**
   * Get the index of the directed edge on the local level of the graph
   * hierarchy. This is used for turn restrictions so the edges can be
   * identified on the different levels.
   * @return  Returns the index of the edge on the local level.
   */
  uint32_t localedgeidx() const {
    return localedgeidx_;
  }

  /** Set the index of the directed edge on the local level of the graph
   * hierarchy. This is used for turn restrictions so the edges can be
   * identified on the different levels.
   * @param idx The index of the edge on the local level.
   */
  void set_localedgeidx(const uint32_t idx);

  /**
   * Get the index of the opposing directed edge on the local hierarchy level
   * at the end node of this directed edge. Only stored for the first 8 edges
   * so it can be used for edge transition costing.
   * @return  Returns the index of the opposing directed edge on the local
   *          hierarchy level at end node of this directed edge.
   */
  uint32_t opp_local_idx() const {
    return opp_local_idx_;
  }

  /**
   * Set the index of the opposing directed edge on the local hierarchy level
   * at the end node of this directed edge. Only stored for the first 8 edges
   * so it can be used for edge transition costing.
   * @param localidx  The index of the opposing directed edge on the local
   *                  hierarchy level at end node of this directed edge.
   */
  void set_opp_local_idx(const uint32_t localidx);

  /**
   * Indicates the mask of the superseded edge bypassed by a shortcut.
   * Shortcuts bypass nodes that only connect to lower levels in the hierarchy
   * (other than the 1-2 higher level edges that superseded by the shortcut).
   * @return  Returns the shortcut mask of the matching superseded edge
   *          outbound from the node. 0 indicates the edge is not a shortcut.
   */
  uint32_t shortcut() const {
    return shortcut_;
  }

  /**
   * Set the mask for whether this edge represents a shortcut between 2 nodes.
   * Shortcuts bypass nodes that only connect to lower levels in the hierarchy
   * (other than the 1-2 higher level edges that superseded by the shortcut).
   * @param  shortcut  Mask indicating the edge that is superseded by
   *                   the shortcut. 0 if not a shortcut.
   */
  void set_shortcut(const uint32_t shortcut);

  /**
   * Mask indicating the shortcut that supersedes this directed edge.
   * Superseded edges can be skipped unless downward transitions are allowed.
   * @return  Returns the mask of the matching shortcut edge that supersedes this
   *          directed edge outbound from the node. 0 indicates the edge is not
   *          superseded by a shortcut edge.
   */
  uint32_t superseded() const {
    return superseded_;
  }

#ifdef _WIN32
  // TODO: Workaround for missing strings.h on Windows. Replace with platform independent
  //       std::countr_zero in C++20 (see https://en.cppreference.com/w/cpp/numeric/countr_zero).
  int ffs(int mask) const {
    if (0 == mask)
      return 0;

    int idx;
    for (idx = 1; !(mask & 1); ++idx)
      mask >>= 1;
    return idx;
  }
#endif

  /**
   * Unlike superseded(), this does not return the raw mask but the shortcut index
   * that was originally passed to set_superseded().
   * @return  Returns the index of the set bit in the superseded mask.
   */
  uint32_t superseded_idx() const {
    return ffs(superseded_);
  }

  /**
   * Set the mask for whether this edge is superseded by a shortcut edge.
   * Superseded edges can be skipped unless downward transitions are allowed.
   * @param  superseded  Mask that matches the shortcut that supersedes this
   *                     directed edge. 0 if not superseded by a shortcut.
   */
  void set_superseded(const uint32_t superseded);

  /**
   * Is this edge a shortcut edge. If there are more than kMaxShortcutsFromNode
   * shortcuts no mask is set but this flag is set to true.
   * @return  Returns true if this edge is a shortcut.
   */
  bool is_shortcut() const {
    return is_shortcut_;
  }

  /**
   * Does this directed edge end in a different tile.
   * @return  Returns true if the end node of this directed edge
   *          is in a different tile.
   */
  bool leaves_tile() const {
    return leaves_tile_;
  }

  /**
   * Set the flag indicating whether the end node of this directed edge is in
   * a different tile
   * @param  leaves_tile  True if the end node of this edge is in a different
   *                      tile than the start node (and the directed edge
   *                      itself).
   */
  void set_leaves_tile(const bool leaves_tile);

  /**
   * Set the flag indicating whether this edge is a bike share station's connection
   * @param  bss_connection  True if the edge is a bike share station's connection
   */
  void set_bss_connection(const bool bss_connection);

  /**
   * If this edge is a bike share station's connection?
   * @return  Returns true if this edge is a bike share station's connection
   */
  bool bss_connection() const {
    return bss_connection_;
  }

  /**
   * Set the flag indicating whether the edge is lit
   * @param lit the edge's lit state
   */
  void set_lit(const bool lit);

  /**
   * Is the edge lit?
   * @return Returns the edge's lit state
   */
  bool lit() const {
    return lit_;
  }

  /**
   * Create a json object representing this edge
   * @return  Returns the json object
   */
  json::MapPtr json() const;

protected:
  // 1st 8-byte word
  uint64_t endnode_ : 46;      // End node of the directed edge
  uint64_t restrictions_ : 8;  // Restrictions - mask of local edge indexes at the end node
  uint64_t opp_index_ : 7;     // Opposing directed edge index
  uint64_t forward_ : 1;       // Is the edge info forward or reverse
  uint64_t leaves_tile_ : 1;   // Does directed edge end in a different tile?
  uint64_t ctry_crossing_ : 1; // Does the edge cross into new country

  // 2nd 8 byte word
  uint64_t edgeinfo_offset_ : 25;    // Offset to edge data
  uint64_t access_restriction_ : 12; // General restriction or access condition (per mode)
  uint64_t start_restriction_ : 12;  // Complex restriction (per mode) starts on this directed edge
  uint64_t end_restriction_ : 12;    // Complex restriction (per mode) ends on this directed edge
  uint64_t complex_restriction_ : 1; // Edge is part of a complex restriction
  uint64_t dest_only_ : 1;           // Access allowed to destination only (e.g., private)
  uint64_t not_thru_ : 1;            // Edge leads to "no-through" region

  // 3rd 8-byte word. Note: speed values above 250 for special cases (closures, construction)
  uint64_t speed_ : 8;                  // Speed (kph)
  uint64_t free_flow_speed_ : 8;        // Speed when there is no traffic(kph)
  uint64_t constrained_flow_speed_ : 8; // Speed when there is traffic(kph)
  uint64_t truck_speed_ : 8;            // Truck speed (kph)
  uint64_t name_consistency_ : 8;       // Name consistency at start node with other local edges
  uint64_t use_ : 6;                    // Specific use types
  uint64_t lanecount_ : 4;              // Number of lanes
  uint64_t density_ : 4;                // Relative road density along the edge
  uint64_t classification_ : 3;         // Classification/importance of the road/path
  uint64_t surface_ : 3;                // Representation of smoothness
  uint64_t toll_ : 1;                   // Edge is part of a toll road
  uint64_t roundabout_ : 1;             // Edge is part of a roundabout
  uint64_t truck_route_ : 1;            // Edge that is part of a truck route/network
  uint64_t has_predicted_speed_ : 1;    // Does this edge have a predicted speed records?

  // 4th 8-byte word
  uint64_t forwardaccess_ : 12; // Access (bit mask) in forward direction (see graphconstants.h)
  uint64_t reverseaccess_ : 12; // Access (bit mask) in reverse direction (see graphconstants.h)
  uint64_t max_up_slope_ : 5;   // Maximum upward slope
  uint64_t max_down_slope_ : 5; // Maximum downward slope
  uint64_t sac_scale_ : 3;      // Is this edge for hiking and if so how difficult is the hike?
  uint64_t cycle_lane_ : 2;     // Does this edge have bicycle lanes?
  uint64_t bike_network_ : 1;   // Edge that is part of a bicycle network
  uint64_t use_sidepath_ : 1;   // Is there a cycling path to the side that should be preferred?
  uint64_t dismount_ : 1;       // Do you need to dismount when biking on this edge?
  uint64_t sidewalk_left_ : 1;  // Sidewalk to the left of the edge
  uint64_t sidewalk_right_ : 1; // Sidewalk to the right of the edge
  uint64_t shoulder_ : 1;       // Does the edge have a shoulder?
  uint64_t lane_conn_ : 1;      // 1 if has lane connectivity, 0 otherwise
  uint64_t turnlanes_ : 1;      // Does this edge have turn lanes (end of edge)
  uint64_t sign_ : 1;           // Exit signs exist for this edge
  uint64_t internal_ : 1;       // Edge that is internal to an intersection
  uint64_t tunnel_ : 1;         // Is this edge part of a tunnel
  uint64_t bridge_ : 1;         // Is this edge part of a bridge?
  uint64_t traffic_signal_ : 1; // Traffic signal at end of the directed edge
  uint64_t seasonal_ : 1;       // Seasonal access (ex. no access in winter)
  uint64_t deadend_ : 1;        // Leads to a dead-end (no other drivable roads) TODO
  uint64_t bss_connection_ : 1; // Does this lead to(come out from) a bike share station?
  uint64_t stop_sign_ : 1;      // Stop sign at end of the directed edge
  uint64_t yield_sign_ : 1;     // Yield/give way sign at end of the directed edge
  uint64_t hov_type_ : 1;       // if (is_hov_only()==true), this means (HOV2=0, HOV3=1)
  uint64_t indoor_ : 1;         // Is this edge indoor
  uint64_t lit_ : 1;            // Is the edge lit?
  uint64_t dest_only_hgv_ : 1;  // destonly for HGV specifically
  uint64_t spare4_ : 3;

  // 5th 8-byte word
  uint64_t turntype_ : 24;      // Turn type (see graphconstants.h)
  uint64_t edge_to_left_ : 8;   // Is there an edge to the left (between
                                // the "from edge" and this edge)
  uint64_t length_ : 24;        // Length in meters
  uint64_t weighted_grade_ : 4; // Weighted estimate of grade
  uint64_t curvature_ : 4;      // Curvature factor

  // Stop impact among edges
  struct StopImpact {
    uint32_t stopimpact : 24;   // Stop impact between edges
    uint32_t edge_to_right : 8; // Is there an edge to the right (between
                                // "from edge" and this edge)
  };

  // Store either the stop impact or the transit line identifier. Since
  // transit lines are schedule based they have no need for edge transition
  // logic so we can safely share this field.
  union StopOrLine {
    StopImpact s;
    uint32_t lineid;
  };

  // 6th 8-byte word (this union plus the next uint32_t bitfield)
  StopOrLine stopimpact_;

  // Local edge index, opposing local index, shortcut info
  uint32_t localedgeidx_ : 7;  // Index of the edge on the local level
  uint32_t opp_local_idx_ : 7; // Opposing local edge index (for costing and Uturn detection)
  uint32_t shortcut_ : 7;      // Shortcut edge (mask)
  uint32_t superseded_ : 7;    // Edge is superseded by a shortcut (mask)
  uint32_t is_shortcut_ : 1;   // True if this edge is a shortcut
  uint32_t speed_type_ : 1;    // Speed type (used in setting default speeds)
  uint32_t named_ : 1;         // 1 if this edge has names, 0 if unnamed
  uint32_t link_ : 1;          // *link tag - Ramp or turn channel. Used in costing.
};

/**
 * Extended directed edge attribution. This structure provides the ability to add extra
 * attribution per directed edge without breaking backward compatibility. For now this structure
 * is unused.
 */
class DirectedEdgeExt {

protected:
  uint64_t spare0_ : 64;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_DIRECTEDEDGE_H_
