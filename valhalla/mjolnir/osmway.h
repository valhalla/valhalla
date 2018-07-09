#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMWAY_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMWAY_H

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/mjolnir/uniquenames.h>

namespace valhalla {
namespace mjolnir {

// OSM way
struct OSMWay {

  /**
   * Set way id.
   * @param   id  way id
   */
  void set_way_id(const uint64_t id) {
    osmwayid_ = id;
  }

  /**
   * Get the way id
   * @return  Returns way id.
   */
  uint64_t way_id() const {
    return osmwayid_;
  }

  /**
   * Set the number of nodes for this way.
   * @param count  Number of nodes in this way.
   */
  void set_node_count(const uint32_t count);

  /**
   * Get the number of nodes for this way.
   */
  uint32_t node_count() const {
    return nodecount_;
  }

  /**
   * Sets the speed
   * @param  speed   Speed in KPH.
   */
  void set_speed(const float speed);

  /**
   * Gets the speed in KPH.
   * @return  Returns speed.
   */
  uint8_t speed() const {
    return speed_;
  }

  /**
   * Sets the speed limit
   * @param  speed_limit   Speed limit in KPH.
   */
  void set_speed_limit(const float speed_limit);

  /**
   * Gets the speed limit in KPH.
   * @return  Returns speed_limit.
   */
  uint8_t speed_limit() const {
    return speed_limit_;
  }
  /**
   * Sets the backward speed
   * @param  backward_speed   Speed in KPH.
   */
  void set_backward_speed(const float backward_speed);

  /**
   * Gets the backward speed in KPH.
   * @return  Returns backward speed.
   */
  uint8_t backward_speed() const {
    return backward_speed_;
  }

  /**
   * Sets the forward speed
   * @param  forward_speed   Speed in KPH.
   */
  void set_forward_speed(const float forward_speed);

  /**
   * Gets the forward speed in KPH.
   * @return  Returns forward speed.
   */
  uint8_t forward_speed() const {
    return forward_speed_;
  }

  /**
   * Sets the truck speed
   * @param  speed   Truck speed in KPH.
   */
  void set_truck_speed(const float truck_speed);

  /**
   * Gets the truck speed in KPH.
   * @return  Returns truck speed.
   */
  uint8_t truck_speed() const {
    return truck_speed_;
  }

  /**
   * Sets the index for the ref
   * @param  idx  Index for the reference.
   */
  void set_ref_index(const uint32_t idx) {
    ref_index_ = idx;
  }

  /**
   * Get the ref index.
   * @return  Returns the index for the ref.
   */
  uint32_t ref_index() const {
    return ref_index_;
  }

  /**
   * Sets the index for int ret
   * @param  idx  Index for the international reference.
   */
  void set_int_ref_index(const uint32_t idx) {
    int_ref_index_ = idx;
  }

  /**
   * Get the int ref index.
   * @return  Returns the index for the int ref.
   */
  uint32_t int_ref_index() const {
    return int_ref_index_;
  }

  /**
   * Sets the index for name
   * @param  idx  Index for the name.
   */
  void set_name_index(const uint32_t idx) {
    name_index_ = idx;
  }

  /**
   * Get the name index.
   * @return  Returns the index for the name.
   */
  uint32_t name_index() const {
    return name_index_;
  }

  /**
   * Sets the index for name:en
   * @param  idx  Index for the English name.
   */
  void set_name_en_index(const uint32_t idx) {
    name_en_index_ = idx;
  }

  /**
   * Get the name:en index.
   * @return  Returns the index for the English name.
   */
  uint32_t name_en_index() const {
    return name_en_index_;
  }

  /**
   * Sets the index for alt name
   * @param  idx  Index for the alt name.
   */
  void set_alt_name_index(const uint32_t idx) {
    alt_name_index_ = idx;
  }

  /**
   * Get the alt name index.
   * @return  Returns the index for the alt name.
   */
  uint32_t alt_name_index() const {
    return alt_name_index_;
  }

  /**
   * Sets the index for official name
   * @param  idx  Index for the official name.
   */
  void set_official_name_index(const uint32_t idx) {
    official_name_index_ = idx;
  }

  /**
   * Get the official name index.
   * @return  Returns the index for the official name.
   */
  uint32_t official_name_index() const {
    return official_name_index_;
  }

  /**
   * Sets the index for forward turn lanes string.
   * @param  idx  Index for the forward turn lanes string.
   */
  void set_fwd_turn_lanes_index(const uint32_t idx) {
    fwd_turn_lanes_index_ = idx;
  }

  /**
   * Get the forward turn lanes string index.
   * @return  Returns the index for the forward turn lanes string.
   */
  uint32_t fwd_turn_lanes_index() const {
    return fwd_turn_lanes_index_;
  }

  /**
   * Sets the index for backward turn lanes string.
   * @param  idx  Index for the backward turn lanes string.
   */
  void set_bwd_turn_lanes_index(const uint32_t idx) {
    bwd_turn_lanes_index_ = idx;
  }

  /**
   * Get the backward turn lanes string index.
   * @return  Returns the index for the backward turn lanes string.
   */
  uint32_t bwd_turn_lanes_index() const {
    return bwd_turn_lanes_index_;
  }

  /**
   * Sets the index for destination.
   * @param  idx  Index for the destination.
   */
  void set_destination_index(const uint32_t idx) {
    destination_index_ = idx;
  }

  /**
   * Get the get_destination index.
   * @return  Returns the index for the destination.
   */
  uint32_t destination_index() const {
    return destination_index_;
  }

  /**
   * Sets the index for destination in forward direction.
   * @param  idx  Index for the destination.
   */
  void set_destination_forward_index(const uint32_t idx) {
    destination_forward_index_ = idx;
  }

  /**
   * Get the get_destination index.
   * @return  Returns the index for the destination.
   */
  uint32_t destination_forward_index() const {
    return destination_forward_index_;
  }

  /**
   * Sets the index for destination in backward direction.
   * @param  idx  Index for the destination.
   */
  void set_destination_backward_index(const uint32_t idx) {
    destination_backward_index_ = idx;
  }

  /**
   * Get the get_destination index.
   * @return  Returns the index for the destination.
   */
  uint32_t destination_backward_index() const {
    return destination_backward_index_;
  }

  /**
   * Sets the index for destination ref.
   * @param  idx  Index for the destination ref.
   */
  void set_destination_ref_index(const uint32_t idx) {
    destination_ref_index_ = idx;
  }

  /**
   * Get the destination_ref index.
   * @return  Returns the index for the destination ref.
   */
  uint32_t destination_ref_index() const {
    return destination_ref_index_;
  }

  /**
   * Sets the index for destination ref to.
   * @param  idx  Index for the destination ref to.
   */
  void set_destination_ref_to_index(const uint32_t idx) {
    destination_ref_to_index_ = idx;
  }

  /**
   * Get the destination ref to index.
   * @return  Returns the index for the destination ref to.
   */
  uint32_t destination_ref_to_index() const {
    return destination_ref_to_index_;
  }

  /**
   * Sets the index for destination street.
   * @param  idx  Index for the destination street.
   */
  void set_destination_street_index(const uint32_t idx) {
    destination_street_index_ = idx;
  }

  /**
   * Get the destination_street index.
   * @return  Returns the index for the destination street.
   */
  uint32_t destination_street_index() const {
    return destination_street_index_;
  }

  /**
   * Sets the index for destination street to.
   * @param  idx  Index for the destination street to.
   */
  void set_destination_street_to_index(const uint32_t idx) {
    destination_street_to_index_ = idx;
  }

  /**
   * Get the destination street to index.
   * @return  Returns the index for the destination street to.
   */
  uint32_t destination_street_to_index() const {
    return destination_street_to_index_;
  }

  /**
   * Sets the index for junction ref.
   * @param  idx  Index for the junction ref.
   */
  void set_junction_ref_index(const uint32_t idx) {
    junction_ref_index_ = idx;
  }

  /**
   * Get the junction ref index.
   * @return  Returns the index for the junction ref.
   */
  uint32_t junction_ref_index() const {
    return junction_ref_index_;
  }

  /**
   * Sets the index for bike national ref.
   * @param  idx  Index for the name of the national bike network.
   */
  void set_bike_national_ref_index(const uint32_t idx) {
    bike_national_ref_index_ = idx;
  }

  /**
   * Get the bike national ref index.
   * @return  Returns the index for the national bike network name.
   */
  uint32_t bike_national_ref_index() const {
    return bike_national_ref_index_;
  }

  /**
   * Sets the index for bike regional ref.
   * @param  idx  Index for the name of the regional bike network.
   */
  void set_bike_regional_ref_index(const uint32_t idx) {
    bike_regional_ref_index_ = idx;
  }

  /**
   * Get the bike regional ref index.
   * @return  Returns the index for the regional bike network name.
   */
  uint32_t bike_regional_ref_index() const {
    return bike_regional_ref_index_;
  }

  /**
   * Sets the index for bike local ref.
   * @param  idx  Index for the name of the local bike network.
   */
  void set_bike_local_ref_index(const uint32_t idx) {
    bike_local_ref_index_ = idx;
  }

  /**
   * Get the bike local ref index.
   * @return  Returns the index for the local bike network name.
   */
  uint32_t bike_local_ref_index() const {
    return bike_local_ref_index_;
  }

  /**
   * Sets the duration for ferries.
   * @param  duration  The time it takes to take this ferry (in seconds).
   */
  void set_duration(const uint32_t duration) {
    duration_ = duration;
  }

  /**
   * Gets the duration for ferries.
   * @return  Returns the time it takes to take this ferry (in seconds).
   */
  uint32_t duration() const {
    return duration_;
  }

  /**
   * Sets the auto_forward flag.
   * @param  auto_forward   Can you drive in the forward direction?
   */
  void set_auto_forward(const bool auto_forward) {
    access_.fields.auto_forward = auto_forward;
  }

  /**
   * Get the auto forward flag.
   * @return  Returns auto forward flag.
   */
  bool auto_forward() const {
    return access_.fields.auto_forward;
  }

  /**
   * Sets the bus_forward flag.
   * @param  bus_forward   Can a bus drive in the forward direction?
   */
  void set_bus_forward(const bool bus_forward) {
    access_.fields.bus_forward = bus_forward;
  }

  /**
   * Get the bus forward flag.
   * @return  Returns bus forward flag.
   */
  bool bus_forward() const {
    return access_.fields.bus_forward;
  }

  /**
   * Sets the taxi_forward flag.
   * @param  taxi_forward   Can a taxi drive in the forward direction?
   */
  void set_taxi_forward(const bool taxi_forward) {
    access_.fields.taxi_forward = taxi_forward;
  }

  /**
   * Get the taxi forward flag.
   * @return  Returns taxi forward flag.
   */
  bool taxi_forward() const {
    return access_.fields.taxi_forward;
  }

  /**
   * Sets the hov_forward flag.
   * @param  hov_forward   hov in the forward direction?
   */
  void set_hov_forward(const bool hov_forward) {
    access_.fields.hov_forward = hov_forward;
  }

  /**
   * Get the hov forward flag.
   * @return  Returns hov forward flag.
   */
  bool hov_forward() const {
    return access_.fields.hov_forward;
  }

  /**
   * Sets the truck_forward flag.
   * @param  truck_forward   Can a truck drive in the forward direction?
   */
  void set_truck_forward(const bool truck_forward) {
    access_.fields.truck_forward = truck_forward;
  }

  /**
   * Get the truck forward flag.
   * @return  Returns truck forward flag.
   */
  bool truck_forward() const {
    return access_.fields.truck_forward;
  }

  /**
   * Sets the bike_forward flag.
   * @param  bike_forward   Can you bike in the forward direction?
   */
  void set_bike_forward(const bool bike_forward) {
    bike_info_.fields.bike_forward = bike_forward;
  }

  /**
   * Get the bike forward flag.
   * @return  Returns bike forward flag.
   */
  bool bike_forward() const {
    return bike_info_.fields.bike_forward;
  }

  /**
   * Sets the emergency_forward flag.
   * @param  emergency_forward   Can an emergency vehicle drive in
   *                             the forward direction?
   */
  void set_emergency_forward(const bool emergency_forward) {
    access_.fields.emergency_forward = emergency_forward;
  }

  /**
   * Get the emergency forward flag.
   * @return  Returns emergency forward flag.
   */
  bool emergency_forward() const {
    return access_.fields.emergency_forward;
  }

  /**
   * Sets the moped_forward flag
   * @param  moped_forward  Can a moped drive in the forward direction?
   */
  void set_moped_forward(const bool moped_forward) {
    access_.fields.moped_forward = moped_forward;
  }

  /**
   * Get the moped forward flag
   * @return  Returns the moped forward flag
   */
  bool moped_forward() const {
    return access_.fields.moped_forward;
  }

  /**
   * Sets the motorcycle_forward flag
   * @param  motorcycle_forward  Can a motorcycle drive in the forward direction?
   */
  void set_motorcycle_forward(const bool motorcycle_forward) {
    access_.fields.motorcycle_forward = motorcycle_forward;
  }

  /**
   * Get the motorcycle forward flag
   * @return  Returns the motorcycle forward flag
   */
  bool motorcycle_forward() const {
    return access_.fields.motorcycle_forward;
  }

  /**
   * Sets the auto_backward flag.
   * @param  auto_backward   Can you drive in the reverse direction?
   */
  void set_auto_backward(const bool auto_backward) {
    access_.fields.auto_backward = auto_backward;
  }

  /**
   * Get the auto backward flag.
   * @return  Returns auto backward flag.
   */
  bool auto_backward() const {
    return access_.fields.auto_backward;
  }

  /**
   * Sets the bus_backward flag.
   * @param  bus_backward   Can you take a bus in the reverse direction?
   */
  void set_bus_backward(const bool bus_backward) {
    access_.fields.bus_backward = bus_backward;
  }

  /**
   * Get the bus backward flag.
   * @return  Returns bus backward flag.
   */
  bool bus_backward() const {
    return access_.fields.bus_backward;
  }

  /**
   * Sets the taxi_backward flag.
   * @param  taxi_backward   Can take a taxi in the reverse direction?
   */
  void set_taxi_backward(const bool taxi_backward) {
    access_.fields.taxi_backward = taxi_backward;
  }

  /**
   * Get the taxi backward flag.
   * @return  Returns taxi backward flag.
   */
  bool taxi_backward() const {
    return access_.fields.taxi_backward;
  }

  /**
   * Sets the hov_backward flag.
   * @param  hov_backward   hov in the reverse direction?
   */
  void set_hov_backward(const bool hov_backward) {
    access_.fields.hov_backward = hov_backward;
  }

  /**
   * Get the hov backward flag.
   * @return  Returns hov backward flag.
   */
  bool hov_backward() const {
    return access_.fields.hov_backward;
  }

  /**
   * Sets the truck_backward flag.
   * @param  truck_backward   Can you drive in the reverse direction?
   */
  void set_truck_backward(const bool truck_backward) {
    access_.fields.truck_backward = truck_backward;
  }

  /**
   * Get the truck backward flag.
   * @return  Returns truck backward flag.
   */
  bool truck_backward() const {
    return access_.fields.truck_backward;
  }

  /**
   * Sets the bike_backward flag.
   * @param  bike_backward   Can you bike in the reverse direction?
   */
  void set_bike_backward(const bool bike_backward) {
    bike_info_.fields.bike_backward = bike_backward;
  }

  /**
   * Get the bike backward flag.
   * @return  Returns bike backward flag.
   */
  bool bike_backward() const {
    return bike_info_.fields.bike_backward;
  }

  /**
   * Sets the emergency_backward flag.
   * @param  emergency_backward   Can an emergency vehicle drive
   *                              in the reverse direction?
   */
  void set_emergency_backward(const bool emergency_backward) {
    access_.fields.emergency_backward = emergency_backward;
  }

  /**
   * Get the emergency backward flag.
   * @return  Returns emergency backward flag.
   */
  bool emergency_backward() const {
    return access_.fields.emergency_backward;
  }

  /**
   * Set the moped_backward flag.
   * @param  moped_backward  Can a moped drive in the
   *                         reverse direction?
   */
  void set_moped_backward(const bool moped_backward) {
    access_.fields.moped_backward = moped_backward;
  }

  /**
   * Get the moped backward flag.
   * @return  Returns moped backward flag.
   */
  bool moped_backward() const {
    return access_.fields.moped_backward;
  }

  /**
   * Set the motorcycle_backward flag.
   * @param  motorcycle_backward  Can a motorcycle drive in the
   *                              reverse direction?
   */
  void set_motorcycle_backward(const bool motorcycle_backward) {
    access_.fields.motorcycle_backward = motorcycle_backward;
  }

  /**
   * Get the motorcycle backward flag.
   * @return  Returns motorcycle backward flag.
   */
  bool motorcycle_backward() const {
    return access_.fields.motorcycle_backward;
  }

  /**
   * Sets the destination_only flag.
   * @param  destination_only   Is private?
   */
  void set_destination_only(const bool destination_only) {
    attributes_.fields.destination_only = destination_only;
  }

  /**
   * Get the destination only/private flag.
   * @return  Returns private flag.
   */
  bool destination_only() const {
    return attributes_.fields.destination_only;
  }

  /**
   * Sets the pedestrian flag.
   * @param  pedestrian   Are pedestrians allowed?
   */
  void set_pedestrian(const bool pedestrian) {
    classification_.fields.pedestrian = pedestrian;
  }

  /**
   * Get the pedestrian flag.
   * @return  Returns pedestrian flag.
   */
  bool pedestrian() const {
    return classification_.fields.pedestrian;
  }

  /**
   * Sets the has_user_tags flag.
   * @param  has_user_tags   Did a user enter the access tags?
   */
  void set_has_user_tags(const bool has_user_tags) {
    classification_.fields.has_user_tags = has_user_tags;
  }

  /**
   * Get the has_user_tags flag.
   * @return  Returns has_user_tags flag.
   */
  bool has_user_tags() const {
    return classification_.fields.has_user_tags;
  }

  /**
   * Sets the no thru traffic flag.
   * @param  no_thru_traffic   Traffic allowed?
   */
  void set_no_thru_traffic(const bool no_thru_traffic) {
    attributes_.fields.no_thru_traffic = no_thru_traffic;
  }

  /**
   * Get the no thru traffic flag.
   * @return  Returns no thru traffic flag.
   */
  bool no_thru_traffic() const {
    return attributes_.fields.no_thru_traffic;
  }

  /**
   * Sets the oneway flag.
   * @param  oneway   Is oneway?
   */
  void set_oneway(const bool oneway) {
    attributes_.fields.oneway = oneway;
  }

  /**
   * Get the oneway flag.
   * @return  Returns oneway flag.
   */
  bool oneway() const {
    return attributes_.fields.oneway;
  }

  /**
   * Sets if the oneway is in the opposite direction
   * @param  oneway_reverse
   */
  void set_oneway_reverse(const bool oneway_reverse) {
    attributes_.fields.oneway_reverse = oneway_reverse;
  }

  /**
   * Gets if the oneway is in the opposite direction
   * @return  Returns if oneway is reversed
   */
  bool oneway_reverse() const {
    return attributes_.fields.oneway_reverse;
  }

  /**
   * Sets the roundabout flag.
   * @param  roundabout   Is a roundabout?
   */
  void set_roundabout(const bool roundabout) {
    attributes_.fields.roundabout = roundabout;
  }

  /**
   * Get the roundabout flag.
   * @return  Returns roundabout flag.
   */
  bool roundabout() const {
    return attributes_.fields.roundabout;
  }

  /**
   * Sets the ferry flag.
   * @param  ferry   Is a ferry?
   */
  void set_ferry(const bool ferry) {
    attributes_.fields.ferry = ferry;
  }

  /**
   * Get the ferry flag.
   * @return  Returns ferry flag.
   */
  bool ferry() const {
    return attributes_.fields.ferry;
  }

  /**
   * Sets the rail flag.
   * @param  rail   Is a auto train?
   */
  void set_rail(const bool rail) {
    attributes_.fields.rail = rail;
  }

  /**
   * Get the rail flag.
   * @return  Returns rail flag.
   */
  bool rail() const {
    return attributes_.fields.rail;
  }

  /**
   * Sets the surface.
   * @param  surface
   */
  void set_surface(const baldr::Surface surface) {
    attributes_.fields.surface = static_cast<uint8_t>(surface);
  }

  /**
   * Get the surface.
   * @return  Returns Surface.
   */
  baldr::Surface surface() const {
    return static_cast<baldr::Surface>(attributes_.fields.surface);
  }

  /**
   * Sets the sac scale.
   * @param  sac_scale
   */
  void set_sac_scale(const baldr::SacScale sac_scale) {
    attributes_.fields.sac_scale = static_cast<uint8_t>(sac_scale);
  }

  /**
   * Gets the sac scale.
   * @return  Returns sac_scale
   */
  baldr::SacScale sac_scale() const {
    return static_cast<baldr::SacScale>(attributes_.fields.sac_scale);
  }

  /**
   * Sets the right cycle lane.
   * @param  cyclelane
   */
  void set_cyclelane_right(const baldr::CycleLane cyclelane) {
    bike_info_.fields.cycle_lane_right = static_cast<uint8_t>(cyclelane);
  }

  /**
   * Gets the right cycle lane.
   * @return  Returns CycleLane on right.
   */
  baldr::CycleLane cyclelane_right() const {
    return static_cast<baldr::CycleLane>(bike_info_.fields.cycle_lane_right);
  }

  /**
   * Sets the left cycle lane.
   * @param  cyclelane
   */
  void set_cyclelane_left(const baldr::CycleLane cyclelane) {
    bike_info_.fields.cycle_lane_left = static_cast<uint8_t>(cyclelane);
  }

  /**
   * Gets the left cycle lane.
   * @return  Returns CycleLane on left.
   */
  baldr::CycleLane cyclelane_left() const {
    return static_cast<baldr::CycleLane>(bike_info_.fields.cycle_lane_left);
  }

  /**
   * Sets if the right cycle lane is facing the opposite direction
   * @param  cyclelane_opposite
   */
  void set_cyclelane_right_opposite(const bool cyclelane_opposite) {
    bike_info_.fields.cycle_lane_right_opposite = cyclelane_opposite;
  }

  /**
   * Gets if the right cycle lane is facing the opposite direction
   * @return  Returns cycle_lane_right_opposite
   */
  bool cyclelane_right_opposite() const {
    return bike_info_.fields.cycle_lane_right_opposite;
  }

  /**
   * Sets if the left cycle lane is facing the opposite direction
   * @param  cyclelane_opposite
   */
  void set_cyclelane_left_opposite(const bool cyclelane_opposite) {
    bike_info_.fields.cycle_lane_left_opposite = cyclelane_opposite;
  }

  /**
   * Gets if the left cycle lane is facing the opposite direction
   * @return  Returns cycle_lane_left_opposite
   */
  bool cyclelane_left_opposite() const {
    return bike_info_.fields.cycle_lane_left_opposite;
  }

  /**
   * Set if edge has a shoulder on the right
   * @param  shoulder  True if edge has shoulder on right
   */
  void set_shoulder_right(const bool shoulder_right) {
    bike_info_.fields.shoulder_right = shoulder_right;
  }

  /**
   * Get if edge has a shoulder on the right
   * @return  Returns if edge has a shoulder on right
   */
  bool shoulder_right() const {
    return bike_info_.fields.shoulder_right;
  }

  /**
   * Set if edge has a shoulder on the left
   * @param  shoulder  True if edge has shoulder on left
   */
  void set_shoulder_left(const bool shoulder_left) {
    bike_info_.fields.shoulder_left = shoulder_left;
  }

  /**
   * Get if edge has a shoulder on the left
   * @return  Returns if edge has a shoulder on left
   */
  bool shoulder_left() const {
    return bike_info_.fields.shoulder_left;
  }

  /**
   * Sets if a bicyclist needs to dismount their bike
   * @param  dismount  Whether a cyclist needs to dismount or not
   */
  void set_dismount(const bool dismount) {
    bike_info_.fields.dismount = dismount;
  }

  /**
   * Gets if a bicyclist needs to dismount their bike
   * @return  Returns dismount
   */
  bool dismount() const {
    return bike_info_.fields.dismount;
  }

  /**
   * Sets whether a pedestrian or cyclist should have preference to use a different
   * path to the side (A separate OSMWay completely)
   * @param  use_sidepath
   */
  void set_use_sidepath(const bool use_sidepath) {
    bike_info_.fields.use_sidepath = use_sidepath;
  }

  /*
   * Gets whether a pedestrian or cyclist should have preference to use a different
   * path to the side (A separate OSMWay completely)
   * @return  Returns if using a sidepath is preffered
   */
  bool use_sidepath() const {
    return bike_info_.fields.use_sidepath;
  }

  /**
   * Sets the number of lanes
   * @param  lanes  Number of lanes
   */
  void set_lanes(const uint32_t lanes);

  /**
   * Get the number of lanes
   * @return  Returns number of lanes.
   */
  uint32_t lanes() const {
    return classification_.fields.lanes;
  }

  /**
   * Sets the number of backward lanes
   * @param  backward_lanes  Number of backward lanes
   */
  void set_backward_lanes(const uint32_t backward_lanes);

  /**
   * Get the number of backward lanes
   * @return  Returns number of backward lanes.
   */
  uint32_t backward_lanes() const {
    return classification_.fields.backward_lanes;
  }

  /**
   * Sets the number of forward lanes
   * @param  forward_lanes  Number of forward lanes
   */
  void set_forward_lanes(const uint32_t forward_lanes);

  /**
   * Get the number of forward lanes
   * @return  Returns number of forward lanes.
   */
  uint32_t forward_lanes() const {
    return classification_.fields.forward_lanes;
  }

  /**
   * Sets the tunnel flag.
   * @param  tunnel   Is a tunnel road?
   */
  void set_tunnel(const bool tunnel) {
    attributes_.fields.tunnel = tunnel;
  }

  /**
   * Get the tunnel flag.
   * @return  Returns tunnel flag.
   */
  bool tunnel() const {
    return attributes_.fields.tunnel;
  }

  /**
   * Sets the toll flag.
   * @param  toll   Is a toll road?
   */
  void set_toll(const bool toll) {
    attributes_.fields.toll = toll;
  }

  /**
   * Get the toll flag.
   * @return  Returns toll flag.
   */
  bool toll() const {
    return attributes_.fields.toll;
  }

  /**
   * Sets the bridge flag.
   * @param  bridge   Is a bridge?
   */
  void set_bridge(const bool bridge) {
    attributes_.fields.bridge = bridge;
  }

  /**
   * Get the bridge flag.
   * @return  Returns bridge flag.
   */
  bool bridge() const {
    return attributes_.fields.bridge;
  }

  /**
   * Set seasonal flag.
   * @param  seasonal   Is this seasonal?
   */
  void set_seasonal(const bool seasonal) {
    attributes_.fields.seasonal = seasonal;
  }

  /**
   * Get the seasonal flag.
   * @return  Returns seasonal flag.
   */
  bool seasonal() const {
    return attributes_.fields.seasonal;
  }

  /**
   * Set wheelchair flag.
   * @param  wheelchair   Is this wheelchair?
   */
  void set_wheelchair(const bool wheelchair) {
    classification_.fields.wheelchair = wheelchair;
  }

  /**
   * Get the wheelchair flag.
   * @return  Returns wheelchair flag.
   */
  bool wheelchair() const {
    return classification_.fields.wheelchair;
  }

  /**
   * Set wheelchair_tag flag.
   * @param  wheelchair_tag   Did the user set the wheelchair_tag?
   */
  void set_wheelchair_tag(const bool wheelchair_tag) {
    classification_.fields.wheelchair_tag = wheelchair_tag;
  }

  /**
   * Get the wheelchair_tag flag.
   * @return  Returns wheelchair_tag flag.
   */
  bool wheelchair_tag() const {
    return classification_.fields.wheelchair_tag;
  }

  /**
   * Set sidewalk_left flag.
   * @param  sidewalk_left   Is there a sidewalk on the left?
   */
  void set_sidewalk_left(const bool sidewalk_left) {
    attributes_.fields.sidewalk_left = sidewalk_left;
  }

  /**
   * Get the sidewalk_left flag.
   * @return  Returns sidewalk_left flag.
   */
  bool sidewalk_left() const {
    return attributes_.fields.sidewalk_left;
  }

  /**
   * Set sidewalk_right flag.
   * @param  sidewalk_right   Is there a sidewalk on the right?
   */
  void set_sidewalk_right(const bool sidewalk_right) {
    attributes_.fields.sidewalk_right = sidewalk_right;
  }

  /**
   * Get the sidewalk_right flag.
   * @return  Returns sidewalk_right flag.
   */
  bool sidewalk_right() const {
    return attributes_.fields.sidewalk_right;
  }

  /**
   * Set drive_on_right flag.
   * @param  drive_on_right   Is a country that we drive on the right?
   */
  void set_drive_on_right(const bool drive_on_right) {
    attributes_.fields.drive_on_right = drive_on_right;
  }

  /**
   * Get the drive on right flag.
   * @return  Returns drive on right flag.
   */
  bool drive_on_right() const {
    return attributes_.fields.drive_on_right;
  }

  /**
   * Sets the bike network.
   * @param  bike_network Mask of the bike networks (ncn/rcn/lcn).
   */
  void set_bike_network(const uint32_t bike_network) {
    attributes_.fields.bike_network = bike_network;
  }

  /**
   * Get the bike network mask.
   * @return  Returns the bike network mask.
   */
  uint32_t bike_network() const {
    return attributes_.fields.bike_network;
  }

  /**
   * Sets the exit tag.
   * @param  exit       Exit flag.
   */
  void set_exit(const bool exit) {
    attributes_.fields.exit = exit;
  }

  /**
   * Get the exit flag.
   * @return  Returns exit flag.
   */
  bool exit() const {
    return attributes_.fields.exit;
  }

  /**
   * Sets the tagged_speed flag.
   * @param  tagged_speed  User specified speed?
   */
  void set_tagged_speed(const bool tagged_speed) {
    attributes_.fields.tagged_speed = tagged_speed;
  }

  /**
   * Get the tagged_speed flag.
   * @return  Returns tagged_speed flag.
   */
  bool tagged_speed() const {
    return attributes_.fields.tagged_speed;
  }

  /**
   * Sets the forward tagged_speed flag.
   * @param  forward_tagged_speed  User specified speed?
   */
  void set_forward_tagged_speed(const bool forward_tagged_speed) {
    attributes_.fields.forward_tagged_speed = forward_tagged_speed;
  }

  /**
   * Get the forward_tagged_speed flag.
   * @return  Returns forward_tagged_speed flag.
   */
  bool forward_tagged_speed() const {
    return attributes_.fields.forward_tagged_speed;
  }

  /**
   * Sets the backward tagged_speed flag.
   * @param  backward_tagged_speed  User specified speed?
   */
  void set_backward_tagged_speed(const bool backward_tagged_speed) {
    attributes_.fields.backward_tagged_speed = backward_tagged_speed;
  }

  /**
   * Get the backward_tagged_speed flag.
   * @return  Returns backward_tagged_speed flag.
   */
  bool backward_tagged_speed() const {
    return attributes_.fields.backward_tagged_speed;
  }

  /**
   * Sets the tagged_lanes flag.
   * @param  tagged_lanes  User specified lanes?
   */
  void set_tagged_lanes(const bool tagged_lanes) {
    attributes_.fields.tagged_lanes = tagged_lanes;
  }

  /**
   * Get the tagged_lanes flag.
   * @return  Returns tagged_lanes flag.
   */
  bool tagged_lanes() const {
    return attributes_.fields.tagged_lanes;
  }

  /**
   * Sets the forward tagged_lanes flag.
   * @param  forward_tagged_lanes  User specified lanes?
   */
  void set_forward_tagged_lanes(const bool forward_tagged_lanes) {
    attributes_.fields.forward_tagged_lanes = forward_tagged_lanes;
  }

  /**
   * Get the forward_tagged_lanes flag.
   * @return  Returns forward_tagged_lanes flag.
   */
  bool forward_tagged_lanes() const {
    return attributes_.fields.forward_tagged_lanes;
  }

  /**
   * Sets the backward tagged_lanes flag.
   * @param  backward_tagged_lanes  User specified lanes?
   */
  void set_backward_tagged_lanes(const bool backward_tagged_lanes) {
    attributes_.fields.backward_tagged_lanes = backward_tagged_lanes;
  }

  /**
   * Get the backward_tagged_lanes flag.
   * @return  Returns backward_tagged_lanes flag.
   */
  bool backward_tagged_lanes() const {
    return attributes_.fields.backward_tagged_lanes;
  }

  /**
   * Sets the truck route flag.
   * @param  truck_route  Is this part of the local, national,
   *                      or state truck network or designated truck way?
   */
  void set_truck_route(const bool truck_route) {
    attributes_.fields.truck_route = truck_route;
  }

  /**
   * Get the truck_route flag.
   * @return  Returns truck_route flag.
   */
  bool truck_route() const {
    return attributes_.fields.truck_route;
  }

  /**
   * Sets the road class.
   * @param  roadclass  Road Class/highway type.
   */
  void set_road_class(const baldr::RoadClass roadclass) {
    classification_.fields.road_class = static_cast<uint8_t>(roadclass);
  }

  /**
   * Get the road class.
   * @return  Returns road class.
   */
  baldr::RoadClass road_class() const {
    return static_cast<baldr::RoadClass>(classification_.fields.road_class);
  }

  /**
   * Sets the use tag.
   * @param  use       use. None Cycleway ParkingAisle, Driveway, Alley,
   *                        EmergencyAccess, DriveThru, Steps, and Other
   */
  void set_use(const baldr::Use use) {
    classification_.fields.use = static_cast<uint8_t>(use);
  }

  /**
   * Get the use.
   * @return  Returns use.
   */
  baldr::Use use() const {
    return static_cast<baldr::Use>(classification_.fields.use);
  }

  /**
   * Sets the link tag.
   * @param  link       Link.  Ramp or turn channel.
   */
  void set_link(const bool link) {
    classification_.fields.link = link;
  }

  /**
   * Get the link flag.
   * @return  Returns link flag.
   */
  bool link() const {
    return classification_.fields.link;
  }

  /**
   * Sets the turn channel tag.
   * @param  turn channel       Turn channel.
   */
  void set_turn_channel(const bool turn_channel) {
    classification_.fields.turn_channel = turn_channel;
  }

  /**
   * Get the turn channel flag.
   * @return  Returns turn channel flag.
   */
  bool turn_channel() const {
    return classification_.fields.turn_channel;
  }

  /**
   * Get the names for the edge info based on the road class.
   * @param  ref              updated refs from relations.
   * @param  ref_offset_map   map of unique refs from ways.
   * @param  name_offset_map  map of unique names from ways.
   * @return  Returns vector of strings
   */
  std::vector<std::string> GetNames(const std::string& ref,
                                    const UniqueNames& ref_offset_map,
                                    const UniqueNames& name_offset_map,
                                    uint16_t& types) const;

  // OSM way Id
  uint64_t osmwayid_;

  // Reference name (highway numbers)
  uint32_t ref_index_;
  uint32_t int_ref_index_;

  // Names
  uint32_t name_index_;
  uint32_t name_en_index_;
  uint32_t alt_name_index_;
  uint32_t official_name_index_;

  // Turn lanes
  uint32_t fwd_turn_lanes_index_;
  uint32_t bwd_turn_lanes_index_;

  // Sign Destination information
  uint32_t destination_index_;
  uint32_t destination_forward_index_;
  uint32_t destination_backward_index_;
  uint32_t destination_ref_index_;
  uint32_t destination_ref_to_index_;
  uint32_t destination_street_index_;
  uint32_t destination_street_to_index_;
  uint32_t junction_ref_index_;

  // Bike network information
  uint32_t bike_national_ref_index_;
  uint32_t bike_regional_ref_index_;
  uint32_t bike_local_ref_index_;

  // duration of a ferry in seconds
  uint32_t duration_;

  // Way attributes
  union WayAttributes {
    struct Fields {
      uint32_t destination_only : 1;
      uint32_t no_thru_traffic : 1;
      uint32_t oneway : 1;
      uint32_t oneway_reverse : 1;
      uint32_t roundabout : 1;
      uint32_t ferry : 1;
      uint32_t rail : 1;
      uint32_t surface : 3;
      uint32_t tunnel : 1;
      uint32_t toll : 1;
      uint32_t bridge : 1;
      uint32_t seasonal : 1;
      uint32_t drive_on_right : 1;
      uint32_t bike_network : 4;
      uint32_t exit : 1;
      uint32_t tagged_speed : 1;
      uint32_t forward_tagged_speed : 1;
      uint32_t backward_tagged_speed : 1;
      uint32_t tagged_lanes : 1;
      uint32_t forward_tagged_lanes : 1;
      uint32_t backward_tagged_lanes : 1;
      uint32_t truck_route : 1;
      uint32_t sidewalk_right : 1;
      uint32_t sidewalk_left : 1;
      uint32_t sac_scale : 3;
    } fields;
    uint32_t v;
  };
  WayAttributes attributes_;

  union Classification {
    struct Fields {
      uint32_t road_class : 3; // Importance of the road/path
      uint32_t link : 1;       // *link tag - Ramp or turn channel
      uint32_t use : 6;        // Use / form
      uint32_t lanes : 4;
      uint32_t forward_lanes : 4;
      uint32_t backward_lanes : 4;
      uint32_t turn_channel : 1; // *link tag - turn channel (no ramp)
      uint16_t wheelchair : 1;
      uint16_t wheelchair_tag : 1;
      uint32_t pedestrian : 1;
      uint32_t has_user_tags : 1;
      uint32_t spare : 5; // Spare
    } fields;
    uint32_t v;
  };
  Classification classification_;

  // Access
  union WayAccess {
    struct Fields {
      uint16_t auto_forward : 1;
      uint16_t bus_forward : 1;
      uint16_t taxi_forward : 1;
      uint16_t truck_forward : 1;
      uint16_t motorcycle_forward : 1;
      uint16_t emergency_forward : 1;
      uint16_t hov_forward : 1;
      uint16_t moped_forward : 1;
      uint16_t auto_backward : 1;
      uint16_t bus_backward : 1;
      uint16_t taxi_backward : 1;
      uint16_t truck_backward : 1;
      uint16_t motorcycle_backward : 1;
      uint16_t emergency_backward : 1;
      uint16_t hov_backward : 1;
      uint16_t moped_backward : 1;
    } fields;
    uint16_t v;
  };
  WayAccess access_;

  // Attributes specific to biking
  union BikeInfo {
    struct Fields {
      uint16_t cycle_lane_right : 2;
      uint16_t cycle_lane_left : 2;
      uint16_t cycle_lane_right_opposite : 1;
      uint16_t cycle_lane_left_opposite : 1;
      uint16_t shoulder_right : 1;
      uint16_t shoulder_left : 1;
      uint16_t dismount : 1;
      uint16_t use_sidepath : 1;
      uint16_t bike_forward : 1;
      uint16_t bike_backward : 1;
      uint16_t spare : 4;
    } fields;
    uint16_t v;
  };
  BikeInfo bike_info_;

  uint16_t nodecount_;

  // max speed limit in kilometers per hour
  uint8_t speed_limit_;

  // average speed if exists, else advisory speed if exists, else max_speed if exists,
  // else categorized speed in kilometers per hour
  uint8_t speed_;

  // Speed in kilometers per hour
  uint8_t backward_speed_;

  // Speed in kilometers per hour
  uint8_t forward_speed_;

  // Truck speed in kilometers per hour
  uint8_t truck_speed_;

  uint8_t spare_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMWAY_H
