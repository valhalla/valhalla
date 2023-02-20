#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMWAY_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMWAY_H

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/mjolnir/osmpronunciation.h>
#include <valhalla/mjolnir/uniquenames.h>

namespace valhalla {
namespace mjolnir {

// OSM way
struct OSMWay {
  OSMWay() {
    memset(this, 0, sizeof(OSMWay));
  }

  OSMWay(uint64_t id) {
    memset(this, 0, sizeof(OSMWay));
    osmwayid_ = id;
  }

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
   * Sets the index for tunnel name
   * @param  idx  Index for the tunnel name.
   */
  void set_tunnel_name_index(const uint32_t idx) {
    tunnel_name_index_ = idx;
  }

  /**
   * Get the tunnel name index.
   * @return  Returns the index for the tunnel name.
   */
  uint32_t tunnel_name_index() const {
    return tunnel_name_index_;
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
   * Sets the index for forward jct base string.
   * @param  idx  Index for the forward jct base string.
   */
  void set_fwd_jct_base_index(const uint32_t idx) {
    fwd_jct_base_index_ = idx;
  }

  /**
   * Get the forward jct base string index.
   * @return  Returns the index for the forward jct base string.
   */
  uint32_t fwd_jct_base_index() const {
    return fwd_jct_base_index_;
  }

  /**
   * Sets the index for backward jct base string.
   * @param  idx  Index for the backward jct base string.
   */
  void set_bwd_jct_base_index(const uint32_t idx) {
    bwd_jct_base_index_ = idx;
  }

  /**
   * Get the backward jct base string index.
   * @return  Returns the index for the backward jct base string.
   */
  uint32_t bwd_jct_base_index() const {
    return bwd_jct_base_index_;
  }

  /**
   * Sets the index for forward jct overlay string.
   * @param  idx  Index for the forward jct overlay string.
   */
  void set_fwd_jct_overlay_index(const uint32_t idx) {
    fwd_jct_overlay_index_ = idx;
  }

  /**
   * Get the forward jct overlay string index.
   * @return  Returns the index for the forward jct overlay string.
   */
  uint32_t fwd_jct_overlay_index() const {
    return fwd_jct_overlay_index_;
  }

  /**
   * Sets the index for backward jct overlay string.
   * @param  idx  Index for the backward jct overlay string.
   */
  void set_bwd_jct_overlay_index(const uint32_t idx) {
    bwd_jct_overlay_index_ = idx;
  }

  /**
   * Get the backward jct overlay string index.
   * @return  Returns the index for the backward jct overlay string.
   */
  uint32_t bwd_jct_overlay_index() const {
    return bwd_jct_overlay_index_;
  }

  /**
   * Sets the index for forward signboard base string.
   * @param  idx  Index for the forward signboard base string.
   */
  void set_fwd_signboard_base_index(const uint32_t idx) {
    fwd_signboard_base_index_ = idx;
  }

  /**
   * Get the forward signboard base string index.
   * @return  Returns the index for the forward signboard base string.
   */
  uint32_t fwd_signboard_base_index() const {
    return fwd_signboard_base_index_;
  }

  /**
   * Sets the index for backward signboard base string.
   * @param  idx  Index for the backward signboard base string.
   */
  void set_bwd_signboard_base_index(const uint32_t idx) {
    bwd_signboard_base_index_ = idx;
  }

  /**
   * Get the backward signboard base string index.
   * @return  Returns the index for the backward signboard base string.
   */
  uint32_t bwd_signboard_base_index() const {
    return bwd_signboard_base_index_;
  }

  /**
   * Sets the index for destination.
   * @param  idx  Index for the destination.
   */
  void set_destination_index(const uint32_t idx) {
    destination_index_ = idx;
  }

  /**
   * Get the destination index.
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
   * Get the destination in forward direction index.
   * @return  Returns the index for the destination in forward direction.
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
   * Get the destination in backward direction index.
   * @return  Returns the index for the destination in backward direction.
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
   * Sets the index for junction ref pronunciation.
   * @param  idx  Index for the junction ref pronunciation.
   */
  void set_junction_ref_index(const uint32_t idx) {
    junction_ref_index_ = idx;
  }

  /**
   * Get the junction ref pronunciation index.
   * @return  Returns the index for the junction ref pronunciation.
   */
  uint32_t junction_ref_index() const {
    return junction_ref_index_;
  }

  /**
   * Sets the index for bike national ref.
   * @param  idx  Index for the name of the national bike network.
   */
  //  void set_bike_national_ref_index(const uint32_t idx) {
  //    ; // bike_national_ref_index_ = idx; UNUSED - future
  //  }

  /**
   * Get the bike national ref index.
   * @return  Returns the index for the national bike network name.
   */
  //  uint32_t bike_national_ref_index() const {
  //    return bike_national_ref_index_;
  //  }

  /**
   * Sets the index for bike regional ref.
   * @param  idx  Index for the name of the regional bike network.
   */
  //  void set_bike_regional_ref_index(const uint32_t idx) {
  //    ; // bike_regional_ref_index_ = idx; UNUSED - future
  //  }

  /**
   * Get the bike regional ref index.
   * @return  Returns the index for the regional bike network name.
   */
  //  uint32_t bike_regional_ref_index() const {
  //    return bike_regional_ref_index_;
  //  }

  /**
   * Sets the index for bike local ref.
   * @param  idx  Index for the name of the local bike network.
   */
  //  void set_bike_local_ref_index(const uint32_t idx) {
  //    ; // bike_local_ref_index_ = idx; UNUSED - future
  //  }

  /**
   * Get the bike local ref index.
   * @return  Returns the index for the local bike network name.
   */
  //  uint32_t bike_local_ref_index() const {
  //    return bike_local_ref_index_;
  //  }

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
    auto_forward_ = auto_forward;
  }

  /**
   * Get the auto forward flag.
   * @return  Returns auto forward flag.
   */
  bool auto_forward() const {
    return auto_forward_;
  }

  /**
   * Sets the bus_forward flag.
   * @param  bus_forward   Can a bus drive in the forward direction?
   */
  void set_bus_forward(const bool bus_forward) {
    bus_forward_ = bus_forward;
  }

  /**
   * Get the bus forward flag.
   * @return  Returns bus forward flag.
   */
  bool bus_forward() const {
    return bus_forward_;
  }

  /**
   * Sets the taxi_forward flag.
   * @param  taxi_forward   Can a taxi drive in the forward direction?
   */
  void set_taxi_forward(const bool taxi_forward) {
    taxi_forward_ = taxi_forward;
  }

  /**
   * Get the taxi forward flag.
   * @return  Returns taxi forward flag.
   */
  bool taxi_forward() const {
    return taxi_forward_;
  }

  /**
   * Sets the hov_forward flag.
   * @param  hov_forward   hov in the forward direction?
   */
  void set_hov_forward(const bool hov_forward) {
    hov_forward_ = hov_forward;
  }

  /**
   * Get the hov forward flag.
   * @return  Returns hov forward flag.
   */
  bool hov_forward() const {
    return hov_forward_;
  }

  /**
   * Sets the truck_forward flag.
   * @param  truck_forward   Can a truck drive in the forward direction?
   */
  void set_truck_forward(const bool truck_forward) {
    truck_forward_ = truck_forward;
  }

  /**
   * Get the truck forward flag.
   * @return  Returns truck forward flag.
   */
  bool truck_forward() const {
    return truck_forward_;
  }

  /**
   * Sets the bike_forward flag.
   * @param  bike_forward   Can you bike in the forward direction?
   */
  void set_bike_forward(const bool bike_forward) {
    bike_forward_ = bike_forward;
  }

  /**
   * Get the bike forward flag.
   * @return  Returns bike forward flag.
   */
  bool bike_forward() const {
    return bike_forward_;
  }

  /**
   * Sets the emergency_forward flag.
   * @param  emergency_forward   Can an emergency vehicle drive in
   *                             the forward direction?
   */
  void set_emergency_forward(const bool emergency_forward) {
    emergency_forward_ = emergency_forward;
  }

  /**
   * Get the emergency forward flag.
   * @return  Returns emergency forward flag.
   */
  bool emergency_forward() const {
    return emergency_forward_;
  }

  /**
   * Sets the moped_forward flag
   * @param  moped_forward  Can a moped drive in the forward direction?
   */
  void set_moped_forward(const bool moped_forward) {
    moped_forward_ = moped_forward;
  }

  /**
   * Get the moped forward flag
   * @return  Returns the moped forward flag
   */
  bool moped_forward() const {
    return moped_forward_;
  }

  /**
   * Sets the motorcycle_forward flag
   * @param  motorcycle_forward  Can a motorcycle drive in the forward direction?
   */
  void set_motorcycle_forward(const bool motorcycle_forward) {
    motorcycle_forward_ = motorcycle_forward;
  }

  /**
   * Get the motorcycle forward flag
   * @return  Returns the motorcycle forward flag
   */
  bool motorcycle_forward() const {
    return motorcycle_forward_;
  }

  /**
   * Sets the pedestrian forward flag.
   * @param  pedestrian_forward   Are pedestrians allowed in the forward direction?
   */
  void set_pedestrian_forward(const bool pedestrian_forward) {
    pedestrian_forward_ = pedestrian_forward;
  }

  /**
   * Get the pedestrian forward flag.
   * @return  Returns pedestrian forward flag.
   */
  bool pedestrian_forward() const {
    return pedestrian_forward_;
  }

  /**
   * Sets the auto_backward flag.
   * @param  auto_backward   Can you drive in the reverse direction?
   */
  void set_auto_backward(const bool auto_backward) {
    auto_backward_ = auto_backward;
  }

  /**
   * Get the auto backward flag.
   * @return  Returns auto backward flag.
   */
  bool auto_backward() const {
    return auto_backward_;
  }

  /**
   * Sets the bus_backward flag.
   * @param  bus_backward   Can you take a bus in the reverse direction?
   */
  void set_bus_backward(const bool bus_backward) {
    bus_backward_ = bus_backward;
  }

  /**
   * Get the bus backward flag.
   * @return  Returns bus backward flag.
   */
  bool bus_backward() const {
    return bus_backward_;
  }

  /**
   * Sets the taxi_backward flag.
   * @param  taxi_backward   Can take a taxi in the reverse direction?
   */
  void set_taxi_backward(const bool taxi_backward) {
    taxi_backward_ = taxi_backward;
  }

  /**
   * Get the taxi backward flag.
   * @return  Returns taxi backward flag.
   */
  bool taxi_backward() const {
    return taxi_backward_;
  }

  /**
   * Sets the hov_backward flag.
   * @param  hov_backward   hov in the reverse direction?
   */
  void set_hov_backward(const bool hov_backward) {
    hov_backward_ = hov_backward;
  }

  /**
   * Get the hov backward flag.
   * @return  Returns hov backward flag.
   */
  bool hov_backward() const {
    return hov_backward_;
  }

  /**
   * Sets the truck_backward flag.
   * @param  truck_backward   Can you drive in the reverse direction?
   */
  void set_truck_backward(const bool truck_backward) {
    truck_backward_ = truck_backward;
  }

  /**
   * Get the truck backward flag.
   * @return  Returns truck backward flag.
   */
  bool truck_backward() const {
    return truck_backward_;
  }

  /**
   * Sets the bike_backward flag.
   * @param  bike_backward   Can you bike in the reverse direction?
   */
  void set_bike_backward(const bool bike_backward) {
    bike_backward_ = bike_backward;
  }

  /**
   * Get the bike backward flag.
   * @return  Returns bike backward flag.
   */
  bool bike_backward() const {
    return bike_backward_;
  }

  /**
   * Sets the emergency_backward flag.
   * @param  emergency_backward   Can an emergency vehicle drive
   *                              in the reverse direction?
   */
  void set_emergency_backward(const bool emergency_backward) {
    emergency_backward_ = emergency_backward;
  }

  /**
   * Get the emergency backward flag.
   * @return  Returns emergency backward flag.
   */
  bool emergency_backward() const {
    return emergency_backward_;
  }

  /**
   * Set the moped_backward flag.
   * @param  moped_backward  Can a moped drive in the
   *                         reverse direction?
   */
  void set_moped_backward(const bool moped_backward) {
    moped_backward_ = moped_backward;
  }

  /**
   * Get the moped backward flag.
   * @return  Returns moped backward flag.
   */
  bool moped_backward() const {
    return moped_backward_;
  }

  /**
   * Set the motorcycle_backward flag.
   * @param  motorcycle_backward  Can a motorcycle drive in the
   *                              reverse direction?
   */
  void set_motorcycle_backward(const bool motorcycle_backward) {
    motorcycle_backward_ = motorcycle_backward;
  }

  /**
   * Get the motorcycle backward flag.
   * @return  Returns motorcycle backward flag.
   */
  bool motorcycle_backward() const {
    return motorcycle_backward_;
  }

  /**
   * Sets the pedestrian backward flag.
   * @param  pedestrian_backward   Are pedestrians allowed in the reverse direction?
   */
  void set_pedestrian_backward(const bool pedestrian_backward) {
    pedestrian_backward_ = pedestrian_backward;
  }

  /**
   * Get the pedestrian backward flag.
   * @return  Returns pedestrian backward flag.
   */
  bool pedestrian_backward() const {
    return pedestrian_backward_;
  }

  /**
   * Sets the destination_only flag.
   * @param  destination_only   Is private?
   */
  void set_destination_only(const bool destination_only) {
    destination_only_ = destination_only;
  }

  /**
   * Get the destination only/private flag.
   * @return  Returns private flag.
   */
  bool destination_only() const {
    return destination_only_;
  }

  /**
   * Sets the has_user_tags flag.
   * @param  has_user_tags   Did a user enter the access tags?
   */
  void set_has_user_tags(const bool has_user_tags) {
    has_user_tags_ = has_user_tags;
  }

  /**
   * Get the has_user_tags flag.
   * @return  Returns has_user_tags flag.
   */
  bool has_user_tags() const {
    return has_user_tags_;
  }

  /**
   * Sets the has_pronunciation_tags flag.
   * @param  has_pronunciation_tags  Do pronunciation tags exist?
   */
  void set_has_pronunciation_tags(const bool has_pronunciation_tags) {
    has_pronunciation_tags_ = has_pronunciation_tags;
  }

  /**
   * Get the has_pronunciation_tags flag.
   * @return  Returns has_pronunciation_tags flag.
   */
  bool has_pronunciation_tags() const {
    return has_pronunciation_tags_;
  }

  /**
   * Sets the internal flag.
   * @param  internal   Is this part of a internal intersection?
   */
  void set_internal(const bool internal) {
    internal_ = internal;
  }

  /**
   * Get the internal flag.
   * @return  Returns internal flag.
   */
  bool internal() const {
    return internal_;
  }

  /**
   * Sets the no thru traffic flag.
   * @param  no_thru_traffic   Traffic allowed?
   */
  void set_no_thru_traffic(const bool no_thru_traffic) {
    no_thru_traffic_ = no_thru_traffic;
  }

  /**
   * Get the no thru traffic flag.
   * @return  Returns no thru traffic flag.
   */
  bool no_thru_traffic() const {
    return no_thru_traffic_;
  }

  /**
   * Sets the oneway flag.
   * @param  oneway   Is oneway?
   */
  void set_oneway(const bool oneway) {
    oneway_ = oneway;
  }

  /**
   * Get the oneway flag.
   * @return  Returns oneway flag.
   */
  bool oneway() const {
    return oneway_;
  }

  /**
   * Sets if the oneway is in the opposite direction
   * @param  oneway_reverse
   */
  void set_oneway_reverse(const bool oneway_reverse) {
    oneway_reverse_ = oneway_reverse;
  }

  /**
   * Gets if the oneway is in the opposite direction
   * @return  Returns if oneway is reversed
   */
  bool oneway_reverse() const {
    return oneway_reverse_;
  }

  /**
   * Sets the roundabout flag.
   * @param  roundabout   Is a roundabout?
   */
  void set_roundabout(const bool roundabout) {
    roundabout_ = roundabout;
  }

  /**
   * Get the roundabout flag.
   * @return  Returns roundabout flag.
   */
  bool roundabout() const {
    return roundabout_;
  }

  /**
   * Sets the ferry flag.
   * @param  ferry   Is a ferry?
   */
  void set_ferry(const bool ferry) {
    ferry_ = ferry;
  }

  /**
   * Get the ferry flag.
   * @return  Returns ferry flag.
   */
  bool ferry() const {
    return ferry_;
  }

  /**
   * Sets the rail flag.
   * @param  rail   Is a auto train?
   */
  void set_rail(const bool rail) {
    rail_ = rail;
  }

  /**
   * Get the rail flag.
   * @return  Returns rail flag.
   */
  bool rail() const {
    return rail_;
  }

  /**
   * Sets the surface.
   * @param  surface
   */
  void set_surface(const baldr::Surface surface) {
    surface_ = static_cast<uint8_t>(surface);
  }

  /**
   * Get the surface.
   * @return  Returns Surface.
   */
  baldr::Surface surface() const {
    return static_cast<baldr::Surface>(surface_);
  }

  /**
   * Sets the sac scale.
   * @param  sac_scale
   */
  void set_sac_scale(const baldr::SacScale sac_scale) {
    sac_scale_ = static_cast<uint8_t>(sac_scale);
  }

  /**
   * Gets the sac scale.
   * @return  Returns sac_scale
   */
  baldr::SacScale sac_scale() const {
    return static_cast<baldr::SacScale>(sac_scale_);
  }

  /**
   * Sets the right cycle lane.
   * @param  cyclelane
   */
  void set_cyclelane_right(const baldr::CycleLane cyclelane) {
    cycle_lane_right_ = static_cast<uint8_t>(cyclelane);
  }

  /**
   * Gets the right cycle lane.
   * @return  Returns CycleLane on right.
   */
  baldr::CycleLane cyclelane_right() const {
    return static_cast<baldr::CycleLane>(cycle_lane_right_);
  }

  /**
   * Sets the left cycle lane.
   * @param  cyclelane
   */
  void set_cyclelane_left(const baldr::CycleLane cyclelane) {
    cycle_lane_left_ = static_cast<uint8_t>(cyclelane);
  }

  /**
   * Gets the left cycle lane.
   * @return  Returns CycleLane on left.
   */
  baldr::CycleLane cyclelane_left() const {
    return static_cast<baldr::CycleLane>(cycle_lane_left_);
  }

  /**
   * Sets if the right cycle lane is facing the opposite direction
   * @param  cyclelane_opposite
   */
  void set_cyclelane_right_opposite(const bool cyclelane_opposite) {
    cycle_lane_right_opposite_ = cyclelane_opposite;
  }

  /**
   * Gets if the right cycle lane is facing the opposite direction
   * @return  Returns cycle_lane_right_opposite
   */
  bool cyclelane_right_opposite() const {
    return cycle_lane_right_opposite_;
  }

  /**
   * Sets if the left cycle lane is facing the opposite direction
   * @param  cyclelane_opposite
   */
  void set_cyclelane_left_opposite(const bool cyclelane_opposite) {
    cycle_lane_left_opposite_ = cyclelane_opposite;
  }

  /**
   * Gets if the left cycle lane is facing the opposite direction
   * @return  Returns cycle_lane_left_opposite
   */
  bool cyclelane_left_opposite() const {
    return cycle_lane_left_opposite_;
  }

  /**
   * Set if edge has a shoulder on the right
   * @param  shoulder  True if edge has shoulder on right
   */
  void set_shoulder_right(const bool shoulder_right) {
    shoulder_right_ = shoulder_right;
  }

  /**
   * Get if edge has a shoulder on the right
   * @return  Returns if edge has a shoulder on right
   */
  bool shoulder_right() const {
    return shoulder_right_;
  }

  /**
   * Set if edge has a shoulder on the left
   * @param  shoulder  True if edge has shoulder on left
   */
  void set_shoulder_left(const bool shoulder_left) {
    shoulder_left_ = shoulder_left;
  }

  /**
   * Get if edge has a shoulder on the left
   * @return  Returns if edge has a shoulder on left
   */
  bool shoulder_left() const {
    return shoulder_left_;
  }

  /**
   * Sets if a bicyclist needs to dismount their bike
   * @param  dismount  Whether a cyclist needs to dismount or not
   */
  void set_dismount(const bool dismount) {
    dismount_ = dismount;
  }

  /**
   * Gets if a bicyclist needs to dismount their bike
   * @return  Returns dismount
   */
  bool dismount() const {
    return dismount_;
  }

  /**
   * Sets whether a pedestrian or cyclist should have preference to use a different
   * path to the side (A separate OSMWay completely)
   * @param  use_sidepath
   */
  void set_use_sidepath(const bool use_sidepath) {
    use_sidepath_ = use_sidepath;
  }

  /*
   * Gets whether a pedestrian or cyclist should have preference to use a different
   * path to the side (A separate OSMWay completely)
   * @return  Returns if using a sidepath is preffered
   */
  bool use_sidepath() const {
    return use_sidepath_;
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
    return lanes_;
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
    return backward_lanes_;
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
    return forward_lanes_;
  }

  /**
   * Sets the tunnel flag.
   * @param  tunnel   Is a tunnel road?
   */
  void set_tunnel(const bool tunnel) {
    tunnel_ = tunnel;
  }

  /**
   * Get the tunnel flag.
   * @return  Returns tunnel flag.
   */
  bool tunnel() const {
    return tunnel_;
  }

  /**
   * Sets the toll flag.
   * @param  toll   Is a toll road?
   */
  void set_toll(const bool toll) {
    toll_ = toll;
  }

  /**
   * Get the toll flag.
   * @return  Returns toll flag.
   */
  bool toll() const {
    return toll_;
  }

  /**
   * Sets the bridge flag.
   * @param  bridge   Is a bridge?
   */
  void set_bridge(const bool bridge) {
    bridge_ = bridge;
  }

  /**
   * Get the bridge flag.
   * @return  Returns bridge flag.
   */
  bool bridge() const {
    return bridge_;
  }

  /**
   * Sets the indoor flag.
   * @param  indoor   True if the edge is indoor, false if not (outdoor).
   */
  void set_indoor(const bool indoor) {
    indoor_ = indoor;
  }

  /**
   * Get the indoor flag.
   * @return  Returns indoor flag.
   */
  bool indoor() const {
    return indoor_;
  }

  /**
   * Sets the HOV Type.
   * @param  hov_type
   */
  void set_hov_type(const baldr::HOVEdgeType hov_type) {
    hov_type_ = static_cast<uint8_t>(hov_type);
  }

  /**
   * Get the hov_type flag.
   * @return  Returns hov_type flag.
   */
  baldr::HOVEdgeType hov_type() const {
    return static_cast<baldr::HOVEdgeType>(hov_type_);
  }

  /**
   * Set seasonal flag.
   * @param  seasonal   Is this seasonal?
   */
  void set_seasonal(const bool seasonal) {
    seasonal_ = seasonal;
  }

  /**
   * Get the seasonal flag.
   * @return  Returns seasonal flag.
   */
  bool seasonal() const {
    return seasonal_;
  }

  /**
   * Set wheelchair flag.
   * @param  wheelchair   Is this wheelchair?
   */
  void set_wheelchair(const bool wheelchair) {
    wheelchair_ = wheelchair;
  }

  /**
   * Get the wheelchair flag.
   * @return  Returns wheelchair flag.
   */
  bool wheelchair() const {
    return wheelchair_;
  }

  /**
   * Set wheelchair_tag flag.
   * @param  wheelchair_tag   Did the user set the wheelchair_tag?
   */
  void set_wheelchair_tag(const bool wheelchair_tag) {
    wheelchair_tag_ = wheelchair_tag;
  }

  /**
   * Get the wheelchair_tag flag.
   * @return  Returns wheelchair_tag flag.
   */
  bool wheelchair_tag() const {
    return wheelchair_tag_;
  }

  /**
   * Set sidewalk_left flag.
   * @param  sidewalk_left   Is there a sidewalk on the left?
   */
  void set_sidewalk_left(const bool sidewalk_left) {
    sidewalk_left_ = sidewalk_left;
  }

  /**
   * Get the sidewalk_left flag.
   * @return  Returns sidewalk_left flag.
   */
  bool sidewalk_left() const {
    return sidewalk_left_;
  }

  /**
   * Set sidewalk_right flag.
   * @param  sidewalk_right   Is there a sidewalk on the right?
   */
  void set_sidewalk_right(const bool sidewalk_right) {
    sidewalk_right_ = sidewalk_right;
  }

  /**
   * Get the sidewalk_right flag.
   * @return  Returns sidewalk_right flag.
   */
  bool sidewalk_right() const {
    return sidewalk_right_;
  }

  /**
   * Set drive_on_right flag.
   * @param  drive_on_right   Is a country that we drive on the right?
   */
  void set_drive_on_right(const bool drive_on_right) {
    drive_on_right_ = drive_on_right;
  }

  /**
   * Get the drive on right flag.
   * @return  Returns drive on right flag.
   */
  bool drive_on_right() const {
    return drive_on_right_;
  }

  /**
   * Sets the bike network.
   * @param  bike_network Mask of the bike networks (ncn/rcn/lcn).
   */
  void set_bike_network(const uint32_t bike_network) {
    bike_network_ = bike_network;
  }

  /**
   * Get the bike network mask.
   * @return  Returns the bike network mask.
   */
  uint32_t bike_network() const {
    return bike_network_;
  }

  /**
   * Sets the exit tag.
   * @param  exit       Exit flag.
   */
  void set_exit(const bool exit) {
    exit_ = exit;
  }

  /**
   * Get the exit flag.
   * @return  Returns exit flag.
   */
  bool exit() const {
    return exit_;
  }

  /**
   * Sets the tagged_speed flag.
   * @param  tagged_speed  User specified speed?
   */
  void set_tagged_speed(const bool tagged_speed) {
    tagged_speed_ = tagged_speed;
  }

  /**
   * Get the tagged_speed flag.
   * @return  Returns tagged_speed flag.
   */
  bool tagged_speed() const {
    return tagged_speed_;
  }

  /**
   * Sets the forward tagged_speed flag.
   * @param  forward_tagged_speed  User specified speed?
   */
  void set_forward_tagged_speed(const bool forward_tagged_speed) {
    forward_tagged_speed_ = forward_tagged_speed;
  }

  /**
   * Get the forward_tagged_speed flag.
   * @return  Returns forward_tagged_speed flag.
   */
  bool forward_tagged_speed() const {
    return forward_tagged_speed_;
  }

  /**
   * Sets the backward tagged_speed flag.
   * @param  backward_tagged_speed  User specified speed?
   */
  void set_backward_tagged_speed(const bool backward_tagged_speed) {
    backward_tagged_speed_ = backward_tagged_speed;
  }

  /**
   * Get the backward_tagged_speed flag.
   * @return  Returns backward_tagged_speed flag.
   */
  bool backward_tagged_speed() const {
    return backward_tagged_speed_;
  }

  /**
   * Sets the tagged_lanes flag.
   * @param  tagged_lanes  User specified lanes?
   */
  void set_tagged_lanes(const bool tagged_lanes) {
    tagged_lanes_ = tagged_lanes;
  }

  /**
   * Get the tagged_lanes flag.
   * @return  Returns tagged_lanes flag.
   */
  bool tagged_lanes() const {
    return tagged_lanes_;
  }

  /**
   * Sets the forward tagged_lanes flag.
   * @param  forward_tagged_lanes  User specified lanes?
   */
  void set_forward_tagged_lanes(const bool forward_tagged_lanes) {
    forward_tagged_lanes_ = forward_tagged_lanes;
  }

  /**
   * Get the forward_tagged_lanes flag.
   * @return  Returns forward_tagged_lanes flag.
   */
  bool forward_tagged_lanes() const {
    return forward_tagged_lanes_;
  }

  /**
   * Sets the backward tagged_lanes flag.
   * @param  backward_tagged_lanes  User specified lanes?
   */
  void set_backward_tagged_lanes(const bool backward_tagged_lanes) {
    backward_tagged_lanes_ = backward_tagged_lanes;
  }

  /**
   * Get the backward_tagged_lanes flag.
   * @return  Returns backward_tagged_lanes flag.
   */
  bool backward_tagged_lanes() const {
    return backward_tagged_lanes_;
  }

  /**
   * Sets the truck route flag.
   * @param  truck_route  Is this part of the local, national,
   *                      or state truck network or designated truck way?
   */
  void set_truck_route(const bool truck_route) {
    truck_route_ = truck_route;
  }

  /**
   * Get the truck_route flag.
   * @return  Returns truck_route flag.
   */
  bool truck_route() const {
    return truck_route_;
  }

  /**
   * Sets the road class.
   * @param  roadclass  Road Class/highway type.
   */
  void set_road_class(const baldr::RoadClass roadclass) {
    road_class_ = static_cast<uint8_t>(roadclass);
  }

  /**
   * Get the road class.
   * @return  Returns road class.
   */
  baldr::RoadClass road_class() const {
    return static_cast<baldr::RoadClass>(road_class_);
  }

  /**
   * Sets the use tag.
   * @param  use       use. None Cycleway ParkingAisle, Driveway, Alley,
   *                        EmergencyAccess, DriveThru, Steps, and Other
   */
  void set_use(const baldr::Use use) {
    use_ = static_cast<uint8_t>(use);
  }

  /**
   * Get the use.
   * @return  Returns use.
   */
  baldr::Use use() const {
    return static_cast<baldr::Use>(use_);
  }

  /**
   * Sets the link tag.
   * @param  link       Link.  Ramp or turn channel.
   */
  void set_link(const bool link) {
    link_ = link;
  }

  /**
   * Get the link flag.
   * @return  Returns link flag.
   */
  bool link() const {
    return link_;
  }

  /**
   * Sets the turn channel tag.
   * @param  turn channel       Turn channel.
   */
  void set_turn_channel(const bool turn_channel) {
    turn_channel_ = turn_channel;
  }

  /**
   * Get the turn channel flag.
   * @return  Returns turn channel flag.
   */
  bool turn_channel() const {
    return turn_channel_;
  }

  void AddPronunciations(std::vector<std::string>& pronunciations,
                         const UniqueNames& name_offset_map,
                         const uint32_t ipa_index,
                         const uint32_t nt_sampa_index,
                         const uint32_t katakana_index,
                         const uint32_t jeita_index,
                         const size_t name_tokens_size,
                         const size_t key) const;

  /**
   * Sets layer index(Z-level) of the way.
   * @param layer
   */
  void set_layer(int8_t layer);

  /**
   * Get layer(Z-level), can be negative.
   * @return returns layer index of the way relatively to other ways.
   */
  int8_t layer() const {
    return layer_;
  }

  /**
   * Sets the index for level
   * @param  idx  Index for the level.
   */
  void set_level_index(const uint32_t idx) {
    level_index_ = idx;
  }

  /**
   * Get the level index.
   * @return  Returns the index for the level.
   */
  uint32_t level_index() const {
    return level_index_;
  }

  /**
   * Sets the index for level_ref
   * @param  idx  Index for the level_ref
   */
  void set_level_ref_index(const uint32_t idx) {
    level_ref_index_ = idx;
  }

  /**
   * Get the level_ref index.
   * @return  Returns the index for the level_ref.
   */
  uint32_t level_ref_index() const {
    return level_ref_index_;
  }

  /**
   * Sets the lit state
   *
   * @param lit whether the way is lit.
   */
  void set_lit(const bool lit) {
    lit_ = lit;
  }

  /**
   * Get the lit state.
   *
   * @return bool
   */
  bool lit() const {
    return lit_;
  }

  /**
   * Get the names for the edge info based on the road class.
   * @param  ref              updated refs from relations.
   * @param  name_offset_map  map of unique names and refs from ways.
   * @return  Returns vector of strings
   */
  void GetNames(const std::string& ref,
                const UniqueNames& name_offset_map,
                const OSMPronunciation& pronunciation,
                uint16_t& types,
                std::vector<std::string>& names,
                std::vector<std::string>& pronunciations) const;

  void GetTaggedValues(const UniqueNames& name_offset_map,
                       const OSMPronunciation& pronunciation,
                       const size_t& names_size,
                       std::vector<std::string>& names,
                       std::vector<std::string>& pronunciations) const;

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
  uint32_t tunnel_name_index_;

  // Turn lanes
  uint32_t fwd_turn_lanes_index_;
  uint32_t bwd_turn_lanes_index_;

  // Guidance views
  uint32_t fwd_jct_base_index_;
  uint32_t bwd_jct_base_index_;

  uint32_t fwd_jct_overlay_index_;
  uint32_t bwd_jct_overlay_index_;

  uint32_t fwd_signboard_base_index_;
  uint32_t bwd_signboard_base_index_;

  // Sign Destination information
  uint32_t destination_index_;
  uint32_t destination_forward_index_;
  uint32_t destination_backward_index_;
  uint32_t destination_ref_index_;
  uint32_t destination_ref_to_index_;
  uint32_t destination_street_index_;
  uint32_t destination_street_to_index_;
  uint32_t junction_ref_index_;

  // level and level:ref of the way
  uint32_t level_index_;
  uint32_t level_ref_index_;

  // Bike network information. TODO - these are not yet used.
  //  uint32_t bike_national_ref_index_;
  //  uint32_t bike_regional_ref_index_;
  //  uint32_t bike_local_ref_index_;

  // duration of a ferry in seconds
  uint32_t duration_;

  // Way attributes
  uint32_t destination_only_ : 1;
  uint32_t no_thru_traffic_ : 1;
  uint32_t oneway_ : 1;
  uint32_t oneway_reverse_ : 1;
  uint32_t roundabout_ : 1;
  uint32_t ferry_ : 1;
  uint32_t rail_ : 1;
  uint32_t surface_ : 3;
  uint32_t tunnel_ : 1;
  uint32_t toll_ : 1;
  uint32_t bridge_ : 1;
  uint32_t seasonal_ : 1;
  uint32_t drive_on_right_ : 1;
  uint32_t bike_network_ : 4;
  uint32_t exit_ : 1;
  uint32_t tagged_speed_ : 1;
  uint32_t forward_tagged_speed_ : 1;
  uint32_t backward_tagged_speed_ : 1;
  uint32_t tagged_lanes_ : 1;
  uint32_t forward_tagged_lanes_ : 1;
  uint32_t backward_tagged_lanes_ : 1;
  uint32_t truck_route_ : 1;
  uint32_t sidewalk_right_ : 1;
  uint32_t sidewalk_left_ : 1;
  uint32_t sac_scale_ : 3;

  // Classification
  uint32_t road_class_ : 3; // Importance of the road/path
  uint32_t link_ : 1;       // *link tag - Ramp or turn channel
  uint32_t use_ : 6;        // Use / form
  uint32_t lanes_ : 4;
  uint32_t forward_lanes_ : 4;
  uint32_t backward_lanes_ : 4;
  uint32_t turn_channel_ : 1; // *link tag - turn channel (no ramp)
  uint32_t wheelchair_ : 1;
  uint32_t wheelchair_tag_ : 1;
  uint32_t has_user_tags_ : 1;
  uint32_t has_pronunciation_tags_ : 1;
  uint32_t internal_ : 1;
  uint32_t hov_type_ : 1;
  uint32_t indoor_ : 1;
  uint32_t pedestrian_forward_ : 1;
  uint32_t pedestrian_backward_ : 1;

  // Access
  uint16_t auto_forward_ : 1;
  uint16_t bus_forward_ : 1;
  uint16_t taxi_forward_ : 1;
  uint16_t truck_forward_ : 1;
  uint16_t motorcycle_forward_ : 1;
  uint16_t emergency_forward_ : 1;
  uint16_t hov_forward_ : 1;
  uint16_t moped_forward_ : 1;
  uint16_t auto_backward_ : 1;
  uint16_t bus_backward_ : 1;
  uint16_t taxi_backward_ : 1;
  uint16_t truck_backward_ : 1;
  uint16_t motorcycle_backward_ : 1;
  uint16_t emergency_backward_ : 1;
  uint16_t hov_backward_ : 1;
  uint16_t moped_backward_ : 1;

  // Attributes specific to biking
  uint16_t cycle_lane_right_ : 2;
  uint16_t cycle_lane_left_ : 2;
  uint16_t cycle_lane_right_opposite_ : 1;
  uint16_t cycle_lane_left_opposite_ : 1;
  uint16_t shoulder_right_ : 1;
  uint16_t shoulder_left_ : 1;
  uint16_t dismount_ : 1;
  uint16_t use_sidepath_ : 1;
  uint16_t bike_forward_ : 1;
  uint16_t bike_backward_ : 1;
  bool lit_ : 1;
  uint16_t spare2_ : 3;

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

  // layer index(Z-level) of the way relatively to other levels
  int8_t layer_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMWAY_H
