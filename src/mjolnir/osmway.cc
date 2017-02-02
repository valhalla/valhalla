#include "mjolnir/osmway.h"
#include "mjolnir/util.h"

#include <iostream>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace {

constexpr uint32_t kMaxNodesPerWay = 65535;

}

namespace valhalla {
namespace mjolnir {

// Set way id.
void OSMWay::set_way_id(const uint64_t id) {
  osmwayid_ = id;
}

// Get the way id
uint64_t OSMWay::way_id() const {
  return osmwayid_;
}

// Set the number of nodes for this way.
void OSMWay::set_node_count(const uint32_t count) {
  if (count > kMaxNodesPerWay) {
    LOG_ERROR("Exceeded max nodes per way: " + std::to_string(count));
    nodecount_ = static_cast<uint16_t>(kMaxNodesPerWay);
  } else {
    nodecount_ = static_cast<uint16_t>(count);
  }
}

// Get the number of nodes for this way.
uint32_t OSMWay::node_count() const {
  return nodecount_;
}

// Sets the speed in KPH.
void OSMWay::set_speed(const float speed) {
  speed_ = static_cast<unsigned char>(speed + 0.5f);
}

// Gets the speed in KPH.
float OSMWay::speed() const {
  return static_cast<float>(speed_);
}

// Sets the speed limit in KPH.
void OSMWay::set_speed_limit(const float speed_limit) {
  speed_limit_ = static_cast<unsigned char>(speed_limit + 0.5f);
}

// Gets the speed limit in KPH.
float OSMWay::speed_limit() const {
  return static_cast<float>(speed_limit_);
}

// Sets the backward speed in KPH.
void OSMWay::set_backward_speed(const float backward_speed) {
  backward_speed_ = static_cast<unsigned char>(backward_speed + 0.5f);
}

// Gets the backward speed in KPH.
float OSMWay::backward_speed() const {
  return static_cast<float>(backward_speed_);
}

// Sets the backward speed in KPH.
void OSMWay::set_forward_speed(const float forward_speed) {
  forward_speed_ = static_cast<unsigned char>(forward_speed + 0.5f);
}

// Gets the backward speed in KPH.
float OSMWay::forward_speed() const {
  return static_cast<float>(forward_speed_);
}

// Sets the truck speed in KPH.
void OSMWay::set_truck_speed(const float speed) {
  truck_speed_ = static_cast<unsigned char>(speed + 0.5f);
}

// Gets the truck speed in KPH.
float OSMWay::truck_speed() const {
  return static_cast<float>(truck_speed_);
}

// Set the index for the ref.
void OSMWay::set_ref_index(const uint32_t idx) {
  ref_index_ = idx;
}

// Get the ref.
uint32_t OSMWay::ref_index() const {
  return ref_index_;
}

// Set the index for the int ref.
void OSMWay::set_int_ref_index(const uint32_t idx) {
  int_ref_index_ = idx;
}

// Get the int ref.
uint32_t OSMWay::int_ref_index() const {
  return int_ref_index_;
}

// Set the index for the name.
void OSMWay::set_name_index(const uint32_t idx) {
  name_index_ = idx;
}

// Get the name.
uint32_t OSMWay::name_index() const {
  return name_index_;
}

// Set the index for the name:en.
void OSMWay::set_name_en_index(const uint32_t idx) {
  name_en_index_ = idx;
}

// Get the name:en.
uint32_t OSMWay::name_en_index() const {
  return name_en_index_;
}

// Set the index for the alt name.
void OSMWay::set_alt_name_index(const uint32_t idx) {
  alt_name_index_ = idx;
}

// Get the alt name.
uint32_t OSMWay::alt_name_index() const {
  return alt_name_index_;
}

// Set the index for the official name.
void OSMWay::set_official_name_index(const uint32_t idx) {
  official_name_index_ = idx;
}

// Get the official name.
uint32_t OSMWay::official_name_index() const {
  return official_name_index_;
}

// Set the index for the destination.
void OSMWay::set_destination_index(const uint32_t idx) {
  destination_index_ = idx;
}

// Get the get_destination.
uint32_t OSMWay::destination_index() const {
  return destination_index_;
}

// Set the index for thedestination_ref.
void OSMWay::set_destination_ref_index(const uint32_t idx) {
  destination_ref_index_ = idx;
}

// Get the destination_ref.
uint32_t OSMWay::destination_ref_index() const {
  return destination_ref_index_;
}

// Set the index for the destination_ref_to.
void OSMWay::set_destination_ref_to_index(const uint32_t idx) {
  destination_ref_to_index_ = idx;
}

// Get the destination ref to.
uint32_t OSMWay::destination_ref_to_index() const {
  return destination_ref_to_index_;
}

// Set the index for thedestination_street.
void OSMWay::set_destination_street_index(const uint32_t idx) {
  destination_street_index_ = idx;
}

// Get the destination_street.
uint32_t OSMWay::destination_street_index() const {
  return destination_street_index_;
}

// Set the index for the destination_street_to.
void OSMWay::set_destination_street_to_index(const uint32_t idx) {
  destination_street_to_index_ = idx;
}

// Get the destination street to.
uint32_t OSMWay::destination_street_to_index() const {
  return destination_street_to_index_;
}

// Set the index for the junction_ref.
void OSMWay::set_junction_ref_index(const uint32_t idx) {
  junction_ref_index_ = idx;
}

// Get the junction ref.
uint32_t OSMWay::junction_ref_index() const {
  return junction_ref_index_;
}

// Set the index for the bike national ref.
void OSMWay::set_bike_national_ref_index(const uint32_t idx) {
  bike_national_ref_index_ = idx;
}

// Get the bike national ref.
uint32_t OSMWay::bike_national_ref_index() const {
  return bike_national_ref_index_;
}

// Set the index for the bike regional ref.
void OSMWay::set_bike_regional_ref_index(const uint32_t idx) {
  bike_regional_ref_index_ = idx;
}

// Get the bike regional ref.
uint32_t OSMWay::bike_regional_ref_index() const {
  return bike_regional_ref_index_;
}

// Set the index for the bike local ref.
void OSMWay::set_bike_local_ref_index(const uint32_t idx) {
  bike_local_ref_index_ = idx;
}

// Get the bike local ref.
uint32_t OSMWay::bike_local_ref_index() const {
  return bike_local_ref_index_;
}

// Set auto forward flag.
void OSMWay::set_auto_forward(const bool auto_forward) {
  access_.fields.auto_forward = auto_forward;
}

// Get the auto forward flag.
bool OSMWay::auto_forward() const {
  return access_.fields.auto_forward;
}

// Set bus forward flag.
void OSMWay::set_bus_forward(const bool bus_forward) {
  access_.fields.bus_forward = bus_forward;
}

// Get the bus forward flag.
bool OSMWay::bus_forward() const {
  return access_.fields.bus_forward;
}

// Set taxi forward flag.
void OSMWay::set_taxi_forward(const bool taxi_forward) {
  access_.fields.taxi_forward = taxi_forward;
}

// Get the taxi forward flag.
bool OSMWay::taxi_forward() const {
  return access_.fields.taxi_forward;
}

// Set hov forward flag.
void OSMWay::set_hov_forward(const bool hov_forward) {
  access_.fields.hov_forward = hov_forward;
}

// Get the hov forward flag.
bool OSMWay::hov_forward() const {
  return access_.fields.hov_forward;
}

// Set truck forward flag.
void OSMWay::set_truck_forward(const bool truck_forward) {
  access_.fields.truck_forward = truck_forward;
}

// Get the truck forward flag.
bool OSMWay::truck_forward() const {
  return access_.fields.truck_forward;
}

// Set bike forward flag.
void OSMWay::set_bike_forward(const bool bike_forward) {
  access_.fields.bike_forward = bike_forward;
}

// Get the bike forward flag.
bool OSMWay::bike_forward() const {
  return access_.fields.bike_forward;
}

// Set emergency forward flag.
void OSMWay::set_emergency_forward(const bool emergency_forward) {
  access_.fields.emergency_forward = emergency_forward;
}

// Get the emergency forward flag.
bool OSMWay::emergency_forward() const {
  return access_.fields.emergency_forward;
}

// Set auto backward flag.
void OSMWay::set_auto_backward(const bool auto_backward) {
  access_.fields.auto_backward = auto_backward;
}

// Get the auto backward flag.
bool OSMWay::auto_backward() const {
  return access_.fields.auto_backward;
}

// Set bus backward flag.
void OSMWay::set_bus_backward(const bool bus_backward) {
  access_.fields.bus_backward = bus_backward;
}

// Get the bus backward flag.
bool OSMWay::bus_backward() const {
  return access_.fields.bus_backward;
}

// Set taxi backward flag.
void OSMWay::set_taxi_backward(const bool taxi_backward) {
  access_.fields.taxi_backward = taxi_backward;
}

// Get the taxi backward flag.
bool OSMWay::taxi_backward() const {
  return access_.fields.taxi_backward;
}

// Set hov backward flag.
void OSMWay::set_hov_backward(const bool hov_backward) {
  access_.fields.hov_backward = hov_backward;
}

// Get the hov backward flag.
bool OSMWay::hov_backward() const {
  return access_.fields.hov_backward;
}

// Set truck backward flag.
void OSMWay::set_truck_backward(const bool truck_backward) {
  access_.fields.truck_backward = truck_backward;
}

// Get the truck backward flag.
bool OSMWay::truck_backward() const {
  return access_.fields.truck_backward;
}

// Set bike backward flag.
void OSMWay::set_bike_backward(const bool bike_backward) {
  access_.fields.bike_backward = bike_backward;
}

// Get the bike backward flag.
bool OSMWay::bike_backward() const {
  return access_.fields.bike_backward;
}

// Set emergency backward flag.
void OSMWay::set_emergency_backward(const bool emergency_backward) {
  access_.fields.emergency_backward = emergency_backward;
}

// Get the emergency backward flag.
bool OSMWay::emergency_backward() const {
  return access_.fields.emergency_backward;
}

// Set destination only/private flag.
void OSMWay::set_destination_only(const bool destination_only) {
  attributes_.fields.destination_only = destination_only;
}

// Get the destination only/private flag.
bool OSMWay::destination_only() const {
  return attributes_.fields.destination_only;
}

// Set pedestrian flag.
void OSMWay::set_pedestrian(const bool pedestrian) {
  classification_.fields.pedestrian = pedestrian;
}

// Get the pedestrian flag.
bool OSMWay::pedestrian() const {
  return classification_.fields.pedestrian;
}

// Sets the has_user_tags flag.
void OSMWay::set_has_user_tags(const bool has_user_tags) {
  classification_.fields.has_user_tags = has_user_tags;
}

// Get the has_user_tags flag.
bool OSMWay::has_user_tags() const {
  return classification_.fields.has_user_tags;
}

// Set no thru traffic flag.
void OSMWay::set_no_thru_traffic(const bool no_thru_traffic) {
  attributes_.fields.no_thru_traffic = no_thru_traffic;
}

// Get the no thru traffic flag.
bool OSMWay::no_thru_traffic() const {
  return attributes_.fields.no_thru_traffic;
}

// Set oneway flag.
void OSMWay::set_oneway(const bool oneway) {
  attributes_.fields.oneway = oneway;
}

// Get the oneway flag.
bool OSMWay::oneway() const{
  return attributes_.fields.oneway;
}

// Set roundabout flag.
void OSMWay::set_roundabout(const bool roundabout) {
  attributes_.fields.roundabout = roundabout;
}

// Get the roundabout flag.
bool OSMWay::roundabout() const {
  return attributes_.fields.roundabout;
}

// Set ferry flag.
void OSMWay::set_ferry(const bool ferry) {
  attributes_.fields.ferry = ferry;
}

// Get the ferry flag.
bool OSMWay::ferry() const {
  return attributes_.fields.ferry;
}

// Set rail flag.
void OSMWay::set_rail(const bool rail) {
  attributes_.fields.rail = rail;
}

// Get the rail flag.
bool OSMWay::rail() const {
  return attributes_.fields.rail;
}

// Set the surface.
void OSMWay::set_surface(const Surface surface) {
  attributes_.fields.surface = static_cast<uint8_t>(surface);
}

// Get the surface.
Surface OSMWay::surface() const {
  return static_cast<Surface>(attributes_.fields.surface);
}

// Set the cycle lane.
void OSMWay::set_cyclelane(const CycleLane cyclelane) {
  attributes_.fields.cycle_lane = static_cast<uint8_t>(cyclelane);
}

// Get the cycle lane.
CycleLane OSMWay::cyclelane() const {
  return static_cast<CycleLane>(attributes_.fields.cycle_lane);
}

// Sets the number of lanes
void OSMWay::set_lanes(const uint32_t lanes) {
  classification_.fields.lanes = lanes;
}

// Get the number of lanes
uint32_t OSMWay::lanes() const {
  return classification_.fields.lanes;
}

// Sets the number of backward lanes
void OSMWay::set_backward_lanes(const uint32_t backward_lanes) {
  classification_.fields.backward_lanes = backward_lanes;
}

// Get the number of backward lanes
uint32_t OSMWay::backward_lanes() const {
  return classification_.fields.backward_lanes;
}

// Sets the number of forward lanes
void OSMWay::set_forward_lanes(const uint32_t forward_lanes) {
  classification_.fields.forward_lanes = forward_lanes;
}

// Get the number of forward lanes
uint32_t OSMWay::forward_lanes() const {
  return classification_.fields.forward_lanes;
}

// Set tunnel flag.
void OSMWay::set_tunnel(const bool tunnel) {
  attributes_.fields.tunnel = tunnel;
}

// Get the tunnel flag.
bool OSMWay::tunnel() const {
  return attributes_.fields.tunnel;
}

// Set toll flag.
void OSMWay::set_toll(const bool toll) {
  attributes_.fields.toll = toll;
}

// Get the toll flag.
bool OSMWay::toll() const {
  return attributes_.fields.toll;
}

// Set bridge flag.
void OSMWay::set_bridge(const bool bridge) {
  attributes_.fields.bridge = bridge;
}

// Get the bridge flag.
bool OSMWay::bridge() const {
  return attributes_.fields.bridge;
}

// Set seasonal flag.
void OSMWay::set_seasonal(const bool seasonal) {
  attributes_.fields.seasonal = seasonal;
}

// Get the seasonal flag.
bool OSMWay::seasonal() const {
  return attributes_.fields.seasonal;
}

// Set wheelchair flag.
void OSMWay::set_wheelchair(const bool wheelchair) {
  classification_.fields.wheelchair = wheelchair;
}

// Get the wheelchair flag.
bool OSMWay::wheelchair() const {
  return classification_.fields.wheelchair;
}

// Set wheelchair_tag flag.
void OSMWay::set_wheelchair_tag(const bool wheelchair_tag) {
  classification_.fields.wheelchair_tag = wheelchair_tag;
}

// Get the wheelchair_tag flag.
bool OSMWay::wheelchair_tag() const {
  return classification_.fields.wheelchair_tag;
}

// Set sidewalk left flag.
void OSMWay::set_sidewalk_left(const bool sidewalk_left) {
  attributes_.fields.sidewalk_left = sidewalk_left;
}

// Get the sidewalk left flag.
bool OSMWay::sidewalk_left() const {
  return attributes_.fields.sidewalk_left;
}

// Set sidewalk right flag.
void OSMWay::set_sidewalk_right(const bool sidewalk_right) {
  attributes_.fields.sidewalk_right = sidewalk_right;
}

// Get the sidewalk right flag.
bool OSMWay::sidewalk_right() const {
  return attributes_.fields.sidewalk_right;
}

// Set drive_on_right flag.
void OSMWay::set_drive_on_right(const bool drive_on_right) {
  attributes_.fields.drive_on_right = drive_on_right;
}

// Get the drive on right flag.
bool OSMWay::drive_on_right() const {
  return attributes_.fields.drive_on_right;
}

//Sets the bike network mask
void OSMWay::set_bike_network(const uint32_t bikenetwork) {
  attributes_.fields.bike_network = bikenetwork;
}

//Get the bike network mask
uint32_t OSMWay::bike_network() const {
  return attributes_.fields.bike_network;
}

// Set exit flag.
void OSMWay::set_exit(const bool exit) {
  attributes_.fields.exit = exit;
}

// Get the exit flag.
bool OSMWay::exit() const {
  return attributes_.fields.exit;
}

// Sets the tagged_speed flag.
void  OSMWay::set_tagged_speed(const bool tagged_speed) {
  attributes_.fields.tagged_speed = tagged_speed;
}

// Get the tagged_speed flag.
bool  OSMWay::tagged_speed() const {
  return attributes_.fields.tagged_speed;
}

// Sets the tagged_forward speed flag.
void  OSMWay::set_forward_tagged_speed(const bool forward_tagged_speed) {
  attributes_.fields.forward_tagged_speed = forward_tagged_speed;
}

// Get the tagged_forward_speed flag.
bool  OSMWay::forward_tagged_speed() const {
  return attributes_.fields.forward_tagged_speed;
}

// Sets the tagged_backward speed flag.
void  OSMWay::set_backward_tagged_speed(const bool backward_tagged_speed) {
  attributes_.fields.backward_tagged_speed = backward_tagged_speed;
}

// Get the tagged_backward_speed flag.
bool  OSMWay::backward_tagged_speed() const {
  return attributes_.fields.backward_tagged_speed;
}

// Sets the tagged_lanes flag.
void  OSMWay::set_tagged_lanes(const bool tagged_lanes) {
  attributes_.fields.tagged_lanes = tagged_lanes;
}

// Get the tagged_lanes flag.
bool  OSMWay::tagged_lanes() const {
  return attributes_.fields.tagged_lanes;
}

// Sets the tagged_forward lanes flag.
void  OSMWay::set_forward_tagged_lanes(const bool forward_tagged_lanes) {
  attributes_.fields.forward_tagged_lanes = forward_tagged_lanes;
}

// Get the tagged_forward_lanes flag.
bool  OSMWay::forward_tagged_lanes() const {
  return attributes_.fields.forward_tagged_lanes;
}

// Sets the tagged_forward lanes flag.
void  OSMWay::set_backward_tagged_lanes(const bool backward_tagged_lanes) {
  attributes_.fields.backward_tagged_lanes = backward_tagged_lanes;
}

// Get the tagged_backward_speed flag.
bool  OSMWay::backward_tagged_lanes() const {
  return attributes_.fields.backward_tagged_lanes;
}

// Sets the truck route flag.
void OSMWay::set_truck_route(const bool truck_route) {
  attributes_.fields.truck_route = truck_route;
}

// Get the truck_route flag.
bool OSMWay::truck_route() const {
  return attributes_.fields.truck_route;
}

// Get the road class.
RoadClass OSMWay::road_class() const {
  return static_cast<RoadClass>(classification_.fields.road_class);
}

// Set the road class.
void OSMWay::set_road_class(const RoadClass roadclass) {
  classification_.fields.road_class = static_cast<uint8_t>(roadclass);
}

// Set the use.
void OSMWay::set_use(const Use use) {
  classification_.fields.use = static_cast<uint8_t>(use);
}

// Get the use.
Use OSMWay::use() const {
  return static_cast<Use>(classification_.fields.use);
}

// Set link flag.
void OSMWay::set_link(const bool link) {
  classification_.fields.link = link;
}

// Get the link flag.
bool OSMWay::link() const {
  return classification_.fields.link;
}

// Set turn channel flag.
void OSMWay::set_turn_channel(const bool turn_channel) {
  classification_.fields.turn_channel = turn_channel;
}

// Get the turn channel flag.
bool OSMWay::turn_channel() const {
  return classification_.fields.turn_channel;
}

// Get the names for the edge info based on the road class.
std::vector<std::string> OSMWay::GetNames(const std::string& ref,
                                          const UniqueNames& ref_offset_map,
                                          const UniqueNames& name_offset_map) const {
  std::vector<std::string> names;
  // Process motorway and trunk refs
  if ((ref_index_ != 0 || !ref.empty())
      && ((static_cast<RoadClass>(classification_.fields.road_class) == RoadClass::kMotorway)
          || (static_cast<RoadClass>(classification_.fields.road_class) == RoadClass::kTrunk))) {
    std::vector<std::string> tokens;

    if (!ref.empty())
      tokens = GetTagTokens(ref);// use updated refs from relations.
    else
      tokens = GetTagTokens(ref_offset_map.name(ref_index_));

    names.insert(names.end(), tokens.begin(), tokens.end());
  }

  // TODO int_ref

  // Process name
  if (name_index_ != 0)
    names.emplace_back(name_offset_map.name(name_index_));

  // Process non limited access refs
  if (ref_index_ != 0 && (static_cast<RoadClass>(classification_.fields.road_class) != RoadClass::kMotorway)
      && (static_cast<RoadClass>(classification_.fields.road_class) != RoadClass::kTrunk)) {
    std::vector<std::string> tokens;
    if (!ref.empty())
      tokens = GetTagTokens(ref);// use updated refs from relations.
    else
      tokens = GetTagTokens(ref_offset_map.name(ref_index_));
    names.insert(names.end(), tokens.begin(), tokens.end());
  }

  // Process alt_name
  if (alt_name_index_ != 0 &&
      alt_name_index_ != name_index_)
    names.emplace_back(name_offset_map.name(alt_name_index_));

  // Process official_name
  if (official_name_index_ != 0 &&
      official_name_index_ != name_index_ &&
      official_name_index_ != alt_name_index_)
    names.emplace_back(name_offset_map.name(official_name_index_));

  // Process name_en_
  // TODO: process country specific names
  if (name_en_index_ != 0 &&
      name_en_index_ != name_index_ &&
      name_en_index_ != alt_name_index_ &&
      name_en_index_ != official_name_index_)
    names.emplace_back(name_offset_map.name(name_en_index_));

  return names;
}

}
}
