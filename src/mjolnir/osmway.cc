#include "mjolnir/osmway.h"
#include "mjolnir/util.h"

#include <valhalla/midgard/logging.h>

namespace valhalla {
namespace mjolnir {

OSMWay::OSMWay()
  : noderef_index_(0), nodecount_(0), ref_index_(0), int_ref_index_(0),
    name_index_(0),name_en_index_(0), alt_name_index_(0), official_name_index_(0),
    destination_index_(0), destination_ref_index_(0), destination_ref_to_index_(0), junction_ref_index_(0),
    bike_national_ref_index_(0), bike_regional_ref_index_(0), bike_local_ref_index_(0),
    osmwayid_(std::numeric_limits<uint64_t>::max()) {

  attributes_.v = {0};
  classification_.v = {0};
  speed_ = static_cast<unsigned char>(0.0f);
}

OSMWay::OSMWay(uint64_t id)
  : noderef_index_(0), nodecount_(0), ref_index_(0), int_ref_index_(0),
    name_index_(0),name_en_index_(0), alt_name_index_(0), official_name_index_(0),
    destination_index_(0), destination_ref_index_(0), destination_ref_to_index_(0), junction_ref_index_(0),
    bike_national_ref_index_(0), bike_regional_ref_index_(0), bike_local_ref_index_(0) {

  osmwayid_ = id;

  attributes_.v = {0};
  classification_.v = {0};
  speed_ = static_cast<unsigned char>(0.0f);
}

OSMWay::~OSMWay() {
}

// Set way id.
void OSMWay::set_way_id(const uint64_t id) {
  osmwayid_ = id;
}

// Get the way id
uint64_t OSMWay::way_id() const {
  return osmwayid_;
}

// Set the index into the node references
void OSMWay::set_noderef_index(const uint32_t idx) {
  noderef_index_ = idx;
}

// Get the index into the node references
uint32_t OSMWay::noderef_index() const {
  return noderef_index_;
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
  return destination_ref_to_index_;
}

// Set the index for the destination_ref_to.
void OSMWay::set_destination_ref_to_index(const uint32_t idx) {
  destination_ref_to_index_ = idx;
}

// Get the destination ref to.
uint32_t OSMWay::destination_ref_to_index() const {
  return destination_ref_to_index_;
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
  return bike_local_ref_;
}

// Set auto forward flag.
void OSMWay::set_auto_forward(const bool auto_forward) {
  attributes_.fields.auto_forward = auto_forward;
}

// Get the auto forward flag.
bool OSMWay::auto_forward() const {
  return attributes_.fields.auto_forward;
}

// Set bike forward flag.
void OSMWay::set_bike_forward(const bool bike_forward) {
  attributes_.fields.bike_forward = bike_forward;
}

// Get the bike forward flag.
bool OSMWay::bike_forward() const {
  return attributes_.fields.bike_forward;
}

// Set auto backward flag.
void OSMWay::set_auto_backward(const bool auto_backward) {
  attributes_.fields.auto_backward = auto_backward;
}

// Get the auto backward flag.
bool OSMWay::auto_backward() const {
  return attributes_.fields.auto_backward;
}

// Set bike backward flag.
void OSMWay::set_bike_backward(const bool bike_backward) {
  attributes_.fields.bike_backward = bike_backward;
}

// Get the bike backward flag.
bool OSMWay::bike_backward() const {
  return attributes_.fields.bike_backward;
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
  attributes_.fields.pedestrian = pedestrian;
}

// Get the pedestrian flag.
bool OSMWay::pedestrian() const {
  return attributes_.fields.pedestrian;
}

// Set no thru traffic flag.
void OSMWay::set_no_thru_traffic(const bool no_thru_traffic) {
  attributes_.fields.no_thru_traffic;
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
  attributes_.fields.lanes = lanes;
}

// Get the number of lanes
uint32_t OSMWay::lanes() const {
  return attributes_.fields.lanes;
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

//Sets the bike network mask
void OSMWay::set_bike_network(const uint32_t bikenetwork) {
  attributes_.fields.bikenetwork = bikenetwork;
}

//Get the bike network mask
uint32_t OSMWay::bike_network() const {
  return attributes_.fields.bikenetwork;
}

// Set exit flag.
void OSMWay::set_exit(const bool exit) {
  attributes_.fields.exit = exit;
}

// Get the exit flag.
bool OSMWay::exit() const {
  return attributes_.fields.exit;
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

// Get the names for the edge info based on the road class.
std::vector<std::string> OSMWay::GetNames(const std::string& ref,
                                          const UniqueNames& ref_offset_map,
                                          const UniqueNames& name_offset_map) const {
  std::vector<std::string> names;
  // Process motorway and trunk refs
  if ((ref_ != 0 || !ref.empty())
      && ((static_cast<RoadClass>(classification_.fields.road_class) == RoadClass::kMotorway)
          || (static_cast<RoadClass>(classification_.fields.road_class) == RoadClass::kTrunk))) {
    std::vector<std::string> tokens;

    if (!ref.empty())
      tokens = GetTagTokens(ref);// use updated refs from relations.
    else
      tokens = GetTagTokens(ref_offset_map.name(ref_));

    names.insert(names.end(), tokens.begin(), tokens.end());
  }

  // TODO int_ref

  // Process name
  if (name_ != 0)
    names.emplace_back(name_offset_map.name(name_));

  // Process non limited access refs
  if (ref_ != 0 && (static_cast<RoadClass>(classification_.fields.road_class) != RoadClass::kMotorway)
      && (static_cast<RoadClass>(classification_.fields.road_class) != RoadClass::kTrunk)) {
    std::vector<std::string> tokens;
    if (!ref.empty())
      tokens = GetTagTokens(ref);// use updated refs from relations.
    else
      tokens = GetTagTokens(ref_offset_map.name(ref_));
    names.insert(names.end(), tokens.begin(), tokens.end());
  }

  // Process alt_name
  if (alt_name_ != 0)
    names.emplace_back(name_offset_map.name(alt_name_));

  // Process official_name
  if (official_name_ != 0)
    names.emplace_back(name_offset_map.name(official_name_));

  // Process name_en_
  // TODO: process country specific names
  if (name_en_ != 0)
    names.emplace_back(ref_offset_map.name(name_en_));

  return names;
}

}
}
