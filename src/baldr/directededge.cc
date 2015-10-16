#include "baldr/directededge.h"
#include <boost/functional/hash.hpp>

using namespace valhalla::baldr;

namespace {

json::MapPtr bike_network_json(uint8_t mask) {
  return json::map({
    {"national", static_cast<bool>(mask && kNcn)},
    {"regional", static_cast<bool>(mask && kRcn)},
    {"local", static_cast<bool>(mask && kLcn)},
    {"mountain", static_cast<bool>(mask && kMcn)},
  });
}

json::MapPtr access_json(uint32_t access) {
  return json::map({
    {"bicycle", static_cast<bool>(access && kBicycleAccess)},
    {"bus", static_cast<bool>(access && kBusAccess)},
    {"car", static_cast<bool>(access && kAutoAccess)},
    {"emergency", static_cast<bool>(access && kEmergencyAccess)},
    {"HOV", static_cast<bool>(access && kHOVAccess)},
    {"pedestrian", static_cast<bool>(access && kPedestrianAccess)},
    {"taxi", static_cast<bool>(access && kTaxiAccess)},
    {"truck", static_cast<bool>(access && kTruckAccess)},
  });
}

}

namespace valhalla {
namespace baldr {

// Default constructor
DirectedEdge::DirectedEdge()
    : dataoffsets_{},
      geoattributes_{0,6}, //grade of 6 means flat
      access_{},
      speed_(0),
      classification_{},
      attributes_{},
      turntypes_{},
      stopimpact_{},
      hierarchy_() {
}

// Gets the end node of this directed edge.
GraphId DirectedEdge::endnode() const {
  return endnode_;
}

// ------------------  Data offsets and flags for extended data -------------//

// Get the offset to the common edge information.
uint64_t DirectedEdge::edgeinfo_offset() const {
  return dataoffsets_.edgeinfo_offset;
}

// Get the general restriction or access condition (per mode) for this directed edge.
uint64_t DirectedEdge::access_restriction() const {
  return dataoffsets_.access_restriction;
}

// Does this directed edge have exit signs.
bool DirectedEdge::exitsign() const {
  return dataoffsets_.exitsign;
}

// ------------------------- Geographic attributes ------------------------- //

// Gets the length of the edge in meters.
uint32_t DirectedEdge::length() const {
  return geoattributes_.length;
}

// Gets the weighted grade factor (0-15).
uint32_t DirectedEdge::weighted_grade() const {
  return geoattributes_.weighted_grade;
}

// Get the road curvature factor. (0-15).
uint32_t DirectedEdge::curvature() const {
  return geoattributes_.curvature;
}

// -------------------------- Routing attributes --------------------------- //

// Is driving on the right hand side of the road along this edge?
bool DirectedEdge::drive_on_right() const {
  return attributes_.drive_on_right;
}

// Is this edge part of a ferry?
bool DirectedEdge::ferry() const {
  return attributes_.ferry;
}

// Is this edge part of a rail ferry?
bool DirectedEdge::railferry() const {
  return attributes_.railferry;
}

// Does this edge have a toll or is it part of a toll road?
bool DirectedEdge::toll() const {
  return attributes_.toll;
}

// Does this edge have a seasonal access (e.g., closed in the winter)?
bool DirectedEdge::seasonal() const {
  return attributes_.seasonal;
}

// Is this edge part of a private or no through road that allows access
// only if required to get to a destination?
bool DirectedEdge::destonly() const {
  return attributes_.dest_only;
}

// Is this edge part of a tunnel?
bool DirectedEdge::tunnel() const {
  return attributes_.tunnel;
}

// Is this edge part of a bridge?
bool DirectedEdge::bridge() const {
  return attributes_.bridge;
}

// Is this edge part of a roundabout?
bool DirectedEdge::roundabout() const {
  return attributes_.roundabout;
}

// Is this edge is unreachable by driving. This can happen if a driveable
// edge is surrounded by pedestrian only edges (e.g. in a city center) or
// is not properly connected to other edges.
bool DirectedEdge::unreachable() const {
  return attributes_.unreachable;
}

// A traffic signal occurs at the end of this edge.
bool DirectedEdge::traffic_signal() const {
  return attributes_.traffic_signal;
}

// Is this directed edge stored forward in edgeinfo (true) or
// reverse (false).
bool DirectedEdge::forward() const {
  return attributes_.forward;
}

// Does this edge lead into a no-thru region
bool DirectedEdge::not_thru() const {
  return attributes_.not_thru;
}

// Get the index of the opposing directed edge at the end node of this
// directed edge.
uint32_t DirectedEdge::opp_index() const {
  return attributes_.opp_index;
}

// Get the cycle lane type along this edge.
CycleLane DirectedEdge::cyclelane() const {
  return static_cast<CycleLane>(attributes_.cycle_lane);
}

// Gets the bike network mask
uint32_t DirectedEdge::bike_network() const {
  return attributes_.bike_network;
}

// Gets the truck network flag
bool DirectedEdge::truck_route() const {
  return attributes_.truck_route;
}

// Gets the lane count
uint32_t DirectedEdge::lanecount() const {
  return attributes_.lanecount;
}

// Get the mask of simple turn restrictions from the end of this directed
// edge. These are turn restrictions from one edge to another that apply to
// all vehicles, at all times.
uint32_t DirectedEdge::restrictions() const {
  return attributes_.restrictions;
}

// Get the use type of this edge.
Use DirectedEdge::use() const {
  return static_cast<Use>(attributes_.use);
}

// Is this edge a transit line (bus or rail)?
bool DirectedEdge::IsTransitLine() const {
  return (use() == Use::kRail || use() == Use::kBus);
}

// Get the speed type (see graphconstants.h)
SpeedType DirectedEdge::speed_type() const {
  return static_cast<SpeedType>(attributes_.speed_type);
}

// Get the country crossing flag.
bool DirectedEdge::ctry_crossing() const {
  return attributes_.ctry_crossing;
}

// Get the access modes in the forward direction (bit field).
uint8_t DirectedEdge::forwardaccess() const {
  return access_.forwardaccess;
}

// Get the access modes in the reverse direction (bit field).
uint8_t DirectedEdge::reverseaccess() const {
  return access_.reverseaccess;
}

// -------------------------------- speed -------------------------- //

// Gets the speed in KPH.
uint8_t DirectedEdge::speed() const {
  return speed_;
}

// Gets the truck speed in KPH.
uint32_t DirectedEdge::truck_speed() const {
  return access_.truck_speed;
}

// ----------------------------- Classification ---------------------------- //
// Get the road classification.
RoadClass DirectedEdge::classification() const {
  return static_cast<RoadClass>(classification_.classification);
}

// Is the edge considered unpaved?
bool DirectedEdge::unpaved() const {
  return (surface() >= Surface::kCompacted);
}

// Get the smoothness of this edge.
Surface DirectedEdge::surface() const {
  return static_cast<Surface>(classification_.surface);
}

// Is this edge a link / ramp?
bool DirectedEdge::link() const {
  return classification_.link;
}

// Gets the intersection internal flag.
bool DirectedEdge::internal() const {
  return classification_.internal;
}

// Gets the turn type given the prior edge's local index
Turn::Type DirectedEdge::turntype(const uint32_t localidx) const {
  // Turn type is 3 bits per index
  uint32_t shift = localidx * 3;
  return static_cast<Turn::Type>(
      ((turntypes_.turntype & (7 << shift))) >> shift);
}

// Is there an edge to the left, in between the from edge and this edge.
bool DirectedEdge::edge_to_left(const uint32_t localidx) const {
  return (turntypes_.edge_to_left & (1 << localidx));
}

// Get the stop impact when transitioning from the prior edge (given
// by the local index of the corresponding inbound edge at the node).
uint32_t DirectedEdge::stopimpact(const uint32_t localidx) const {
  // Stop impact is 3 bits per index
  uint32_t shift = localidx * 3;
  return (stopimpact_.s.stopimpact & (7 << shift)) >> shift;
}

// Get the transit line Id (for departure lookups along an edge)
uint32_t DirectedEdge::lineid() const {
  return stopimpact_.lineid;
}

// Is there an edge to the right, in between the from edge and this edge.
bool DirectedEdge::edge_to_right(const uint32_t localidx) const {
  return (stopimpact_.s.edge_to_right & (1 << localidx));
}

// Get the index of the directed edge on the local level of the graph
// hierarchy. This is used for turn restrictions so the edges can be
// identified on the different levels.
uint32_t DirectedEdge::localedgeidx() const {
  return hierarchy_.localedgeidx;
}

// Get the index of the opposing directed edge on the local hierarchy level
// at the end node of this directed edge. Only stored for the first 8 edges
// so it can be used for edge transition costing.
uint32_t DirectedEdge::opp_local_idx() const {
  return hierarchy_.opp_local_idx;
}

// Get the shortcut mask indicating the edge index that is superseded by
// this shortcut (0 if not a shortcut)
uint32_t DirectedEdge::shortcut() const {
 return hierarchy_.shortcut;
}

// Get the mask of the shortcut edge that supersedes this edge. 0 indicates
// no shortcut edge supersedes this edge.
uint32_t DirectedEdge::superseded() const {
  return hierarchy_.superseded;
}

// Does this edge represent a transition up one level in the hierarchy.
bool DirectedEdge::trans_up() const {
  return hierarchy_.trans_up;
}

// Does this edge represent a transition down one level in the hierarchy.
bool DirectedEdge::trans_down() const {
  return hierarchy_.trans_down;
}

// Is this directed edge a shortcut?
bool DirectedEdge::is_shortcut() const {
  return hierarchy_.is_shortcut;
}

// Json representation
json::MapPtr DirectedEdge::json() const {
  return json::map({
    {"end_node", endnode_.json()},
    {"speed", static_cast<uint64_t>(speed_)},
    //{"opp_index", static_cast<bool>(attributes_.opp_index)},
    //{"edge_info_offset", static_cast<uint64_t>(dataoffsets_.edgeinfo_offset)},
    //{"restrictions", attributes_.restrictions},
    {"access_restriction", static_cast<bool>(dataoffsets_.access_restriction)},
    {"start_complex_restriction", static_cast<bool>(dataoffsets_.start_complex_restriction)},
    {"end_complex_restriction", static_cast<bool>(dataoffsets_.end_complex_restriction)},
    {"has_exit_sign", static_cast<bool>(dataoffsets_.exitsign)},
    {"drive_on_right", static_cast<bool>(attributes_.drive_on_right)},
    {"ferry", static_cast<bool>(attributes_.ferry)},
    {"rail_ferry", static_cast<bool>(attributes_.railferry)},
    {"toll", static_cast<bool>(attributes_.toll)},
    {"seasonal", static_cast<bool>(attributes_.seasonal)},
    {"destination_only", static_cast<bool>(attributes_.dest_only)},
    {"tunnel", static_cast<bool>(attributes_.tunnel)},
    {"bridge", static_cast<bool>(attributes_.bridge)},
    {"round_about", static_cast<bool>(attributes_.roundabout)},
    {"unreachable", static_cast<bool>(attributes_.unreachable)},
    {"traffic_signal", static_cast<bool>(attributes_.traffic_signal)},
    {"forward", static_cast<bool>(attributes_.forward)},
    {"not_thru", static_cast<bool>(attributes_.not_thru)},
    {"cycle_lane", to_string(static_cast<CycleLane>(attributes_.cycle_lane))},
    {"bike_network", bike_network_json(attributes_.bike_network)},
    {"truck_route", bike_network_json(attributes_.truck_route)},
    {"lane_count", static_cast<uint64_t>(attributes_.lanecount)},
    {"use", to_string(static_cast<Use>(attributes_.use))},
    {"speed_type", to_string(static_cast<SpeedType>(attributes_.speed_type))},
    {"country_crossing", static_cast<bool>(attributes_.ctry_crossing)},
    {"geo_attributes", json::map({
      {"length", static_cast<uint64_t>(geoattributes_.length)},
      {"weighted_grade", json::fp_t{static_cast<double>(geoattributes_.weighted_grade - 6.5) / .6, 2}},
      //{"curvature", static_cast<uint64_t>(geoattributes_.curvature)},
    })},
    {"access", access_json(access_.forwardaccess)},
    //{"access", access_json(reverseaccess_)},
    {"classification", json::map({
      {"classification", to_string(static_cast<RoadClass>(classification_.classification))},
      {"surface", to_string(static_cast<Surface>(classification_.surface))},
      {"link", static_cast<bool>(classification_.link)},
      {"internal", static_cast<bool>(classification_.internal)},
    })},
    //{"hierarchy", json::map({
    //  {"", hierarchy_.localedgeidx},
    //  {"", hierarchy_.opp_local_idx},
    //  {"", hierarchy_.shortcut},
    //  {"", hierarchy_.superseded},
    //  {"", hierarchy_.trans_up},
    //  {"", hierarchy_.trans_down},
    //  {"", hierarchy_.is_shortcut},
    //})},
  });
}

// Get the internal version. Used for data validity checks.
const uint64_t DirectedEdge::internal_version() {
  size_t seed = 0;

  DirectedEdge de;
  de.dataoffsets_ = {};
  de.geoattributes_ = {};
  de.access_ = {};
  de.speed_ = 0;
  de.attributes_ = {};
  de.classification_ = {};

  // DataOffsets
  de.dataoffsets_.edgeinfo_offset = ~de.dataoffsets_.edgeinfo_offset;
  boost::hash_combine(seed, ffs(de.dataoffsets_.edgeinfo_offset+1)-1);
  de.dataoffsets_.access_restriction = ~de.dataoffsets_.access_restriction;
  boost::hash_combine(seed, ffs(de.dataoffsets_.access_restriction+1)-1);
  de.dataoffsets_.start_complex_restriction = ~de.dataoffsets_.start_complex_restriction;
  boost::hash_combine(seed, ffs(de.dataoffsets_.start_complex_restriction+1)-1);
  de.dataoffsets_.end_complex_restriction = ~de.dataoffsets_.end_complex_restriction;
  boost::hash_combine(seed, ffs(de.dataoffsets_.end_complex_restriction+1)-1);
  de.dataoffsets_.exitsign = ~de.dataoffsets_.exitsign;
  boost::hash_combine(seed, ffs(de.dataoffsets_.exitsign+1)-1);
  de.dataoffsets_.spare = ~de.dataoffsets_.spare;
  boost::hash_combine(seed, ffs(de.dataoffsets_.spare+1)-1);

  // GeoAttributes
  de.geoattributes_.length = ~de.geoattributes_.length;
  boost::hash_combine(seed,ffs(de.geoattributes_.length+1)-1);
  de.geoattributes_.weighted_grade = ~de.geoattributes_.weighted_grade;
  boost::hash_combine(seed,ffs(de.geoattributes_.weighted_grade+1)-1);
  de.geoattributes_.curvature = ~de.geoattributes_.curvature;
  boost::hash_combine(seed,ffs(de.geoattributes_.curvature+1)-1);

  // Attributes
  de.attributes_.drive_on_right = ~de.attributes_.drive_on_right;
  boost::hash_combine(seed,ffs(de.attributes_.drive_on_right+1)-1);
  de.attributes_.ferry = ~de.attributes_.ferry;
  boost::hash_combine(seed,ffs(de.attributes_.ferry+1)-1);
  de.attributes_.railferry = ~de.attributes_.railferry;
  boost::hash_combine(seed,ffs(de.attributes_.railferry+1)-1);
  de.attributes_.toll = ~de.attributes_.toll;
  boost::hash_combine(seed,ffs(de.attributes_.toll+1)-1);
  de.attributes_.seasonal = ~de.attributes_.seasonal;
  boost::hash_combine(seed,ffs(de.attributes_.seasonal+1)-1);
  de.attributes_.dest_only = ~de.attributes_.dest_only;
  boost::hash_combine(seed,ffs(de.attributes_.dest_only+1)-1);
  de.attributes_.tunnel = ~de.attributes_.tunnel;
  boost::hash_combine(seed,ffs(de.attributes_.tunnel+1)-1);
  de.attributes_.bridge = ~de.attributes_.bridge;
  boost::hash_combine(seed,ffs(de.attributes_.bridge+1)-1);
  de.attributes_.roundabout = ~de.attributes_.roundabout;
  boost::hash_combine(seed,ffs(de.attributes_.roundabout+1)-1);
  de.attributes_.unreachable = ~de.attributes_.unreachable;
  boost::hash_combine(seed,ffs(de.attributes_.unreachable+1)-1);
  de.attributes_.traffic_signal = ~de.attributes_.traffic_signal;
  boost::hash_combine(seed,ffs(de.attributes_.traffic_signal+1)-1);
  de.attributes_.forward = ~de.attributes_.forward;
  boost::hash_combine(seed,ffs(de.attributes_.forward+1)-1);
  de.attributes_.not_thru = ~de.attributes_.not_thru;
  boost::hash_combine(seed,ffs(de.attributes_.not_thru+1)-1);
  de.attributes_.opp_index = ~de.attributes_.opp_index;
  boost::hash_combine(seed,ffs(de.attributes_.opp_index+1)-1);
  de.attributes_.cycle_lane = ~de.attributes_.cycle_lane;
  boost::hash_combine(seed,ffs(de.attributes_.cycle_lane+1)-1);
  de.attributes_.bike_network = ~de.attributes_.bike_network;
  boost::hash_combine(seed,ffs(de.attributes_.bike_network+1)-1);
  de.attributes_.truck_route = ~de.attributes_.truck_route;
  boost::hash_combine(seed,ffs(de.attributes_.truck_route+1)-1);
  de.attributes_.lanecount = ~de.attributes_.lanecount;
  boost::hash_combine(seed,ffs(de.attributes_.lanecount+1)-1);
  de.attributes_.restrictions = ~de.attributes_.restrictions;
  boost::hash_combine(seed,ffs(de.attributes_.restrictions+1)-1);
  de.attributes_.use = ~de.attributes_.use;
  boost::hash_combine(seed,ffs(de.attributes_.use+1)-1);
  de.attributes_.speed_type = ~de.attributes_.speed_type;
  boost::hash_combine(seed,ffs(de.attributes_.speed_type+1)-1);
  de.attributes_.ctry_crossing = ~de.attributes_.ctry_crossing;
  boost::hash_combine(seed,ffs(de.attributes_.ctry_crossing+1)-1);
  de.attributes_.spare = ~de.attributes_.spare;
  boost::hash_combine(seed,ffs(de.attributes_.spare+1)-1);

  // Access
  de.access_.forwardaccess  = ~de.access_.forwardaccess;
  boost::hash_combine(seed,ffs(de.access_.forwardaccess+1)-1);
  de.access_.reverseaccess  = ~de.access_.reverseaccess;
  boost::hash_combine(seed,ffs(de.access_.reverseaccess+1)-1);

  de.access_.truck_speed  = ~de.access_.truck_speed;
  boost::hash_combine(seed,ffs(de.access_.truck_speed+1)-1);

  // Speed
  boost::hash_combine(seed,de.speed_);

  // Classification
  de.classification_.classification = ~de.classification_.classification;
  boost::hash_combine(seed,ffs(de.classification_.classification+1)-1);
  de.classification_.surface = ~de.classification_.surface;
  boost::hash_combine(seed,ffs(de.classification_.surface+1)-1);
  de.classification_.link = ~de.classification_.link;
  boost::hash_combine(seed,ffs(de.classification_.link+1)-1);
  de.classification_.internal = ~de.classification_.internal;
  boost::hash_combine(seed,ffs(de.classification_.internal+1)-1);

  // Turn types
  de.turntypes_.turntype = ~de.turntypes_.turntype;
  boost::hash_combine(seed,ffs(de.turntypes_.turntype+1)-1);
  de.turntypes_.edge_to_left = ~de.turntypes_.edge_to_left;
  boost::hash_combine(seed,ffs(de.turntypes_.edge_to_left+1)-1);

  // Stop impact
  de.stopimpact_.s.stopimpact = ~de.stopimpact_.s.stopimpact;
  boost::hash_combine(seed,ffs(de.stopimpact_.s.stopimpact+1)-1);
  de.stopimpact_.s.edge_to_right = ~de.stopimpact_.s.edge_to_right;
  boost::hash_combine(seed,ffs(de.stopimpact_.s.edge_to_right+1)-1);

  // Hierarchy and shortcut information
  de.hierarchy_.localedgeidx = ~de.hierarchy_.localedgeidx;
  boost::hash_combine(seed,ffs(de.hierarchy_.localedgeidx+1)-1);
  de.hierarchy_.opp_local_idx = ~de.hierarchy_.opp_local_idx;
  boost::hash_combine(seed,ffs(de.hierarchy_.opp_local_idx+1)-1);
  de.hierarchy_.trans_up = ~de.hierarchy_.trans_up;
  boost::hash_combine(seed,ffs(de.hierarchy_.trans_up+1)-1);
  de.hierarchy_.trans_down = ~de.hierarchy_.trans_down;
  boost::hash_combine(seed,ffs(de.hierarchy_.trans_down+1)-1);
  de.hierarchy_.shortcut = ~de.hierarchy_.shortcut;
  boost::hash_combine(seed,ffs(de.hierarchy_.shortcut+1)-1);
  de.hierarchy_.superseded = ~de.hierarchy_.superseded;
  boost::hash_combine(seed,ffs(de.hierarchy_.superseded+1)-1);
  de.hierarchy_.is_shortcut = ~de.hierarchy_.is_shortcut;
  boost::hash_combine(seed,ffs(de.hierarchy_.is_shortcut+1)-1);
  de.hierarchy_.spare = ~de.hierarchy_.spare;
  boost::hash_combine(seed,ffs(de.hierarchy_.spare+1)-1);

  boost::hash_combine(seed,sizeof(DirectedEdge));

  return seed;
}

}
}
