#include "baldr/directededge.h"
#include "baldr/nodeinfo.h"
#include <boost/functional/hash.hpp>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace {

json::MapPtr bike_network_json(uint8_t mask) {
  return json::map({
    {"national", static_cast<bool>(mask & kNcn)},
    {"regional", static_cast<bool>(mask & kRcn)},
    {"local", static_cast<bool>(mask & kLcn)},
    {"mountain", static_cast<bool>(mask & kMcn)},
  });
}

json::MapPtr access_json(uint32_t access) {
  return json::map({
    {"bicycle", static_cast<bool>(access & kBicycleAccess)},
    {"bus", static_cast<bool>(access & kBusAccess)},
    {"car", static_cast<bool>(access & kAutoAccess)},
    {"emergency", static_cast<bool>(access & kEmergencyAccess)},
    {"HOV", static_cast<bool>(access & kHOVAccess)},
    {"pedestrian", static_cast<bool>(access & kPedestrianAccess)},
    {"taxi", static_cast<bool>(access & kTaxiAccess)},
    {"truck", static_cast<bool>(access & kTruckAccess)},
    {"wheelchair", static_cast<bool>(access & kWheelchairAccess)}
  });
}

/**
 * Get the updated bit field.
 * @param dst  Data member to be updated.
 * @param src  Value to be updated.
 * @param pos  Position (pos element within the bit field).
 * @param len  Length of each element within the bit field.
 * @return  Returns an updated value for the bit field.
 */
uint32_t OverwriteBits(const uint32_t dst, const uint32_t src,
                       const uint32_t pos, const uint32_t len) {
  uint32_t shift = (pos * len);
  uint32_t mask  = (((uint32_t)1 << len) - 1) << shift;
  return (dst & ~mask) | (src << shift);
}

}

namespace valhalla {
namespace baldr {

// Default constructor
DirectedEdge::DirectedEdge() {
  memset(this, 0, sizeof(DirectedEdge));
  weighted_grade_ = 6;
}

// Gets the end node of this directed edge.
GraphId DirectedEdge::endnode() const {
  return endnode_;
}

// Sets the end node of this directed edge.
void DirectedEdge::set_endnode(const GraphId& endnode) {
  endnode_ = endnode;
}

// ------------------  Data offsets and flags for extended data -------------//

// Get the offset to the common edge information.
uint64_t DirectedEdge::edgeinfo_offset() const {
  return edgeinfo_offset_;
}

// Get the offset to the common edge data.
void DirectedEdge::set_edgeinfo_offset(const uint32_t offset) {
  if (offset > kMaxEdgeInfoOffset) {
    // Consider this a catastrophic error
    LOG_ERROR("Exceeded maximum edgeinfo offset: " + std::to_string(offset));
    throw std::runtime_error("DirectedEdge: exceeded maximum edgeinfo offset");
  } else {
    edgeinfo_offset_ = offset;
  }
}

// Get the general restriction or access condition (per mode) for this directed edge.
uint64_t DirectedEdge::access_restriction() const {
  return access_restriction_;
}

// Set the modes which have access restrictions on this edge.
void DirectedEdge::set_access_restriction(const uint32_t access) {
  access_restriction_ = access;
}

// Does this directed edge have exit signs.
bool DirectedEdge::exitsign() const {
  return exitsign_;
}

// Sets the exit flag.
void DirectedEdge::set_exitsign(const bool exit) {
  exitsign_ = exit;
}

// ------------------------- Geographic attributes ------------------------- //

// Gets the length of the edge in meters.
uint32_t DirectedEdge::length() const {
  return length_;
}

// Sets the length of the edge in meters.
void DirectedEdge::set_length(const uint32_t length) {
  if (length > kMaxEdgeLength) {
    LOG_WARN("Exceeding max. edge length: " + std::to_string(length));
    length_ = kMaxEdgeLength;
  } else {
    length_ = length;
  }
}

// Gets the weighted grade factor (0-15).
uint32_t DirectedEdge::weighted_grade() const {
  return weighted_grade_;
}

// Sets the weighted_grade factor (0-15) for the edge.
void DirectedEdge::set_weighted_grade(const uint32_t factor) {
  if (factor > kMaxGradeFactor) {
    LOG_WARN("Exceeding max. weighted grade factor: " + std::to_string(factor));
    weighted_grade_ = 6;
  } else {
    weighted_grade_ = factor;
  }
}

// Get the road curvature factor. (0-15).
uint32_t DirectedEdge::curvature() const {
  return curvature_;
}

// Sets the curvature factor (0-15) for the edge.
void DirectedEdge::set_curvature(const uint32_t factor) {
  if (factor > kMaxCurvatureFactor) {
    LOG_WARN("Exceeding max. curvature factor: " + std::to_string(factor));
    curvature_ = 0;
  } else {
    curvature_ = factor;
  }
}

// -------------------------- Routing attributes --------------------------- //

// Is driving on the right hand side of the road along this edge?
bool DirectedEdge::drive_on_right() const {
  return drive_on_right_;
}

// Set the flag indicating driving is on the right hand side of the road
// along this edge?
void DirectedEdge::set_drive_on_right(const bool rsd) {
  drive_on_right_ = rsd;
}

// Flag indicating the edge is a dead end (no other driveable
// roads at the end node of this edge).
bool DirectedEdge::deadend() const {
  return deadend_;
}

// Set the flag indicating the edge is a dead end (no other driveable
// roads at the end node of this edge).
void DirectedEdge::set_deadend(const bool d) {
  deadend_ = d;
}

// Does this edge have a toll or is it part of a toll road?
bool DirectedEdge::toll() const {
  return toll_;
}

// Sets the flag indicating this edge has a toll or is it part of a toll road.
void DirectedEdge::set_toll(const bool toll) {
  toll_ = toll;
}

// Does this edge have a seasonal access (e.g., closed in the winter)?
bool DirectedEdge::seasonal() const {
  return seasonal_;
}

// Sets the flag indicating this edge has seasonal access
void DirectedEdge::set_seasonal(const bool seasonal) {
  seasonal_ = seasonal;
}

// Is this edge part of a private or no through road that allows access
// only if required to get to a destination?
bool DirectedEdge::destonly() const {
  return dest_only_;
}

// Sets the destination only (private) flag. This indicates the edge should
// allow access only to locations that are destinations and not allow
// "through" traffic
void DirectedEdge::set_dest_only(const bool destonly) {
  dest_only_ = destonly;
}

// Is this edge part of a tunnel?
bool DirectedEdge::tunnel() const {
  return tunnel_;
}

// Sets the flag indicating this edge has is a tunnel of part of a tunnel.
void DirectedEdge::set_tunnel(const bool tunnel) {
  tunnel_ = tunnel;
}

// Is this edge part of a bridge?
bool DirectedEdge::bridge() const {
  return bridge_;
}

// Sets the flag indicating this edge has is a bridge of part of a bridge.
void DirectedEdge::set_bridge(const bool bridge) {
  bridge_ = bridge;
}

// Is this edge part of a roundabout?
bool DirectedEdge::roundabout() const {
  return roundabout_;
}

// Sets the flag indicating the  edge is part of a roundabout.
void DirectedEdge::set_roundabout(const bool roundabout) {
  roundabout_ = roundabout;
}

// Is this edge is unreachable by driving. This can happen if a driveable
// edge is surrounded by pedestrian only edges (e.g. in a city center) or
// is not properly connected to other edges.
bool DirectedEdge::unreachable() const {
  return unreachable_;
}

// Sets the flag indicating the edge is unreachable by driving. This can
// happen if a driveable edge is surrounded by pedestrian only edges (e.g.
// in a city center) or is not properly connected to other edges.
void DirectedEdge::set_unreachable(const bool unreachable) {
  unreachable_ = unreachable;
}

// A traffic signal occurs at the end of this edge.
bool DirectedEdge::traffic_signal() const {
  return traffic_signal_;
}

// Sets the flag indicating a traffic signal is present at the end of
// this edge.
void DirectedEdge::set_traffic_signal(const bool signal) {
  traffic_signal_ = signal;
}

// Is this directed edge stored forward in edgeinfo (true) or
// reverse (false).
bool DirectedEdge::forward() const {
  return forward_;
}

// Set the forward flag. Tells if this directed edge is stored forward
// in edgeinfo (true) or reverse (false).
void DirectedEdge::set_forward(const bool forward) {
  forward_ = forward;
}

// Does this edge lead into a no-thru region
bool DirectedEdge::not_thru() const {
  return not_thru_;
}

// Sets the not thru flag.
void DirectedEdge::set_not_thru(const bool not_thru) {
  not_thru_ = not_thru;
}

// Get the index of the opposing directed edge at the end node of this
// directed edge.
uint32_t DirectedEdge::opp_index() const {
  return opp_index_;
}

// Set the index of the opposing directed edge at the end node of this
// directed edge.
void DirectedEdge::set_opp_index(const uint32_t opp_index) {
  opp_index_ = opp_index;
}

// Get the cycle lane type along this edge.
CycleLane DirectedEdge::cyclelane() const {
  return static_cast<CycleLane>(cycle_lane_);
}

// Sets the type of cycle lane (if any) present on this edge.
void DirectedEdge::set_cyclelane(const CycleLane cyclelane) {
  cycle_lane_ = static_cast<uint32_t>(cyclelane);
}

// Gets the bike network mask
uint32_t DirectedEdge::bike_network() const {
  return bike_network_;
}

// Sets the bike network mask indicating which (if any) bicycle networks are
// along this edge. See baldr/directededge.h for definitions.
void DirectedEdge::set_bike_network(const uint32_t bike_network) {
  if (bike_network > kMaxBicycleNetwork) {
    LOG_WARN("Bicycle Network mask exceeds maximum: " +
              std::to_string(bike_network));
    bike_network_ = 0;
  } else {
    bike_network_ = bike_network;
  }
}

// Gets the truck network flag
bool DirectedEdge::truck_route() const {
  return truck_route_;
}

// Sets truck route flag.
void DirectedEdge::set_truck_route(const bool truck_route) {
  truck_route_ = truck_route;
}

// Gets the lane count
uint32_t DirectedEdge::lanecount() const {
  return lanecount_;
}

// Sets the number of lanes
void DirectedEdge::set_lanecount(const uint32_t lanecount) {
  // TOTO - make sure we don't exceed some max value
  if (lanecount > kMaxLaneCount) {
    LOG_WARN("Exceeding maximum lane count: " + std::to_string(lanecount));
    lanecount_ = kMaxLaneCount;
  } else {
    lanecount_ = lanecount;
  }
}

// Get the mask of simple turn restrictions from the end of this directed
// edge. These are turn restrictions from one edge to another that apply to
// all vehicles, at all times.
uint32_t DirectedEdge::restrictions() const {
  return restrictions_;
}

// Set simple turn restrictions from the end of this directed edge.
// These are turn restrictions from one edge to another that apply to
// all vehicles, at all times.
void DirectedEdge::set_restrictions(const uint32_t mask) {
  if (mask >= (1 << kMaxTurnRestrictionEdges)) {
    LOG_WARN("Restrictions mask exceeds allowable limit: " +
                std::to_string(mask));
    restrictions_ = (mask & ((1 << kMaxTurnRestrictionEdges) - 1));
  } else {
    restrictions_ = mask;
  }
}

// Get the use type of this edge.
Use DirectedEdge::use() const {
  return static_cast<Use>(use_);
}

// Is this edge a transit line (bus or rail)?
bool DirectedEdge::IsTransitLine() const {
  return (use() == Use::kRail || use() == Use::kBus);
}

// Sets the specialized use type of this edge.
void DirectedEdge::set_use(const Use use) {
  use_ = static_cast<uint32_t>(use);
}

// Get the speed type (see graphconstants.h)
SpeedType DirectedEdge::speed_type() const {
  return static_cast<SpeedType>(speed_type_);
}

// Set the speed type (see graphconstants.h)
void DirectedEdge::set_speed_type(const SpeedType speed_type) {
  speed_type_ = static_cast<uint32_t>(speed_type);
}

// Get the country crossing flag.
bool DirectedEdge::ctry_crossing() const {
  return ctry_crossing_;
}

// Set the country crossing flag.
void DirectedEdge::set_ctry_crossing(const bool crossing) {
  ctry_crossing_ = crossing;
}

// Get the access modes in the forward direction (bit field).
uint32_t DirectedEdge::forwardaccess() const {
  return forwardaccess_;
}

// Set the access modes in the forward direction (bit field).
void DirectedEdge::set_forwardaccess(const uint32_t modes) {
  if (modes > kAllAccess) {
    LOG_ERROR("DirectedEdge: forward access exceeds maximum allowed: " +
              std::to_string(modes));
    forwardaccess_ = (modes & kAllAccess);
  } else {
    forwardaccess_ = modes;
  }
  forwardaccess_ = modes;
}

// Set all forward access modes to true (used for transition edges)
// Also sets reverse access so opposing edge matches.
void DirectedEdge::set_all_forward_access() {
  forwardaccess_ = kAllAccess;
  reverseaccess_ = kAllAccess;
}

// Get the access modes in the reverse direction (bit field).
uint32_t DirectedEdge::reverseaccess() const {
  return reverseaccess_;
}

// Set the access modes in the reverse direction (bit field).
void DirectedEdge::set_reverseaccess(const uint32_t modes) {
  if (modes > kAllAccess) {
    LOG_ERROR("DirectedEdge: reverse access exceeds maximum allowed: " +
              std::to_string(modes));
    reverseaccess_ = (modes & kAllAccess);
  } else {
    reverseaccess_ = modes;
  }
}

// -------------------------------- speed -------------------------- //

// Gets the speed in KPH.
uint32_t DirectedEdge::speed() const {
  return speed_;
}

// Sets the speed in KPH.
void DirectedEdge::set_speed(const uint32_t speed) {
  if (speed > kMaxSpeed) {
    LOG_WARN("Exceeding maximum speed: " + std::to_string(speed));
    speed_ = kMaxSpeed;
  } else {
    speed_ = speed;
  }
}

// Gets the truck speed in KPH.
uint32_t DirectedEdge::truck_speed() const {
  return truck_speed_;
}

// Sets the truck speed in KPH.
void DirectedEdge::set_truck_speed(const uint32_t speed) {
  if (speed > kMaxSpeed) {
    LOG_WARN("Exceeding maximum truck speed: " + std::to_string(speed));
    truck_speed_ = kMaxSpeed;
  } else {
    truck_speed_ = speed;
  }
}

// ----------------------------- Classification ---------------------------- //
// Get the road classification.
RoadClass DirectedEdge::classification() const {
  return static_cast<RoadClass>(classification_);
}

// Sets the classification (importance) of this edge.
void DirectedEdge::set_classification(const RoadClass roadclass) {
  classification_= static_cast<uint32_t>(roadclass);
}

// Is the edge considered unpaved?
bool DirectedEdge::unpaved() const {
  return (surface() >= Surface::kCompacted);
}

// Get the smoothness of this edge.
Surface DirectedEdge::surface() const {
  return static_cast<Surface>(surface_);
}

// Sets the surface type (see baldr/graphconstants.h). This is a general
// indication of smoothness.
void DirectedEdge::set_surface(const Surface surface) {
  surface_ = static_cast<uint32_t>(surface);
}

// Is this edge a link / ramp?
bool DirectedEdge::link() const {
  return link_;
}

// Sets the link flag indicating the edge is part of a link or connection
// (ramp or turn channel).
void DirectedEdge::set_link(const bool link) {
  link_ = link;
}

// Gets the intersection internal flag.
bool DirectedEdge::internal() const {
  return internal_;
}

// Sets the intersection internal flag.
void DirectedEdge::set_internal(const bool internal) {
  internal_ = internal;
}

// Get the Complex restriction (per mode) for this directed edge at the start.
uint32_t DirectedEdge::start_restriction() const {
  return start_restriction_;
}

// Set the Complex restriction (per mode) for this directed edge at the start.
void DirectedEdge::set_start_restriction(const uint32_t modes) {
  start_restriction_ = modes;
}

// Get the Complex restriction (per mode) for this directed edge at the end.
uint32_t DirectedEdge::end_restriction() const {
  return end_restriction_;
}

// Set the Complex restriction (per mode) for this directed edge at the end.
void DirectedEdge::set_end_restriction(const uint32_t modes) {
  end_restriction_ = modes;
}

// Is this part of complex restriction flag.
bool DirectedEdge::part_of_complex_restriction() const {
  return part_of_complex_restriction_;
}

// Set the part of complex restriction flag.
void DirectedEdge::set_part_of_complex_restriction(const bool part_of) {
  part_of_complex_restriction_ = part_of;
}

// Gets the maximum upward slope. Uses 1 degree precision for slopes to
// 16 degrees, and 4 degree precision afterwards (up to a max of 76 degrees).
int DirectedEdge::max_up_slope() const {
  return ((max_up_slope_ & 0x10) == 0) ? max_up_slope_ :
          16 + ((max_up_slope_ & 0xf) * 4);
}

// Sets the maximum upward slope.
void DirectedEdge::set_max_up_slope(const float slope) {
  if (slope < 0.0f) {
    max_up_slope_ = 0;
  } else if (slope < 16.0f) {
    max_up_slope_ = static_cast<int>(std::ceil(slope));
  } else if (slope < 76.0f) {
    max_up_slope_ = 0x10 | static_cast<int>(std::ceil((slope - 16.0f) * 0.25f));
  } else {
    max_up_slope_ = 0x1f;
  }
}

// Gets the maximum downward slope. Uses 1 degree precision for slopes to
// -8 degrees, and 4 degree precision afterwards (up to a max of -76 degs).
int DirectedEdge::max_down_slope() const {
  return ((max_down_slope_ & 0x10) == 0) ? -static_cast<int>(max_down_slope_) :
          -static_cast<int>(16 + ((max_down_slope_ & 0xf) * 4));
}

// Sets the maximum downward slope.
void DirectedEdge::set_max_down_slope(const float slope) {
  if (slope > 0.0f) {
    max_down_slope_ = 0;
  } else if (slope > -16.0f) {
    max_down_slope_ = static_cast<int>(std::ceil(-slope));
  } else if (slope > -76.0f) {
    max_down_slope_ = 0x10 | static_cast<int>(std::ceil((-slope - 16.0f) * 0.25f));
  } else {
    max_down_slope_ = 0x1f;
  }
}

// Get the density along the edges.
uint32_t DirectedEdge::density() const {
  return density_;
}

// Set the density along the edges.
void DirectedEdge::set_density(const uint32_t density) {
  if (density > kMaxDensity) {
    LOG_WARN("Exceeding max. density: " + std::to_string(density));
    density_ = kMaxDensity;
  } else {
    density_ = density;
  }
}

// Is there a sidewalk to the left of this directed edge?
bool DirectedEdge::sidewalk_left() const {
  return sidewalk_left_;
}

// Set the flag for a sidewalk to the left of this directed edge.
void DirectedEdge::set_sidewalk_left(const bool sidewalk) {
  sidewalk_left_ = sidewalk;
}

// Is there a sidewalk to the right of this directed edge?
bool DirectedEdge::sidewalk_right() const {
  return sidewalk_right_;
}

// Set the flag for a sidewalk to the right of this directed edge.
void DirectedEdge::set_sidewalk_right(const bool sidewalk) {
  sidewalk_right_ = sidewalk;
}

// Gets the turn type given the prior edge's local index
Turn::Type DirectedEdge::turntype(const uint32_t localidx) const {
  // Turn type is 3 bits per index
  uint32_t shift = localidx * 3;
  return static_cast<Turn::Type>(
      ((turntype_ & (7 << shift))) >> shift);
}

// Sets the turn type given the prior edge's local index
// (index of the inbound edge).
void DirectedEdge::set_turntype(const uint32_t localidx,
                                       const Turn::Type turntype) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Exceeding max local index in set_turntype. Skipping");
  } else {
    turntype_ = OverwriteBits(turntype_,
                   static_cast<uint32_t>(turntype), localidx, 3);
  }
}

// Is there an edge to the left, in between the from edge and this edge.
bool DirectedEdge::edge_to_left(const uint32_t localidx) const {
  return (edge_to_left_ & (1 << localidx));
}

// Set the flag indicating there is an edge to the left, in between
// the from edge and this edge.
void DirectedEdge::set_edge_to_left(const uint32_t localidx,
                                           const bool left) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Exceeding max local index in set_edge_to_left. Skipping");
  } else {
    edge_to_left_ = OverwriteBits(edge_to_left_, left, localidx, 1);
  }
}

// Get the stop impact when transitioning from the prior edge (given
// by the local index of the corresponding inbound edge at the node).
uint32_t DirectedEdge::stopimpact(const uint32_t localidx) const {
  // Stop impact is 3 bits per index
  uint32_t shift = localidx * 3;
  return (stopimpact_.s.stopimpact & (7 << shift)) >> shift;
}

// Set the stop impact when transitioning from the prior edge (given
// by the local index of the corresponding inbound edge at the node).
void DirectedEdge::set_stopimpact(const uint32_t localidx,
                                         const uint32_t stopimpact) {
  if (stopimpact > kMaxStopImpact) {
    LOG_WARN("Exceeding maximum stop impact: " + std::to_string(stopimpact));
    stopimpact_.s.stopimpact = OverwriteBits(stopimpact_.s.stopimpact,
                                           kMaxStopImpact, localidx, 3);
  } else {
    stopimpact_.s.stopimpact = OverwriteBits(stopimpact_.s.stopimpact, stopimpact,
                                           localidx, 3);
  }
}

// Get the transit line Id (for departure lookups along an edge)
uint32_t DirectedEdge::lineid() const {
  return stopimpact_.lineid;
}

// Set the unique transit line Id.
void DirectedEdge::set_lineid(const uint32_t lineid) {
  stopimpact_.lineid = lineid;
}

// Is there an edge to the right, in between the from edge and this edge.
bool DirectedEdge::edge_to_right(const uint32_t localidx) const {
  return (stopimpact_.s.edge_to_right & (1 << localidx));
}

// Set the flag indicating there is an edge to the right, in between
// the from edge and this edge.
void DirectedEdge::set_edge_to_right(const uint32_t localidx,
                                            const bool right) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Exceeding max local index in set_edge_to_right. Skipping");
  } else {
    stopimpact_.s.edge_to_right = OverwriteBits(stopimpact_.s.edge_to_right,
                                right, localidx, 1);
  }
}

// Get the index of the directed edge on the local level of the graph
// hierarchy. This is used for turn restrictions so the edges can be
// identified on the different levels.
uint32_t DirectedEdge::localedgeidx() const {
  return localedgeidx_;
}

// Set the index of the directed edge on the local level of the graph
// hierarchy. This is used for turn restrictions so the edges can be
// identified on the different levels.
void DirectedEdge::set_localedgeidx(const uint32_t idx) {
  if (idx > kMaxEdgesPerNode) {
    LOG_WARN("Local Edge Index exceeds max: " + std::to_string(idx));
    localedgeidx_ = kMaxEdgesPerNode;
  } else {
    localedgeidx_ = idx;
  }
}

// Get the index of the opposing directed edge on the local hierarchy level
// at the end node of this directed edge. Only stored for the first 8 edges
// so it can be used for edge transition costing.
uint32_t DirectedEdge::opp_local_idx() const {
  return opp_local_idx_;
}

// Set the index of the opposing directed edge on the local hierarchy level
// at the end node of this directed edge. Only stored for the first 8 edges
// so it can be used for edge transition costing.
void DirectedEdge::set_opp_local_idx(const uint32_t idx) {
  if (idx > kMaxEdgesPerNode) {
    LOG_WARN("Exceeding max edges in opposing local index: " + std::to_string(idx));
    opp_local_idx_ = kMaxEdgesPerNode;
  } else {
    opp_local_idx_ = idx;
  }
}

// Get the shortcut mask indicating the edge index that is superseded by
// this shortcut (0 if not a shortcut)
uint32_t DirectedEdge::shortcut() const {
 return shortcut_;
}

// Set the flag for whether this edge represents a shortcut between 2 nodes.
void DirectedEdge::set_shortcut(const uint32_t shortcut) {
  // 0 is not a valid shortcut
  if (shortcut == 0) {
    LOG_WARN("Invalid shortcut mask = 0");
    return;
  }

  // Set the shortcut mask if within the max number of masked shortcut edges
  if (shortcut <= kMaxShortcutsFromNode) {
    shortcut_ = (1 << (shortcut-1));
  }

  // Set the is_shortcut flag
  is_shortcut_ = true;
}

// Get the mask of the shortcut edge that supersedes this edge. 0 indicates
// no shortcut edge supersedes this edge.
uint32_t DirectedEdge::superseded() const {
  return superseded_;
}

// Set the flag for whether this edge is superseded by a shortcut edge.
void DirectedEdge::set_superseded(const uint32_t superseded) {
  if (superseded > kMaxShortcutsFromNode) {
      LOG_WARN("Exceeding max shortcut edges from a node: " + std::to_string(superseded));
  } else {
    superseded_ = (1 << (superseded-1));
  }
}

// Does this edge represent a transition up one level in the hierarchy.
bool DirectedEdge::trans_up() const {
  return (use() == Use::kTransitionUp);
}


// Set the use indicating this edge represents a transition up one level
// in the hierarchy.
void DirectedEdge::set_trans_up() {
  set_use(Use::kTransitionUp);
}

// Does this edge represent a transition down one level in the hierarchy.
bool DirectedEdge::trans_down() const {
  return (use() == Use::kTransitionDown);
}


// Set the use indicating this edge represents a transition down one level
// in the hierarchy.
void DirectedEdge::set_trans_down() {
  set_use(Use::kTransitionDown);
}

// Is this directed edge a shortcut?
bool DirectedEdge::is_shortcut() const {
  return is_shortcut_;
}

// Does this directed edge end in a different tile.
bool DirectedEdge::leaves_tile() const {
  return leaves_tile_;
}

// Set the flag indicating whether the end node of this directed edge is in
// a different tile
void DirectedEdge::set_leaves_tile(const bool leaves_tile) {
  leaves_tile_ = leaves_tile;
}

// Json representation
json::MapPtr DirectedEdge::json() const {
  return json::map({
    {"end_node", endnode_.json()},
    {"speed", static_cast<uint64_t>(speed_)},
    //{"opp_index", static_cast<bool>(opp_index_)},
    //{"edge_info_offset", static_cast<uint64_t>(edgeinfo_offset_)},
    //{"restrictions", restrictions_},
    {"access_restriction", static_cast<bool>(access_restriction_)},
    {"start_restriction", access_json(start_restriction_)},
    {"end_restriction", access_json(end_restriction_)},
    {"part_of_complex_restriction", static_cast<bool>(part_of_complex_restriction_)},
    {"has_exit_sign", static_cast<bool>(exitsign_)},
    {"drive_on_right", static_cast<bool>(drive_on_right_)},
    {"toll", static_cast<bool>(toll_)},
    {"seasonal", static_cast<bool>(seasonal_)},
    {"destination_only", static_cast<bool>(dest_only_)},
    {"tunnel", static_cast<bool>(tunnel_)},
    {"bridge", static_cast<bool>(bridge_)},
    {"round_about", static_cast<bool>(roundabout_)},
    {"unreachable", static_cast<bool>(unreachable_)},
    {"traffic_signal", static_cast<bool>(traffic_signal_)},
    {"forward", static_cast<bool>(forward_)},
    {"not_thru", static_cast<bool>(not_thru_)},
    {"cycle_lane", to_string(static_cast<CycleLane>(cycle_lane_))},
    {"bike_network", bike_network_json(bike_network_)},
    {"truck_route", static_cast<bool>(truck_route_)},
    {"lane_count", static_cast<uint64_t>(lanecount_)},
    {"use", to_string(static_cast<Use>(use_))},
    {"speed_type", to_string(static_cast<SpeedType>(speed_type_))},
    {"country_crossing", static_cast<bool>(ctry_crossing_)},
    {"geo_attributes", json::map({
      {"length", static_cast<uint64_t>(length_)},
      {"weighted_grade", json::fp_t{static_cast<double>(weighted_grade_ - 6.0) / .6, 2}},
      //{"curvature", static_cast<uint64_t>(curvature_)},
    })},
    {"access", access_json(forwardaccess_)},
    //{"access", access_json(reverseaccess_)},
    {"classification", json::map({
      {"classification", to_string(static_cast<RoadClass>(classification_))},
      {"surface", to_string(static_cast<Surface>(surface_))},
      {"link", static_cast<bool>(link_)},
      {"internal", static_cast<bool>(internal_)},
    })},
    //{"hierarchy", json::map({
    //  {"", localedgeidx_},
    //  {"", opp_local_idx_},
    //  {"", shortcut_},
    //  {"", superseded_},
    //  {"", trans_up_},
    //  {"", trans_down_},
    //  {"", is_shortcut_},
    //})},
  });
}


}
}
