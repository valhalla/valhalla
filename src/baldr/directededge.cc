#include "baldr/directededge.h"
#include "baldr/nodeinfo.h"
#include "midgard/logging.h"

using namespace valhalla::baldr;

namespace {

json::MapPtr access_json(uint32_t access) {
  return json::map({{"bicycle", static_cast<bool>(access & kBicycleAccess)},
                    {"bus", static_cast<bool>(access & kBusAccess)},
                    {"car", static_cast<bool>(access & kAutoAccess)},
                    {"emergency", static_cast<bool>(access & kEmergencyAccess)},
                    {"HOV", static_cast<bool>(access & kHOVAccess)},
                    {"pedestrian", static_cast<bool>(access & kPedestrianAccess)},
                    {"taxi", static_cast<bool>(access & kTaxiAccess)},
                    {"truck", static_cast<bool>(access & kTruckAccess)},
                    {"wheelchair", static_cast<bool>(access & kWheelchairAccess)},
                    {"moped", static_cast<bool>(access & kMopedAccess)},
                    {"motorcycle", static_cast<bool>(access & kMotorcycleAccess)}});
}

/**
 * Get the updated bit field.
 * @param dst  Data member to be updated.
 * @param src  Value to be updated.
 * @param pos  Position (pos element within the bit field).
 * @param len  Length of each element within the bit field.
 * @return  Returns an updated value for the bit field.
 */
uint32_t
OverwriteBits(const uint32_t dst, const uint32_t src, const uint32_t pos, const uint32_t len) {
  uint32_t shift = (pos * len);
  uint32_t mask = ((static_cast<uint32_t>(1) << len) - 1) << shift;
  return (dst & ~mask) | (src << shift);
}

/**
 * Get the updated bit field.
 * @param dst  Data member to be updated.
 * @param src  Value to be updated.
 * @param pos  Position (pos element within the bit field).
 * @return  Returns an updated value for the bit field.
 */
uint32_t OverwriteBit(const uint32_t dst, const uint32_t src, const uint32_t pos) {
  uint32_t mask = (static_cast<uint32_t>(1) << pos);
  return (dst & ~mask) | (src << pos);
}

} // namespace

namespace valhalla {
namespace baldr {

// Default constructor
DirectedEdge::DirectedEdge() {
  memset(this, 0, sizeof(DirectedEdge));
  weighted_grade_ = 6;
}

// Sets the end node of this directed edge.
void DirectedEdge::set_endnode(const GraphId& endnode) {
  endnode_ = endnode.value;
}

// Sets the free flow speed in KPH.
void DirectedEdge::set_free_flow_speed(const uint32_t speed) {
  if (speed > kMaxAssumedSpeed) {
    LOG_WARN("Exceeding maximum.  Free flow speed: " + std::to_string(speed));
    free_flow_speed_ = kMaxAssumedSpeed;
  } else {
    free_flow_speed_ = speed;
  }
}

// Sets the constrained flow speed in KPH.
void DirectedEdge::set_constrained_flow_speed(const uint32_t speed) {
  if (speed > kMaxAssumedSpeed) {
    LOG_WARN("Exceeding maximum.  Constrained flow speed: " + std::to_string(speed));
    constrained_flow_speed_ = kMaxAssumedSpeed;
  } else {
    constrained_flow_speed_ = speed;
  }
}

// Set the flag indicating the edge has predicted speed records.
void DirectedEdge::set_has_predicted_speed(const bool p) {
  has_predicted_speed_ = p;
}

// ------------------  Data offsets and flags for extended data -------------//

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

// Set the modes which have access restrictions on this edge.
void DirectedEdge::set_access_restriction(const uint32_t access) {
  access_restriction_ = access;
}

// Sets the sign flag.
void DirectedEdge::set_sign(const bool exit) {
  sign_ = exit;
}

// ------------------------- Geographic attributes ------------------------- //

// Sets the length of the edge in meters.
void DirectedEdge::set_length(const uint32_t length, bool should_error) {
  if (length > kMaxEdgeLength) {
    if (should_error) {
      // Consider this a catastrophic error.
      LOG_ERROR("Exceeding max. edge length: " + std::to_string(length));
      throw std::runtime_error("DirectedEdgeBuilder: exceeded maximum edge length");
    }
    LOG_WARN("Exceeding max. edge length: " + std::to_string(length));
    length_ = kMaxEdgeLength;
  } else {
    length_ = length;
  }
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

// Sets the curvature factor (0-15) for the edge.
void DirectedEdge::set_curvature(const uint32_t factor) {
  if (factor > kMaxCurvatureFactor) {
    LOG_WARN("Exceeding max. curvature factor: " + std::to_string(factor));
    curvature_ = 0;
  } else {
    curvature_ = factor;
  }
}

// Sets the lane connectivity flag.
void DirectedEdge::set_laneconnectivity(const bool lc) {
  lane_conn_ = lc;
}

// -------------------------- Routing attributes --------------------------- //

// Set if edge has a shoulder (Beneficial to know for cycling)
void DirectedEdge::set_shoulder(const bool shoulder) {
  shoulder_ = shoulder;
}

// Set if bikers need to dismount along the edge
void DirectedEdge::set_dismount(const bool dismount) {
  dismount_ = dismount;
}

// Set if a sidepath should be preffered when cycling over this one
void DirectedEdge::set_use_sidepath(const bool use_sidepath) {
  use_sidepath_ = use_sidepath;
}

// Set the flag indicating the edge is a dead end (no other driveable
// roads at the end node of this edge).
void DirectedEdge::set_deadend(const bool d) {
  deadend_ = d;
}

// Sets the flag indicating this edge has a toll or is it part of a toll road.
void DirectedEdge::set_toll(const bool toll) {
  toll_ = toll;
}

// Sets the flag indicating this edge has seasonal access
void DirectedEdge::set_seasonal(const bool seasonal) {
  seasonal_ = seasonal;
}

// Sets the destination only (private) flag. This indicates the edge should
// allow access only to locations that are destinations and not allow
// "through" traffic
void DirectedEdge::set_dest_only(const bool destonly) {
  dest_only_ = destonly;
}

// Sets the flag indicating this edge has is a tunnel of part of a tunnel.
void DirectedEdge::set_tunnel(const bool tunnel) {
  tunnel_ = tunnel;
}

// Sets the flag indicating this edge has is a bridge of part of a bridge.
void DirectedEdge::set_bridge(const bool bridge) {
  bridge_ = bridge;
}

// Sets the flag indicating this edge is indoor.
void DirectedEdge::set_indoor(const bool indoor) {
  indoor_ = indoor;
}

// Sets the hov type.
void DirectedEdge::set_hov_type(const HOVEdgeType hov_type) {
  hov_type_ = static_cast<uint32_t>(hov_type);
}

// Is this edge strictly hov? (if so, it could be hov2 or hov3, check the hov_type_)
bool DirectedEdge::is_hov_only() const {
  return (forwardaccess() & kHOVAccess) && !(forwardaccess() & kAutoAccess);
}

// Sets the flag indicating the  edge is part of a roundabout.
void DirectedEdge::set_roundabout(const bool roundabout) {
  roundabout_ = roundabout;
}

// Sets the flag indicating a traffic signal is present at the end of
// this edge.
void DirectedEdge::set_traffic_signal(const bool signal) {
  traffic_signal_ = signal;
}

// Sets the flag indicating a stop sign is present at the end of
// this edge.
void DirectedEdge::set_stop_sign(const bool sign) {
  stop_sign_ = sign;
}

// Sets the flag indicating a yield sign is present at the end of
// this edge.
void DirectedEdge::set_yield_sign(const bool sign) {
  yield_sign_ = sign;
}

// Set the forward flag. Tells if this directed edge is stored forward
// in edgeinfo (true) or reverse (false).
void DirectedEdge::set_forward(const bool forward) {
  forward_ = forward;
}

// Sets the not thru flag.
void DirectedEdge::set_not_thru(const bool not_thru) {
  not_thru_ = not_thru;
}

// Set the index of the opposing directed edge at the end node of this
// directed edge.
void DirectedEdge::set_opp_index(const uint32_t opp_index) {
  opp_index_ = opp_index;
}

// Sets the type of cycle lane (if any) present on this edge.
void DirectedEdge::set_cyclelane(const CycleLane cyclelane) {
  cycle_lane_ = static_cast<uint32_t>(cyclelane);
}

// Sets the bike network flag.
void DirectedEdge::set_bike_network(const bool bike_network) {
  bike_network_ = bike_network;
}

// Sets truck route flag.
void DirectedEdge::set_truck_route(const bool truck_route) {
  truck_route_ = truck_route;
}

// Sets the number of lanes
void DirectedEdge::set_lanecount(const uint32_t lanecount) {
  // Make sure we don't exceed max lane count. Also make sure lane count
  // is at least 1.
  if (lanecount > kMaxLaneCount) {
    LOG_WARN("Exceeding maximum lane count: " + std::to_string(lanecount));
    lanecount_ = kMaxLaneCount;
  } else if (lanecount == 0) {
    lanecount_ = 1;
  } else {
    lanecount_ = lanecount;
  }
}

// Set simple turn restrictions from the end of this directed edge.
// These are turn restrictions from one edge to another that apply to
// all vehicles, at all times.
void DirectedEdge::set_restrictions(const uint32_t mask) {
  if (mask >= (1 << kMaxTurnRestrictionEdges)) {
    LOG_WARN("Restrictions mask exceeds allowable limit: " + std::to_string(mask));
    restrictions_ = (mask & ((1 << kMaxTurnRestrictionEdges) - 1));
  } else {
    restrictions_ = mask;
  }
}

// Sets the specialized use type of this edge.
void DirectedEdge::set_use(const Use use) {
  use_ = static_cast<uint32_t>(use);
}

// Set the speed type (see graphconstants.h)
void DirectedEdge::set_speed_type(const SpeedType speed_type) {
  speed_type_ = static_cast<uint32_t>(speed_type);
}

// Set the country crossing flag.
void DirectedEdge::set_ctry_crossing(const bool crossing) {
  ctry_crossing_ = crossing;
}

// Set the access modes in the forward direction (bit field).
void DirectedEdge::set_forwardaccess(const uint32_t modes) {
  if (modes > kAllAccess) {
    LOG_ERROR("DirectedEdge: forward access exceeds maximum allowed: " + std::to_string(modes));
    forwardaccess_ = (modes & kAllAccess);
  } else {
    forwardaccess_ = modes;
  }
}

// Set all forward access modes to true (used for transition edges)
// Also sets reverse access so opposing edge matches.
void DirectedEdge::set_all_forward_access() {
  forwardaccess_ = kAllAccess;
  reverseaccess_ = kAllAccess;
}

// Set the access modes in the reverse direction (bit field).
void DirectedEdge::set_reverseaccess(const uint32_t modes) {
  if (modes > kAllAccess) {
    LOG_ERROR("DirectedEdge: reverse access exceeds maximum allowed: " + std::to_string(modes));
    reverseaccess_ = (modes & kAllAccess);
  } else {
    reverseaccess_ = modes;
  }
}

// -------------------------------- speed -------------------------- //

// Sets the average speed in KPH.
void DirectedEdge::set_speed(const uint32_t speed) {
  if (speed > kMaxAssumedSpeed) {
    LOG_WARN("Exceeding maximum.  Average speed: " + std::to_string(speed));
    speed_ = kMaxAssumedSpeed;
  } else {
    speed_ = speed;
  }
}

// Sets the truck speed in KPH.
void DirectedEdge::set_truck_speed(const uint32_t speed) {
  if (speed > kMaxAssumedSpeed) {
    LOG_WARN("Exceeding maximum.  Truck speed: " + std::to_string(speed));
    truck_speed_ = kMaxAssumedSpeed;
  } else {
    truck_speed_ = speed;
  }
}

// ----------------------------- Classification ---------------------------- //

// Sets the classification (importance) of this edge.
void DirectedEdge::set_classification(const RoadClass roadclass) {
  classification_ = static_cast<uint32_t>(roadclass);
}

// Sets the sac scale. Shows if edge is meant for hiking, and if so how difficult
// of a hike it is.
void DirectedEdge::set_sac_scale(const SacScale sac_scale) {
  sac_scale_ = static_cast<uint64_t>(sac_scale);
}

// Set the name consistency given the other edge's local index. This is limited
// to the first 8 local edge indexes.
void DirectedEdge::set_name_consistency(const uint32_t idx, const bool c) {
  if (idx > kMaxLocalEdgeIndex) {
    LOG_WARN("Local index exceeds max in set_name_consistency, skip");
  } else {
    name_consistency_ = OverwriteBit(name_consistency_, c, idx);
  }
}

// Sets the surface type (see baldr/graphconstants.h). This is a general
// indication of smoothness.
void DirectedEdge::set_surface(const Surface surface) {
  surface_ = static_cast<uint32_t>(surface);
}

// Sets the link flag indicating the edge is part of a link or connection
// (ramp or turn channel).
void DirectedEdge::set_link(const bool link) {
  link_ = link;
}

// Sets the intersection internal flag.
void DirectedEdge::set_internal(const bool internal) {
  internal_ = internal;
}

// Set the Complex restriction (per mode) for this directed edge at the start.
void DirectedEdge::set_start_restriction(const uint32_t modes) {
  start_restriction_ = modes;
}

// Set the Complex restriction (per mode) for this directed edge at the end.
void DirectedEdge::set_end_restriction(const uint32_t modes) {
  end_restriction_ = modes;
}

// Set the part of complex restriction flag.
void DirectedEdge::complex_restriction(const bool part_of) {
  complex_restriction_ = part_of;
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

// Sets the named flag.
void DirectedEdge::set_named(const bool named) {
  named_ = named;
}

// Set the flag for a sidewalk to the left of this directed edge.
void DirectedEdge::set_sidewalk_left(const bool sidewalk) {
  sidewalk_left_ = sidewalk;
}

// Set the flag for a sidewalk to the right of this directed edge.
void DirectedEdge::set_sidewalk_right(const bool sidewalk) {
  sidewalk_right_ = sidewalk;
}

// Sets the turn type given the prior edge's local index
// (index of the inbound edge).
void DirectedEdge::set_turntype(const uint32_t localidx, const Turn::Type turntype) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Exceeding max local index in set_turntype. Skipping");
  } else {
    turntype_ = OverwriteBits(turntype_, static_cast<uint32_t>(turntype), localidx, 3);
  }
}

// Set the flag indicating there is an edge to the left, in between
// the from edge and this edge.
void DirectedEdge::set_edge_to_left(const uint32_t localidx, const bool left) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Exceeding max local index in set_edge_to_left. Skipping");
  } else {
    edge_to_left_ = OverwriteBits(edge_to_left_, left, localidx, 1);
  }
}

// Set the stop impact when transitioning from the prior edge (given
// by the local index of the corresponding inbound edge at the node).
void DirectedEdge::set_stopimpact(const uint32_t localidx, const uint32_t stopimpact) {
  if (stopimpact > kMaxStopImpact) {
    LOG_WARN("Exceeding maximum stop impact: " + std::to_string(stopimpact));
    stopimpact_.s.stopimpact = OverwriteBits(stopimpact_.s.stopimpact, kMaxStopImpact, localidx, 3);
  } else {
    stopimpact_.s.stopimpact = OverwriteBits(stopimpact_.s.stopimpact, stopimpact, localidx, 3);
  }
}

// Set the unique transit line Id.
void DirectedEdge::set_lineid(const uint32_t lineid) {
  stopimpact_.lineid = lineid;
}

// Set the flag indicating there is an edge to the right, in between
// the from edge and this edge.
void DirectedEdge::set_edge_to_right(const uint32_t localidx, const bool right) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Exceeding max local index in set_edge_to_right. Skipping");
  } else {
    stopimpact_.s.edge_to_right = OverwriteBits(stopimpact_.s.edge_to_right, right, localidx, 1);
  }
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

// Set the flag for whether this edge represents a shortcut between 2 nodes.
void DirectedEdge::set_shortcut(const uint32_t shortcut) {
  // 0 is not a valid shortcut
  if (shortcut == 0) {
    LOG_WARN("Invalid shortcut mask = 0");
    return;
  }

  // Set the shortcut mask if within the max number of masked shortcut edges
  if (shortcut <= kMaxShortcutsFromNode) {
    shortcut_ = (1 << (shortcut - 1));
  }

  // Set the is_shortcut flag
  is_shortcut_ = true;
}

// Set the flag for whether this edge is superseded by a shortcut edge.
void DirectedEdge::set_superseded(const uint32_t superseded) {
  if (superseded > kMaxShortcutsFromNode) {
    LOG_WARN("Exceeding max shortcut edges from a node: " + std::to_string(superseded));
  } else if (superseded == 0) {
    superseded_ = 0;
  } else {
    superseded_ = (1 << (superseded - 1));
  }
}

// Set the flag indicating whether the end node of this directed edge is in
// a different tile
void DirectedEdge::set_leaves_tile(const bool leaves_tile) {
  leaves_tile_ = leaves_tile;
}

// Sets the maximum upward slope. If slope is negative, 0 is set.
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

// Sets the maximum downward slope. If slope is positive, 0 is set.
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

void DirectedEdge::set_bss_connection(const bool bss_connection) {
  bss_connection_ = bss_connection;
}

// Json representation
json::MapPtr DirectedEdge::json() const {
  json::MapPtr map = json::map({
      {"end_node", endnode().json()},
      {"speeds", json::map({
                     {"default", static_cast<uint64_t>(speed_)},
                     {"type", to_string(static_cast<SpeedType>(speed_type_))},
                     {"free_flow", static_cast<uint64_t>(free_flow_speed_)},
                     {"constrained_flow", static_cast<uint64_t>(constrained_flow_speed_)},
                     {"predicted", static_cast<bool>(has_predicted_speed_)},
                 })},
      //{"opp_index", static_cast<bool>(opp_index_)},
      //{"edge_info_offset", static_cast<uint64_t>(edgeinfo_offset_)},
      //{"restrictions", restrictions_},
      {"access_restriction", static_cast<bool>(access_restriction_)},
      {"start_restriction", access_json(start_restriction_)},
      {"end_restriction", access_json(end_restriction_)},
      {"part_of_complex_restriction", static_cast<bool>(complex_restriction_)},
      {"has_sign", static_cast<bool>(sign_)},
      {"toll", static_cast<bool>(toll_)},
      {"seasonal", static_cast<bool>(seasonal_)},
      {"destination_only", static_cast<bool>(dest_only_)},
      {"tunnel", static_cast<bool>(tunnel_)},
      {"bridge", static_cast<bool>(bridge_)},
      {"round_about", static_cast<bool>(roundabout_)},
      {"traffic_signal", static_cast<bool>(traffic_signal_)},
      {"forward", static_cast<bool>(forward_)},
      {"not_thru", static_cast<bool>(not_thru_)},
      {"stop_sign", static_cast<bool>(stop_sign_)},
      {"yield_sign", static_cast<bool>(yield_sign_)},
      {"cycle_lane", to_string(static_cast<CycleLane>(cycle_lane_))},
      {"bike_network", static_cast<bool>(bike_network_)},
      {"truck_route", static_cast<bool>(truck_route_)},
      {"lane_count", static_cast<uint64_t>(lanecount_)},
      {"country_crossing", static_cast<bool>(ctry_crossing_)},
      {"sidewalk_left", static_cast<bool>(sidewalk_left_)},
      {"sidewalk_right", static_cast<bool>(sidewalk_right_)},
      {"sac_scale", to_string(static_cast<SacScale>(sac_scale_))},
      {"geo_attributes",
       json::map({
           {"length", static_cast<uint64_t>(length_)},
           {"weighted_grade", json::fixed_t{static_cast<double>(weighted_grade_ - 6.0) / .6, 2}},
           {"max_up_slope", json::fixed_t{static_cast<double>(max_up_slope()), 2}},
           {"max_down_slope", json::fixed_t{static_cast<double>(max_down_slope()), 2}},
           {"curvature", static_cast<uint64_t>(curvature_)},
       })},
      {"access", access_json(forwardaccess_)},
      //{"access", access_json(reverseaccess_)},
      {"classification", json::map({
                             {"classification", to_string(static_cast<RoadClass>(classification_))},
                             {"use", to_string(static_cast<Use>(use_))},
                             {"surface", to_string(static_cast<Surface>(surface_))},
                             {"link", static_cast<bool>(link_)},
                             {"internal", static_cast<bool>(internal_)},
                         })},
      /*{"hierarchy", json::map({
        {"local_edge_index", static_cast<uint64_t>(localedgeidx_)},
        {"opposing_local_index", static_cast<uint64_t>(opp_local_idx_)},
        {"shortcut_mask", static_cast<uint64_t>(shortcut_)},
        {"superseded_mask", static_cast<uint64_t>(superseded_)},
        {"shortcut", static_cast<bool>(is_shortcut_)},
      })},*/
  });

  if (is_hov_only()) {
    map->emplace("hov_type", to_string(static_cast<HOVEdgeType>(hov_type_)));
  }

  return map;
}

} // namespace baldr
} // namespace valhalla
