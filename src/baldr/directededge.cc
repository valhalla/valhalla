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
DirectedEdge::DirectedEdge() {
  memset(this, 0, sizeof(DirectedEdge));
  weighted_grade_ = 6;
}

// Gets the end node of this directed edge.
GraphId DirectedEdge::endnode() const {
  return endnode_;
}

// ------------------  Data offsets and flags for extended data -------------//

// Get the offset to the common edge information.
uint64_t DirectedEdge::edgeinfo_offset() const {
  return edgeinfo_offset_;
}

// Get the general restriction or access condition (per mode) for this directed edge.
uint64_t DirectedEdge::access_restriction() const {
  return access_restriction_;
}

// Does this directed edge have exit signs.
bool DirectedEdge::exitsign() const {
  return exitsign_;
}

// ------------------------- Geographic attributes ------------------------- //

// Gets the length of the edge in meters.
uint32_t DirectedEdge::length() const {
  return length_;
}

// Gets the weighted grade factor (0-15).
uint32_t DirectedEdge::weighted_grade() const {
  return weighted_grade_;
}

// Get the road curvature factor. (0-15).
uint32_t DirectedEdge::curvature() const {
  return curvature_;
}

// -------------------------- Routing attributes --------------------------- //

// Is driving on the right hand side of the road along this edge?
bool DirectedEdge::drive_on_right() const {
  return drive_on_right_;
}

// Is this edge part of a ferry?
bool DirectedEdge::ferry() const {
  return ferry_;
}

// Is this edge part of a rail ferry?
bool DirectedEdge::railferry() const {
  return railferry_;
}

// Does this edge have a toll or is it part of a toll road?
bool DirectedEdge::toll() const {
  return toll_;
}

// Does this edge have a seasonal access (e.g., closed in the winter)?
bool DirectedEdge::seasonal() const {
  return seasonal_;
}

// Is this edge part of a private or no through road that allows access
// only if required to get to a destination?
bool DirectedEdge::destonly() const {
  return dest_only_;
}

// Is this edge part of a tunnel?
bool DirectedEdge::tunnel() const {
  return tunnel_;
}

// Is this edge part of a bridge?
bool DirectedEdge::bridge() const {
  return bridge_;
}

// Is this edge part of a roundabout?
bool DirectedEdge::roundabout() const {
  return roundabout_;
}

// Is this edge is unreachable by driving. This can happen if a driveable
// edge is surrounded by pedestrian only edges (e.g. in a city center) or
// is not properly connected to other edges.
bool DirectedEdge::unreachable() const {
  return unreachable_;
}

// A traffic signal occurs at the end of this edge.
bool DirectedEdge::traffic_signal() const {
  return traffic_signal_;
}

// Is this directed edge stored forward in edgeinfo (true) or
// reverse (false).
bool DirectedEdge::forward() const {
  return forward_;
}

// Does this edge lead into a no-thru region
bool DirectedEdge::not_thru() const {
  return not_thru_;
}

// Get the index of the opposing directed edge at the end node of this
// directed edge.
uint32_t DirectedEdge::opp_index() const {
  return opp_index_;
}

// Get the cycle lane type along this edge.
CycleLane DirectedEdge::cyclelane() const {
  return static_cast<CycleLane>(cycle_lane_);
}

// Gets the bike network mask
uint32_t DirectedEdge::bike_network() const {
  return bike_network_;
}

// Gets the truck network flag
bool DirectedEdge::truck_route() const {
  return truck_route_;
}

// Gets the lane count
uint32_t DirectedEdge::lanecount() const {
  return lanecount_;
}

// Get the mask of simple turn restrictions from the end of this directed
// edge. These are turn restrictions from one edge to another that apply to
// all vehicles, at all times.
uint32_t DirectedEdge::restrictions() const {
  return restrictions_;
}

// Get the use type of this edge.
Use DirectedEdge::use() const {
  return static_cast<Use>(use_);
}

// Is this edge a transit line (bus or rail)?
bool DirectedEdge::IsTransitLine() const {
  return (use() == Use::kRail || use() == Use::kBus);
}

// Get the speed type (see graphconstants.h)
SpeedType DirectedEdge::speed_type() const {
  return static_cast<SpeedType>(speed_type_);
}

// Get the country crossing flag.
bool DirectedEdge::ctry_crossing() const {
  return ctry_crossing_;
}

// Get the access modes in the forward direction (bit field).
uint32_t DirectedEdge::forwardaccess() const {
  return forwardaccess_;
}

// Get the access modes in the reverse direction (bit field).
uint32_t DirectedEdge::reverseaccess() const {
  return reverseaccess_;
}

// -------------------------------- speed -------------------------- //

// Gets the speed in KPH.
uint32_t DirectedEdge::speed() const {
  return speed_;
}

// Gets the truck speed in KPH.
uint32_t DirectedEdge::truck_speed() const {
  return truck_speed_;
}

// ----------------------------- Classification ---------------------------- //
// Get the road classification.
RoadClass DirectedEdge::classification() const {
  return static_cast<RoadClass>(classification_);
}

// Is the edge considered unpaved?
bool DirectedEdge::unpaved() const {
  return (surface() >= Surface::kCompacted);
}

// Get the smoothness of this edge.
Surface DirectedEdge::surface() const {
  return static_cast<Surface>(surface_);
}

// Is this edge a link / ramp?
bool DirectedEdge::link() const {
  return link_;
}

// Gets the intersection internal flag.
bool DirectedEdge::internal() const {
  return internal_;
}

// Gets the turn type given the prior edge's local index
Turn::Type DirectedEdge::turntype(const uint32_t localidx) const {
  // Turn type is 3 bits per index
  uint32_t shift = localidx * 3;
  return static_cast<Turn::Type>(
      ((turntype_ & (7 << shift))) >> shift);
}

// Is there an edge to the left, in between the from edge and this edge.
bool DirectedEdge::edge_to_left(const uint32_t localidx) const {
  return (edge_to_left_ & (1 << localidx));
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
  return localedgeidx_;
}

// Get the index of the opposing directed edge on the local hierarchy level
// at the end node of this directed edge. Only stored for the first 8 edges
// so it can be used for edge transition costing.
uint32_t DirectedEdge::opp_local_idx() const {
  return opp_local_idx_;
}

// Get the shortcut mask indicating the edge index that is superseded by
// this shortcut (0 if not a shortcut)
uint32_t DirectedEdge::shortcut() const {
 return shortcut_;
}

// Get the mask of the shortcut edge that supersedes this edge. 0 indicates
// no shortcut edge supersedes this edge.
uint32_t DirectedEdge::superseded() const {
  return superseded_;
}

// Does this edge represent a transition up one level in the hierarchy.
bool DirectedEdge::trans_up() const {
  return trans_up_;
}

// Does this edge represent a transition down one level in the hierarchy.
bool DirectedEdge::trans_down() const {
  return trans_down_;
}

// Is this directed edge a shortcut?
bool DirectedEdge::is_shortcut() const {
  return is_shortcut_;
}

// Does this directed edge end in a different tile.
bool DirectedEdge::leaves_tile() const {
  return leaves_tile_;
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
    {"start_complex_restriction", static_cast<bool>(start_complex_restriction_)},
    {"end_complex_restriction", static_cast<bool>(end_complex_restriction_)},
    {"has_exit_sign", static_cast<bool>(exitsign_)},
    {"drive_on_right", static_cast<bool>(drive_on_right_)},
    {"ferry", static_cast<bool>(ferry_)},
    {"rail_ferry", static_cast<bool>(railferry_)},
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
      {"weighted_grade", json::fp_t{static_cast<double>(weighted_grade_ - 6.5) / .6, 2}},
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
