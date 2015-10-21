#include "mjolnir/directededgebuilder.h"
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/midgard/logging.h>

#include <algorithm>

namespace {

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

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Default constructor
DirectedEdgeBuilder::DirectedEdgeBuilder()
  : DirectedEdge() {
}

// Constructor with parameters
DirectedEdgeBuilder::DirectedEdgeBuilder(
                   const OSMWay& way, const GraphId& endnode,
                   const bool forward, const uint32_t length,
                   const uint32_t speed, const uint32_t truck_speed,
                   const baldr::Use use, const RoadClass rc,
                   const uint32_t localidx, const bool signal,
                   const uint32_t restrictions, const uint32_t bike_network)
     :  DirectedEdge() {
  set_endnode(endnode);
  set_length(length);
  set_use(use);
  set_speed(speed);    // KPH
  set_truck_speed(truck_speed); // KPH
  set_ferry(way.ferry());
  set_railferry(way.rail());
  set_toll(way.toll());
  set_dest_only(way.destination_only());

  if (bike_network)
    set_bike_network(way.bike_network() | bike_network);
  else
    set_bike_network(way.bike_network());

  set_truck_route(way.truck_route());

  if (!way.destination_only())
    set_dest_only(way.no_thru_traffic());

  set_surface(way.surface());
  set_cyclelane(way.cyclelane());
  set_tunnel(way.tunnel());
  set_roundabout(way.roundabout());
  set_bridge(way.bridge());
  set_link(way.link());
  set_classification(rc);
  set_localedgeidx(localidx);
  set_restrictions(restrictions);
  set_traffic_signal(signal);

  set_speed_type(way.tagged_speed() ?
        SpeedType::kTagged : SpeedType::kClassified);

  // Set forward flag and access (based on direction)
  set_forward(forward);
  set_caraccess(forward, way.auto_forward());
  set_truckaccess(forward, way.truck_forward());
  set_busaccess(forward, way.bus_forward());
  set_bicycleaccess(forward, way.bike_forward());
  set_emergencyaccess(forward, way.emergency_forward());
  set_pedestrianaccess(forward, way.pedestrian());

  // Access for opposite direction
  set_caraccess(!forward, way.auto_backward());
  set_truckaccess(!forward, way.truck_backward());
  set_busaccess(!forward, way.bus_backward());
  set_bicycleaccess(!forward, way.bike_backward());
  set_emergencyaccess(!forward, way.emergency_backward());
  set_pedestrianaccess(!forward, way.pedestrian());
}

// Sets the end node of this directed edge.
void DirectedEdgeBuilder::set_endnode(const GraphId& endnode) {
  endnode_ = endnode;
}

// Get the offset to the common edge data.
void DirectedEdgeBuilder::set_edgeinfo_offset(const uint32_t offset) {
  if (offset > kMaxEdgeInfoOffset) {
    // Consider this a catastrophic error
    LOG_ERROR("Exceeded maximum edgeinfo offset: " + std::to_string(offset));
    throw std::runtime_error("DirectedEdgeBuilder: exceeded maximum edgeinfo offset");
  } else {
    edgeinfo_offset_ = offset;
  }
}

// Set the access.
void DirectedEdgeBuilder::set_access(const uint32_t access) {
  access_restriction_ = access;
}

// Sets the exit flag.
void DirectedEdgeBuilder::set_exitsign(const bool exit) {
  exitsign_ = exit;
}

// Sets the length of the edge in meters.
void DirectedEdgeBuilder::set_length(const uint32_t length) {
  if (length > kMaxEdgeLength) {
    LOG_WARN("Exceeding max. edge length: " + std::to_string(length));
    length_ = kMaxEdgeLength;
  } else {
    length_ = length;
  }
}

// Sets the weighted_grade factor (0-15) for the edge.
void DirectedEdgeBuilder::set_weighted_grade(const uint32_t factor) {
  if (factor > kMaxGradeFactor) {
    LOG_WARN("Exceeding max. weighted grade factor: " + std::to_string(factor));
    weighted_grade_ = 6;
  } else {
    weighted_grade_ = factor;
  }
}

// Sets the curvature factor (0-15) for the edge.
void DirectedEdgeBuilder::set_curvature(const uint32_t factor) {
  if (factor > kMaxCurvatureFactor) {
    LOG_WARN("Exceeding max. curvature factor: " + std::to_string(factor));
    curvature_ = 0;
  } else {
    curvature_ = factor;
  }
}

// Set the flag indicating driving is on the right hand side of the road
// along this edge?
void DirectedEdgeBuilder::set_drive_on_right(const bool rsd) {
  drive_on_right_ = rsd;
}

// Sets the flag indicating this edge is a ferry (or part of a ferry).
void DirectedEdgeBuilder::set_ferry(const bool ferry) {
  ferry_ = ferry;
}

// Sets the flag indicating this edge is a rail ferry (or part of a
// rail ferry). Example is the EuroTunnel (Channel Tunnel).
void DirectedEdgeBuilder::set_railferry(const bool railferry) {
  railferry_ = railferry;
}

// Sets the flag indicating this edge has a toll or is it part of a toll road.
void DirectedEdgeBuilder::set_toll(const bool toll) {
  toll_ = toll;
}

// Sets the flag indicating this edge has seasonal access
void DirectedEdgeBuilder::set_seasonal(const bool seasonal) {
  seasonal_ = seasonal;
}

// Sets the destination only (private) flag. This indicates the edge should
// allow access only to locations that are destinations and not allow
// "through" traffic
void DirectedEdgeBuilder::set_dest_only(const bool destonly) {
  dest_only_ = destonly;
}

// Sets the flag indicating this edge has is a tunnel of part of a tunnel.
void DirectedEdgeBuilder::set_tunnel(const bool tunnel) {
  tunnel_ = tunnel;
}

// Sets the flag indicating this edge has is a bridge of part of a bridge.
void DirectedEdgeBuilder::set_bridge(const bool bridge) {
  bridge_ = bridge;
}

// Sets the flag indicating the  edge is part of a roundabout.
void DirectedEdgeBuilder::set_roundabout(const bool roundabout) {
  roundabout_ = roundabout;
}

// Sets the flag indicating the edge is unreachable by driving. This can
// happen if a driveable edge is surrounded by pedestrian only edges (e.g.
// in a city center) or is not properly connected to other edges.
void DirectedEdgeBuilder::set_unreachable(const bool unreachable) {
  unreachable_ = unreachable;
}

// Sets the flag indicating a traffic signal is present at the end of
// this edge.
void DirectedEdgeBuilder::set_traffic_signal(const bool signal) {
  traffic_signal_ = signal;
}

// Set the forward flag. Tells if this directed edge is stored forward
// in edgeinfo (true) or reverse (false).
void DirectedEdgeBuilder::set_forward(const bool forward) {
  forward_ = forward;
}

// Sets the not thru flag.
void DirectedEdgeBuilder::set_not_thru(const bool not_thru) {
  not_thru_ = not_thru;
}

// Set the index of the opposing directed edge at the end node of this
// directed edge.
void DirectedEdgeBuilder::set_opp_index(const uint32_t opp_index) {
  opp_index_ = opp_index;
}

// Sets the type of cycle lane (if any) present on this edge.
void DirectedEdgeBuilder::set_cyclelane(const CycleLane cyclelane) {
  cycle_lane_ = static_cast<uint32_t>(cyclelane);
}

// Sets the bike network mask indicating which (if any) bicycle networks are
// along this edge. See baldr/directededge.h for definitions.
void DirectedEdgeBuilder::set_bike_network(const uint32_t bike_network) {
  if (bike_network > kMaxBicycleNetwork) {
    LOG_WARN("Bicycle Network mask exceeds maximum: " +
              std::to_string(bike_network));
    bike_network_ = 0;
  } else {
    bike_network_ = bike_network;
  }
}

// Sets truck route flag.
void DirectedEdgeBuilder::set_truck_route(const bool truck_route) {
  truck_route_ = truck_route;
}

// Sets the number of lanes
void DirectedEdgeBuilder::set_lanecount(const uint32_t lanecount) {
  // TOTO - make sure we don't exceed some max value
  if (lanecount > kMaxLaneCount) {
    LOG_WARN("Exceeding maximum lane count: " + std::to_string(lanecount));
    lanecount_ = kMaxLaneCount;
  } else {
    lanecount_ = lanecount;
  }
}

// Set simple turn restrictions from the end of this directed edge.
// These are turn restrictions from one edge to another that apply to
// all vehicles, at all times.
void DirectedEdgeBuilder::set_restrictions(const uint32_t mask) {
  if (mask >= (1 << kMaxTurnRestrictionEdges)) {
    LOG_WARN("Restrictions mask exceeds allowable limit: " +
                std::to_string(mask));
    restrictions_ &= ((1 << kMaxTurnRestrictionEdges) - 1);
  } else {
    restrictions_ = mask;
  }
}

// Sets the specialized use type of this edge.
void DirectedEdgeBuilder::set_use(const Use use) {
  use_ = static_cast<uint32_t>(use);
}

// Set the speed type (see graphconstants.h)
void DirectedEdgeBuilder::set_speed_type(const SpeedType speed_type) {
  speed_type_ = static_cast<uint32_t>(speed_type);
}

// Set the country crossing flag.
void DirectedEdgeBuilder::set_ctry_crossing(const bool crossing) {
  ctry_crossing_ = crossing;
}

// Set all forward access modes to true (used for transition edges)
void DirectedEdgeBuilder::set_all_forward_access() {
  forwardaccess_ = kAllAccess;
}

// Sets the car access of the edge in the specified direction.
void DirectedEdgeBuilder::set_caraccess(const bool forward, const bool car) {
  if (forward && car) {
    forwardaccess_ |= kAutoAccess;
  } else if (!forward && car) {
    reverseaccess_ |= kAutoAccess;
  }
}

// Sets the taxi access of the edge in the specified direction.
void DirectedEdgeBuilder::set_taxiaccess(const bool forward, const bool taxi) {
  if (forward && taxi) {
    forwardaccess_ |= kTaxiAccess;
  } else if (!forward && taxi) {
    reverseaccess_ |= kTaxiAccess;
  }
}

// Sets the truck access of the edge in the specified direction.
void DirectedEdgeBuilder::set_truckaccess(const bool forward,
                                          const bool truck) {
  if (forward && truck) {
    forwardaccess_ |= kTruckAccess;
  } else if (!forward && truck) {
    reverseaccess_ |= kTruckAccess;
  }
}

// Sets the pedestrian access of the edge in the specified direction.
void DirectedEdgeBuilder::set_pedestrianaccess(const bool forward,
                                               const bool pedestrian) {
  if (forward && pedestrian) {
    forwardaccess_ |= kPedestrianAccess;
  } else if (!forward && pedestrian) {
    reverseaccess_ |= kPedestrianAccess;
  }
}

// Sets the bicycle access of the edge in the specified direction.
void DirectedEdgeBuilder::set_bicycleaccess(const bool forward,
                                            const bool bicycle) {
  if (forward && bicycle) {
    forwardaccess_ |= kBicycleAccess;
  } else if (!forward && bicycle) {
    reverseaccess_ |= kBicycleAccess;
  }
}

// Sets the emergency access of the edge in the specified direction.
void DirectedEdgeBuilder::set_emergencyaccess(const bool forward,
                                              const bool emergency) {
  if (forward && emergency) {
    forwardaccess_ |= kEmergencyAccess;
  } else if (!forward && emergency) {
    reverseaccess_ |= kEmergencyAccess;
  }
}

// Sets the bus access of the edge in the specified direction.
void DirectedEdgeBuilder::set_busaccess(const bool forward,
                                        const bool bus) {
  if (forward && bus) {
    forwardaccess_ |= kBusAccess;
  } else if (!forward && bus) {
    reverseaccess_ |= kBusAccess;
  }
}

// Sets the high occupancy vehicle (HOV) access of the edge in the specified
// direction.
void DirectedEdgeBuilder::set_hovaccess(const bool forward, const bool hov) {
  if (forward && hov) {
    forwardaccess_ |= kHOVAccess;
  } else if (!forward && hov) {
    reverseaccess_ |= kHOVAccess;
  }
}

// Sets the speed in KPH.
void DirectedEdgeBuilder::set_speed(const uint32_t speed) {
  if (speed > kMaxSpeed) {
    LOG_WARN("Exceeding maximum speed: " + std::to_string(speed));
    speed_ = kMaxSpeed;
  } else {
    speed_ = speed;
  }
}

// Sets the truck speed in KPH.
void DirectedEdgeBuilder::set_truck_speed(const uint32_t speed) {
  if (speed > kMaxSpeed) {
    LOG_WARN("Exceeding maximum truck speed: " + std::to_string(speed));
    truck_speed_ = kMaxSpeed;
  } else {
    truck_speed_ = speed;
  }
}

// Sets the classification (importance) of this edge.
void DirectedEdgeBuilder::set_classification(const RoadClass roadclass) {
  classification_= static_cast<uint32_t>(roadclass);
}

// Sets the surface type (see baldr/graphconstants.h). This is a general
// indication of smoothness.
void DirectedEdgeBuilder::set_surface(const Surface surface) {
  surface_ = static_cast<uint32_t>(surface);
}

// Sets the link flag indicating the edge is part of a link or connection
// (ramp or turn channel).
void DirectedEdgeBuilder::set_link(const bool link) {
  link_ = link;
}

// Sets the intersection internal flag.
void DirectedEdgeBuilder::set_internal(const bool internal) {
  internal_ = internal;
}

// Sets the turn type given the prior edge's local index
// (index of the inbound edge).
void DirectedEdgeBuilder::set_turntype(const uint32_t localidx,
                                       const Turn::Type turntype) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Exceeding max local index in set_turntype. Skipping");
  } else {
    turntype_ = OverwriteBits(turntype_,
                   static_cast<uint32_t>(turntype), localidx, 3);
  }
}

// Set the flag indicating there is an edge to the left, in between
// the from edge and this edge.
void DirectedEdgeBuilder::set_edge_to_left(const uint32_t localidx,
                                           const bool left) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Exceeding max local index in set_edge_to_left. Skipping");
  } else {
    edge_to_left_ = OverwriteBits(edge_to_left_, left, localidx, 1);
  }
}

// Set the stop impact when transitioning from the prior edge (given
// by the local index of the corresponding inbound edge at the node).
void DirectedEdgeBuilder::set_stopimpact(const uint32_t localidx,
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

// Set the flag indicating there is an edge to the right, in between
// the from edge and this edge.
void DirectedEdgeBuilder::set_edge_to_right(const uint32_t localidx,
                                            const bool right) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Exceeding max local index in set_edge_to_right. Skipping");
  } else {
    stopimpact_.s.edge_to_right = OverwriteBits(stopimpact_.s.edge_to_right,
                                right, localidx, 1);
  }
}

// Set the unique transit line Id.
void DirectedEdgeBuilder::set_lineid(const uint32_t lineid) {
  stopimpact_.lineid = lineid;
}

// Set the index of the directed edge on the local level of the graph
// hierarchy. This is used for turn restrictions so the edges can be
// identified on the different levels.
void DirectedEdgeBuilder::set_localedgeidx(const uint32_t idx) {
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
void DirectedEdgeBuilder::set_opp_local_idx(const uint32_t idx) {
  if (idx > kMaxEdgesPerNode) {
    LOG_WARN("Exceeding max edges in opposing local index: " + std::to_string(idx));
    opp_local_idx_ = kMaxEdgesPerNode;
  } else {
    opp_local_idx_ = idx;
  }
}

// Set the flag for whether this edge represents a shortcut between 2 nodes.
void DirectedEdgeBuilder::set_shortcut(const uint32_t shortcut) {
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

// Set the flag for whether this edge is superseded by a shortcut edge.
void DirectedEdgeBuilder::set_superseded(const uint32_t superseded) {
  if (superseded > kMaxShortcutsFromNode) {
      LOG_WARN("Exceeding max shortcut edges from a node: " + std::to_string(superseded));
  } else {
    superseded_ = (1 << (superseded-1));
  }
}

// Set the flag for whether this edge represents a transition up one level
// in the hierarchy.
void DirectedEdgeBuilder::set_trans_up(const bool trans_up) {
  trans_up_ = trans_up;
}

// Set the flag for whether this edge represents a transition down one level
// in the hierarchy.
void DirectedEdgeBuilder::set_trans_down(const bool trans_down) {
  trans_down_ = trans_down;
}

}
}
