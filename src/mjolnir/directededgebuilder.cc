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
DirectedEdgeBuilder::DirectedEdgeBuilder(const OSMWay& way,
                   const GraphId& endnode,
                   const bool forward, const uint32_t length,
                   const uint32_t speed, const baldr::Use use,
                   const RoadClass rc, const uint32_t localidx,
                   const bool signal, const uint32_t restrictions,
                   const uint32_t bike_network)
     :  DirectedEdge() {
  set_endnode(endnode);
  set_length(length);
  set_use(use);
  set_speed(speed);    // KPH
  set_ferry(way.ferry());
  set_railferry(way.rail());
  set_toll(way.toll());
  set_dest_only(way.destination_only());

  if (bike_network)
    set_bikenetwork(way.bike_network() | bike_network);
  else
    set_bikenetwork(way.bike_network());

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
  set_busaccess(forward, way.bus_forward());
  set_bicycleaccess(forward, way.bike_forward());
  set_pedestrianaccess(forward, way.pedestrian());

  // Access for opposite direction
  set_caraccess(!forward, way.auto_backward());
  set_busaccess(!forward, way.bus_backward());
  set_bicycleaccess(!forward, way.bike_backward());
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
    dataoffsets_.edgeinfo_offset = offset;
  }
}

// Set the access conditions flag.
void DirectedEdgeBuilder::set_access_conditions(const bool access) {
  dataoffsets_.access_conditions = access;
}

// Set the flag indicating this directed edge starts a simple, timed turn
// restriction (from one edge to another).
void DirectedEdgeBuilder::start_ttr(const bool ttr) {
  dataoffsets_.start_ttr = ttr;
}

// Set the flag indicating this directed edge starts a multi-edge turn
// restriction. These are restrictions from one edge to another via one or
// more edges. Can include times.
void DirectedEdgeBuilder::start_mer(const bool mer) {
  dataoffsets_.start_mer = mer;
}

// Set the flag indicating this directed edge ends a multi-edge turn
// restriction. These are restrictions from one edge to another via one
// or more edges. This is the end edge of such a restriction.
// Can include times.
void DirectedEdgeBuilder::end_mer(const bool mer) {
  dataoffsets_.end_mer = mer;
}

// Sets the exit flag.
void DirectedEdgeBuilder::set_exitsign(const bool exit) {
  dataoffsets_.exitsign = exit;
}

// Sets the length of the edge in meters.
void DirectedEdgeBuilder::set_length(const uint32_t length) {
  if (length > kMaxEdgeLength) {
    LOG_WARN("Exceeding max. edge length: " + std::to_string(length));
    geoattributes_.length = kMaxEdgeLength;
  } else {
    geoattributes_.length = length;
  }
}

// Sets the weighted_grade factor (0-15) for the edge.
void DirectedEdgeBuilder::set_weighted_grade(const uint32_t factor) {
  if (factor > kMaxGradeFactor) {
    LOG_WARN("Exceeding max. weighted grade factor: " + std::to_string(factor));
    geoattributes_.weighted_grade = 6;
  } else {
    geoattributes_.weighted_grade = factor;
  }
}

// Sets the curvature factor (0-15) for the edge.
void DirectedEdgeBuilder::set_curvature(const uint32_t factor) {
  if (factor > kMaxCurvatureFactor) {
    LOG_WARN("Exceeding max. curvature factor: " + std::to_string(factor));
    geoattributes_.curvature = 0;
  } else {
    geoattributes_.curvature = factor;
  }
}

// Set the flag indicating driving is on the right hand side of the road
// along this edge?
void DirectedEdgeBuilder::set_drive_on_right(const bool rsd) {
  attributes_.drive_on_right = rsd;
}

// Sets the flag indicating this edge is a ferry (or part of a ferry).
void DirectedEdgeBuilder::set_ferry(const bool ferry) {
  attributes_.ferry = ferry;
}

// Sets the flag indicating this edge is a rail ferry (or part of a
// rail ferry). Example is the EuroTunnel (Channel Tunnel).
void DirectedEdgeBuilder::set_railferry(const bool railferry) {
  attributes_.railferry = railferry;
}

// Sets the flag indicating this edge has a toll or is it part of a toll road.
void DirectedEdgeBuilder::set_toll(const bool toll) {
  attributes_.toll = toll;
}

// Sets the flag indicating this edge has seasonal access
void DirectedEdgeBuilder::set_seasonal(const bool seasonal) {
  attributes_.seasonal = seasonal;
}

// Sets the destination only (private) flag. This indicates the edge should
// allow access only to locations that are destinations and not allow
// "through" traffic
void DirectedEdgeBuilder::set_dest_only(const bool destonly) {
  attributes_.dest_only = destonly;
}

// Sets the flag indicating this edge has is a tunnel of part of a tunnel.
void DirectedEdgeBuilder::set_tunnel(const bool tunnel) {
  attributes_.tunnel = tunnel;
}

// Sets the flag indicating this edge has is a bridge of part of a bridge.
void DirectedEdgeBuilder::set_bridge(const bool bridge) {
  attributes_.bridge = bridge;
}

// Sets the flag indicating the  edge is part of a roundabout.
void DirectedEdgeBuilder::set_roundabout(const bool roundabout) {
  attributes_.roundabout = roundabout;
}

// Sets the flag indicating the edge is unreachable by driving. This can
// happen if a driveable edge is surrounded by pedestrian only edges (e.g.
// in a city center) or is not properly connected to other edges.
void DirectedEdgeBuilder::set_unreachable(const bool unreachable) {
  attributes_.unreachable = unreachable;
}

// Sets the flag indicating a traffic signal is present at the end of
// this edge.
void DirectedEdgeBuilder::set_traffic_signal(const bool signal) {
  attributes_.traffic_signal = signal;
}

// Set the forward flag. Tells if this directed edge is stored forward
// in edgeinfo (true) or reverse (false).
void DirectedEdgeBuilder::set_forward(const bool forward) {
  attributes_.forward = forward;
}

// Sets the not thru flag.
void DirectedEdgeBuilder::set_not_thru(const bool not_thru) {
  attributes_.not_thru = not_thru;
}

// Set the index of the opposing directed edge at the end node of this
// directed edge.
void DirectedEdgeBuilder::set_opp_index(const uint32_t opp_index) {
  attributes_.opp_index = opp_index;
}

// Sets the type of cycle lane (if any) present on this edge.
void DirectedEdgeBuilder::set_cyclelane(const CycleLane cyclelane) {
  attributes_.cycle_lane = static_cast<uint8_t>(cyclelane);
}

// Sets the bike network mask indicating which (if any) bicycle networks are
// along this edge. See baldr/directededge.h for definitions.
void DirectedEdgeBuilder::set_bikenetwork(const uint32_t bikenetwork) {
  if (bikenetwork > kMaxBicycleNetwork) {
    LOG_ERROR("Bicycle Network mask exceeds maximum: " +
              std::to_string(bikenetwork));
    attributes_.bikenetwork = 0;
  } else {
    attributes_.bikenetwork = bikenetwork;
  }
}

// Sets the number of lanes
void DirectedEdgeBuilder::set_lanecount(const uint32_t lanecount) {
  // TOTO - make sure we don't exceed some max value
  if (lanecount > kMaxLaneCount) {
    LOG_ERROR("Exceeding maximum lane count: " + std::to_string(lanecount));
    attributes_.lanecount = kMaxLaneCount;
  } else {
    attributes_.lanecount = lanecount;
  }
}

// Set simple turn restrictions from the end of this directed edge.
// These are turn restrictions from one edge to another that apply to
// all vehicles, at all times.
void DirectedEdgeBuilder::set_restrictions(const uint32_t mask) {
  if (mask >= (1 << kMaxTurnRestrictionEdges)) {
    LOG_ERROR("Restrictions mask exceeds allowable limit: " +
                std::to_string(mask));
    attributes_.restrictions &= ((1 << kMaxTurnRestrictionEdges) - 1);
  } else {
    attributes_.restrictions = mask;
  }
}

// Sets the specialized use type of this edge.
void DirectedEdgeBuilder::set_use(const Use use) {
  attributes_.use = static_cast<uint8_t>(use);
}

// Set the speed type (see graphconstants.h)
void DirectedEdgeBuilder::set_speed_type(const SpeedType speed_type) {
  attributes_.speed_type = static_cast<uint8_t>(speed_type);
}

// Set the country crossing flag.
void DirectedEdgeBuilder::set_ctry_crossing(const bool crossing) {
  attributes_.ctry_crossing = crossing;
}

// Set all forward access modes to true (used for transition edges)
void DirectedEdgeBuilder::set_all_forward_access() {
  forwardaccess_.v = kAllAccess;
}

// Sets the car access of the edge in the specified direction.
void DirectedEdgeBuilder::set_caraccess(const bool forward, const bool car) {
  if (forward) {
    forwardaccess_.fields.car = car;
  } else {
    reverseaccess_.fields.car = car;
  }
}

// Sets the taxi access of the edge in the specified direction.
void DirectedEdgeBuilder::set_taxiaccess(const bool forward, const bool taxi) {
  if (forward) {
    forwardaccess_.fields.taxi = taxi;
  } else {
    reverseaccess_.fields.taxi = taxi;
  }
}

// Sets the truck access of the edge in the specified direction.
void DirectedEdgeBuilder::set_truckaccess(const bool forward,
                                          const bool truck) {
  if (forward) {
    forwardaccess_.fields.truck = truck;
  } else {
    reverseaccess_.fields.truck = truck;
  }
}

// Sets the pedestrian access of the edge in the specified direction.
void DirectedEdgeBuilder::set_pedestrianaccess(const bool forward,
                                               const bool pedestrian) {
  if (forward) {
    forwardaccess_.fields.pedestrian = pedestrian;
  } else {
    reverseaccess_.fields.pedestrian = pedestrian;
  }
}

// Sets the bicycle access of the edge in the specified direction.
void DirectedEdgeBuilder::set_bicycleaccess(const bool forward,
                                            const bool bicycle) {
  if (forward) {
    forwardaccess_.fields.bicycle = bicycle;
  } else {
    reverseaccess_.fields.bicycle = bicycle;
  }
}

// Sets the emergency access of the edge in the specified direction.
void DirectedEdgeBuilder::set_emergencyaccess(const bool forward,
                                              const bool emergency) {
  if (forward) {
    forwardaccess_.fields.emergency = emergency;
  } else {
    reverseaccess_.fields.emergency = emergency;
  }
}

// Sets the bus access of the edge in the specified direction.
void DirectedEdgeBuilder::set_busaccess(const bool forward,
                                        const bool bus) {
  if (forward) {
    forwardaccess_.fields.bus = bus;
  } else {
    reverseaccess_.fields.bus = bus;
  }
}

// Sets the high occupancy vehicle (HOV) access of the edge in the specified
// direction.
void DirectedEdgeBuilder::set_hovaccess(const bool forward, const bool hov) {
  if (forward) {
    forwardaccess_.fields.hov = hov;
  } else {
    reverseaccess_.fields.hov = hov;
  }
}

// Sets the speed in KPH.
void DirectedEdgeBuilder::set_speed(const uint32_t speed) {
  // TODO - protect against exceeding max speed
  if (speed > kMaxSpeed) {
    LOG_ERROR("Exceeding maximum speed: " + std::to_string(speed));
    speed_ = static_cast<unsigned char>(kMaxSpeed);
  } else {
    speed_ = static_cast<unsigned char>(speed);
  }
}

// Sets the classification (importance) of this edge.
void DirectedEdgeBuilder::set_classification(const RoadClass roadclass) {
  classification_.classification = static_cast<uint8_t>(roadclass);
}

// Sets the surface type (see baldr/graphconstants.h). This is a general
// indication of smoothness.
void DirectedEdgeBuilder::set_surface(const Surface surface) {
  classification_.surface = static_cast<uint8_t>(surface);
}

// Sets the link flag indicating the edge is part of a link or connection
// (ramp or turn channel).
void DirectedEdgeBuilder::set_link(const uint8_t link) {
  classification_.link = link;
}

// Sets the intersection internal flag.
void DirectedEdgeBuilder::set_internal(const bool internal) {
  classification_.internal = internal;
}

// Sets the turn type given the prior edge's local index
// (index of the inbound edge).
void DirectedEdgeBuilder::set_turntype(const uint32_t localidx,
                                       const Turn::Type turntype) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Exceeding max local index in set_turntype. Skipping");
  } else {
    turntypes_.turntype = OverwriteBits(turntypes_.turntype,
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
    turntypes_.edge_to_left = OverwriteBits(turntypes_.edge_to_left,
                                          left, localidx, 1);
  }
}

// Set the stop impact when transitioning from the prior edge (given
// by the local index of the corresponding inbound edge at the node).
void DirectedEdgeBuilder::set_stopimpact(const uint32_t localidx,
                                         const uint32_t stopimpact) {
  if (stopimpact > kMaxStopImpact) {
    LOG_ERROR("Exceeding maximum stop impact: " + std::to_string(stopimpact));
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

DirectedEdgeBuilder DirectedEdgeBuilder::flipped() const {
  auto other = *this;
  std::swap(other.forwardaccess_, other.reverseaccess_);
  other.attributes_.forward = !other.attributes_.forward;
  return other;
}

// Set the index of the directed edge on the local level of the graph
// hierarchy. This is used for turn restrictions so the edges can be
// identified on the different levels.
void DirectedEdgeBuilder::set_localedgeidx(const uint32_t idx) {
  if (idx > kMaxEdgesPerNode) {
    LOG_ERROR("Local Edge Index exceeds max: " + std::to_string(idx));
    hierarchy_.localedgeidx = kMaxEdgesPerNode;
  } else {
    hierarchy_.localedgeidx = idx;
  }
}

// Set the index of the opposing directed edge on the local hierarchy level
// at the end node of this directed edge. Only stored for the first 8 edges
// so it can be used for edge transition costing.
void DirectedEdgeBuilder::set_opp_local_idx(const uint32_t idx) {
  if (idx > kMaxEdgesPerNode) {
    LOG_ERROR("Exceeding max edges in opposing local index: " + std::to_string(idx));
    hierarchy_.opp_local_idx = kMaxEdgesPerNode;
  } else {
    hierarchy_.opp_local_idx = idx;
  }
}

// Set the flag for whether this edge represents a shortcut between 2 nodes.
void DirectedEdgeBuilder::set_shortcut(const uint32_t shortcut) {
  // 0 is not a valid shortcut
  if (shortcut == 0) {
    LOG_ERROR("Invalid shortcut mask = 0");
    return;
  }

  // Set the shortcut mask if within the max number of masked shortcut edges
  if (shortcut <= kMaxShortcutsFromNode) {
    hierarchy_.shortcut = (1 << (shortcut-1));
  }

  // Set the is_shortcut flag
  hierarchy_.is_shortcut = true;
}

// Set the flag for whether this edge is superseded by a shortcut edge.
void DirectedEdgeBuilder::set_superseded(const uint32_t superseded) {
  if (superseded > kMaxShortcutsFromNode) {
      LOG_ERROR("Exceeding max shortcut edges from a node: " + std::to_string(superseded));
  } else {
    hierarchy_.superseded = (1 << (superseded-1));
  }
}

// Set the flag for whether this edge represents a transition up one level
// in the hierarchy.
void DirectedEdgeBuilder::set_trans_up(const bool trans_up) {
  hierarchy_.trans_up = trans_up;
}

// Set the flag for whether this edge represents a transition down one level
// in the hierarchy.
void DirectedEdgeBuilder::set_trans_down(const bool trans_down) {
  hierarchy_.trans_down = trans_down;
}

}
}
