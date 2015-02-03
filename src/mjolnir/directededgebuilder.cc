#include "mjolnir/directededgebuilder.h"
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// TODO - validation of incoming data!

// Default constructor
DirectedEdgeBuilder::DirectedEdgeBuilder()
  : DirectedEdge() {
}

// Constructor with parameters
DirectedEdgeBuilder::DirectedEdgeBuilder(const OSMWay& way,
                   const GraphId& endnode,
                   const bool forward, const uint32_t length,
                   const float speed, const baldr::Use use,
                   const bool not_thru, const bool exit,
                   const bool internal, const RoadClass rc)
     :  DirectedEdge() {
  set_endnode(endnode);
  set_length(length);
  set_use(use);
  set_speed(speed);    // KPH
  set_ferry(way.ferry());
  set_railferry(way.rail());
  set_toll(way.toll());
  set_exit(exit);
  set_dest_only(way.destination_only());
  set_surface(way.surface());
  set_cyclelane(way.cyclelane());
  set_tunnel(way.tunnel());
  set_roundabout(way.roundabout());
  set_bridge(way.bridge());
  set_bikenetwork(way.bike_network());
  set_link(way.link());
  set_importance(rc);
  set_not_thru(not_thru);
  set_internal(internal);

  // Set forward flag and access (based on direction)
  set_forward(forward);
  set_caraccess(forward, way.auto_forward());
  set_bicycleaccess(forward, way.bike_forward());
  set_pedestrianaccess(forward, way.pedestrian());

  // Access for opposite direction
  set_caraccess(!forward, way.auto_backward());
  set_bicycleaccess(!forward, way.bike_backward());
  set_pedestrianaccess(!forward, way.pedestrian());
}

// Sets the end node of this directed edge.
void DirectedEdgeBuilder::set_endnode(const GraphId& endnode) {
  endnode_ = endnode;
}

// Get the offset to the common edge data.
void DirectedEdgeBuilder::set_edgeinfo_offset(const uint32_t offset) {
  dataoffsets_.edgeinfo_offset = offset;
}

//Sets the exit flag.
void DirectedEdgeBuilder::set_exit(const bool exit) {
  dataoffsets_.exit = exit;
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

// Sets the number of lanes
void DirectedEdgeBuilder::set_lanecount(const uint32_t lanecount) {
  geoattributes_.lanecount = lanecount;
}

// Sets the car access of the edge in each direction.
void DirectedEdgeBuilder::set_caraccess(const bool forward, const bool car) {
  if (forward) {
    forwardaccess_.fields.car = car;
  } else {
    reverseaccess_.fields.car = car;
  }
}

// Sets the taxi access of the edge in each direction.
void DirectedEdgeBuilder::set_taxiaccess(const bool forward, const bool taxi) {
  if (forward) {
    forwardaccess_.fields.taxi = taxi;
  } else {
    reverseaccess_.fields.taxi = taxi;
  }
}

// Sets the truck access of the edge in each direction.
void DirectedEdgeBuilder::set_truckaccess(const bool forward, const bool truck) {
  if (forward) {
    forwardaccess_.fields.truck = truck;
  } else {
    reverseaccess_.fields.truck = truck;
  }
}

// Sets the pedestrian access of the edge in each direction.
void DirectedEdgeBuilder::set_pedestrianaccess(const bool forward, const bool pedestrian) {
  if (forward) {
    forwardaccess_.fields.pedestrian = pedestrian;
  } else {
    reverseaccess_.fields.pedestrian = pedestrian;
  }
}

// Sets the bicycle access of the edge in each direction.
void DirectedEdgeBuilder::set_bicycleaccess(const bool forward, const bool bicycle) {
  if (forward) {
    forwardaccess_.fields.bicycle = bicycle;
  } else {
    reverseaccess_.fields.bicycle = bicycle;
  }
}

// Sets the emergency access of the edge in each direction.
void DirectedEdgeBuilder::set_emergencyaccess(const bool forward, const bool emergency) {
  if (forward) {
    forwardaccess_.fields.emergency = emergency;
  } else {
    reverseaccess_.fields.emergency = emergency;
  }
}

// Sets the horse access of the edge in each direction.
void DirectedEdgeBuilder::set_horseaccess(const bool forward, const bool horse) {
  if (forward) {
    forwardaccess_.fields.horse = horse;
  } else {
    reverseaccess_.fields.horse = horse;
  }
}

//Sets the ferry flag.
void DirectedEdgeBuilder::set_ferry(const bool ferry) {
  attributes_.ferry = ferry;
}

//Sets the rail ferry flag.
void DirectedEdgeBuilder::set_railferry(const bool railferry) {
  attributes_.railferry = railferry;
}

//Sets the toll flag.
void DirectedEdgeBuilder::set_toll(const bool toll) {
  attributes_.toll = toll;
}

//Sets the destination only (private) flag.
void DirectedEdgeBuilder::set_dest_only(const bool destonly) {
  attributes_.dest_only = destonly;
}

//Sets the tunnel flag.
void DirectedEdgeBuilder::set_tunnel(const bool tunnel) {
  attributes_.tunnel = tunnel;
}

//Sets the bridge flag.
void DirectedEdgeBuilder::set_bridge(const bool bridge) {
  attributes_.bridge = bridge;
}

//Sets the roundabout flag.
void DirectedEdgeBuilder::set_roundabout(const bool roundabout) {
  attributes_.roundabout = roundabout;
}

//Sets the surface.
void DirectedEdgeBuilder::set_surface(const Surface surface) {
  attributes_.surface = static_cast<uint8_t>(surface);
}

//Sets the cycle lane.
void DirectedEdgeBuilder::set_cyclelane(const CycleLane cyclelane) {
  attributes_.cycle_lane = static_cast<uint8_t>(cyclelane);
}

// Set the flag for whether this edge represents a transition up one level
// in the hierarchy.
void DirectedEdgeBuilder::set_trans_up(const bool trans_up) {
  attributes_.trans_up = trans_up;
}

// Set the flag for whether this edge represents a transition down one level
// in the hierarchy.
void DirectedEdgeBuilder::set_trans_down(const bool trans_down) {
  attributes_.trans_down = trans_down;
}

// Set the flag for whether this edge represents a shortcut between 2 nodes.
void DirectedEdgeBuilder::set_shortcut(const bool shortcut) {
  attributes_.shortcut = shortcut;
}

// Set the flag for whether this edge is superseded by a shortcut edge.
void DirectedEdgeBuilder::set_superseded(const bool superseded) {
  attributes_.superseded = superseded;
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

//Sets the bike network mask
void DirectedEdgeBuilder::set_bikenetwork(const uint32_t bikenetwork) {
  attributes_.bikenetwork = bikenetwork;
}

// Sets the intersection internal flag.
void DirectedEdgeBuilder::set_internal(const bool internal) {
  attributes_.internal = internal;
}

//Sets the road class.
void DirectedEdgeBuilder::set_importance(const RoadClass roadclass) {
  classification_.importance = static_cast<uint8_t>(roadclass);
}

//Sets the link tag.
void DirectedEdgeBuilder::set_link(const uint8_t link) {
  classification_.link = link;
}

//Sets the use.
void DirectedEdgeBuilder::set_use(const Use use) {
  classification_.use = static_cast<uint8_t>(use);
}

// Sets the speed in KPH.
void DirectedEdgeBuilder::set_speed(const float speed) {
  // TODO - protect against exceeding max speed
  speed_ = static_cast<unsigned char>(speed + 0.5f);
}

}
}
