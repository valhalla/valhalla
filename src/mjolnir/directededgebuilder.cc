#include "mjolnir/directededgebuilder.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Default constructor
DirectedEdgeBuilder::DirectedEdgeBuilder()
  : DirectedEdge() {
}

// Sets the length of the edge in kilometers.
void DirectedEdgeBuilder::set_length(const float length) {
  length_ = length;
}

// Sets the end node of this directed edge.
void DirectedEdgeBuilder::set_endnode(const GraphId& endnode) {
  endnode_ = endnode;
}

// Get the offset to the common edge data.
void DirectedEdgeBuilder::set_edgedataoffset(const uint32_t offset) {
  edgedataoffset_ = offset;
}

// Sets the car access of the edge in each direction.
void DirectedEdgeBuilder::set_caraccess(const bool forward, const bool reverse, const bool car) {
  if ( forward )
    forwardaccess_.fields.car = car;

  if ( reverse )
    reverseaccess_.fields.car = car;
}

// Sets the taxi access of the edge in each direction.
void DirectedEdgeBuilder::set_taxiaccess(const bool forward, const bool reverse, const bool taxi) {
  if ( forward )
    forwardaccess_.fields.taxi = taxi;

  if ( reverse )
    reverseaccess_.fields.taxi = taxi;
}

// Sets the truck access of the edge in each direction.
void DirectedEdgeBuilder::set_truckaccess(const bool forward, const bool reverse, const bool truck) {
  if ( forward )
    forwardaccess_.fields.truck = truck;

  if ( reverse )
    reverseaccess_.fields.truck = truck;
}

// Sets the pedestrian access of the edge in each direction.
void DirectedEdgeBuilder::set_pedestrianaccess(const bool forward, const bool reverse, const bool pedestrian) {
  if ( forward )
    forwardaccess_.fields.pedestrian = pedestrian;

  if ( reverse )
    reverseaccess_.fields.pedestrian = pedestrian;
}

// Sets the bicycle access of the edge in each direction.
void DirectedEdgeBuilder::set_bicycleaccess(const bool forward, const bool reverse, const bool bicycle) {
  if ( forward )
    forwardaccess_.fields.bicycle = bicycle;

  if ( reverse )
    reverseaccess_.fields.bicycle = bicycle;
}

// Sets the emergency access of the edge in each direction.
void DirectedEdgeBuilder::set_emergencyaccess(const bool forward, const bool reverse, const bool emergency) {
  if ( forward )
    forwardaccess_.fields.emergency = emergency;

  if ( reverse )
    reverseaccess_.fields.emergency = emergency;
}

// Sets the horse access of the edge in each direction.
void DirectedEdgeBuilder::set_horseaccess(const bool forward, const bool reverse, const bool horse) {
  if ( forward )
    forwardaccess_.fields.horse = horse;

  if ( reverse )
    reverseaccess_.fields.horse = horse;
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

//Sets the unpaved flag.
void DirectedEdgeBuilder::set_unpaved(const bool unpaved) {
  attributes_.unpaved = unpaved;
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

//Sets the number of lanes
void DirectedEdgeBuilder::set_lanecount(const uint32_t lanecount) {
  attributes_.lanecount = lanecount;
}

//Sets the bike network mask
void DirectedEdgeBuilder::set_bikenetwork(const uint32_t bikenetwork) {
  attributes_.bikenetwork = bikenetwork;
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
