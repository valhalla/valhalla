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
void DirectedEdgeBuilder::set_edgedataoffset(const unsigned int offset) {
  edgedataoffset_ = offset;
}

// Sets the car access of the edge in each direction.
void DirectedEdgeBuilder::set_caraccess(const bool forward, const bool reverse, const bool car) {
  if ( forward )
    forwardaccess_.car_ = car;

  if ( reverse )
    reverseaccess_.car_ = car;
}

// Sets the taxi access of the edge in each direction.
void DirectedEdgeBuilder::set_taxiaccess(const bool forward, const bool reverse, const bool taxi) {
  if ( forward )
    forwardaccess_.taxi_ = taxi;

  if ( reverse )
    reverseaccess_.taxi_ = taxi;
}

// Sets the truck access of the edge in each direction.
void DirectedEdgeBuilder::set_truckaccess(const bool forward, const bool reverse, const bool truck) {
  if ( forward )
    forwardaccess_.truck_ = truck;

  if ( reverse )
    reverseaccess_.truck_ = truck;
}

// Sets the pedestrian access of the edge in each direction.
void DirectedEdgeBuilder::set_pedestrianaccess(const bool forward, const bool reverse, const bool pedestrian) {
  if ( forward )
    forwardaccess_.pedestrian_ = pedestrian;

  if ( reverse )
    reverseaccess_.pedestrian_ = pedestrian;
}

// Sets the bicycle access of the edge in each direction.
void DirectedEdgeBuilder::set_bicycleaccess(const bool forward, const bool reverse, const bool bicycle) {
  if ( forward )
    forwardaccess_.bicycle_ = bicycle;

  if ( reverse )
    reverseaccess_.bicycle_ = bicycle;
}

// Sets the emergency access of the edge in each direction.
void DirectedEdgeBuilder::set_emergencyaccess(const bool forward, const bool reverse, const bool emergency) {
  if ( forward )
    forwardaccess_.emergency_ = emergency;

  if ( reverse )
    reverseaccess_.emergency_ = emergency;
}

// Sets the horse access of the edge in each direction.
void DirectedEdgeBuilder::set_horseaccess(const bool forward, const bool reverse, const bool horse) {
  if ( forward )
    forwardaccess_.horse_ = horse;

  if ( reverse )
    reverseaccess_.horse_ = horse;
}

//Sets the ferry flag.
void DirectedEdgeBuilder::set_ferry(const bool ferry) {
  attributes_.ferry_ = ferry;
}

//Sets the rail ferry flag.
void DirectedEdgeBuilder::set_railferry(const bool railferry) {
  attributes_.railferry_ = railferry;
}

//Sets the toll flag.
void DirectedEdgeBuilder::set_toll(const bool toll) {
  attributes_.toll_ = toll;
}

//Sets the private flag.
void DirectedEdgeBuilder::set_private(const bool priv) {
  attributes_.private_ = priv;
}

//Sets the unpaved flag.
void DirectedEdgeBuilder::set_unpaved(const bool unpaved) {
  attributes_.unpaved_ = unpaved;
}

//Sets the tunnel flag.
void DirectedEdgeBuilder::set_tunnel(const bool tunnel) {
  attributes_.tunnel_ = tunnel;
}

//Sets the road class.
void DirectedEdgeBuilder::set_class(const unsigned int roadclass) {
  classification_.class_ = roadclass;
}

//Sets the link tag.
void DirectedEdgeBuilder::set_link(const unsigned int link) {
  classification_.link_ = link;
}

//Sets the use.
void DirectedEdgeBuilder::set_use(const unsigned int  use) {
  classification_.use_ = use;
}

// Sets the speed in KPH.
void DirectedEdgeBuilder::set_speed(const float speed) {
  // TODO - protect against exceeding max speed
  speed_ = static_cast<unsigned char>(speed + 0.5f);
}

}
}
