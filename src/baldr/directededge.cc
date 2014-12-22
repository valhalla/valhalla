#include "baldr/directededge.h"

namespace valhalla {
namespace baldr {

// Default constructor
DirectedEdge::DirectedEdge()
    : speed_(0),
      length_(0),
      edgedataoffset_(0) {
}

// Gets the length of the edge in kilometers.
float DirectedEdge::length() const {
  return length_;
}

// Gets the end node of this directed edge.
GraphId DirectedEdge::endnode() const {
  return endnode_;
}

// Get the offset to the common edge data.
uint32_t DirectedEdge::edgedataoffset() const {
  return edgedataoffset_;
}

// Get the access modes in the forward direction (bit field).
uint8_t DirectedEdge::forwardaccess() const {
  return forwardaccess_.v;
}

// Get the access modes in the reverse direction (bit field).
uint8_t DirectedEdge::reverseaccess() const {
  return reverseaccess_.v;
}

// TODO - methods for individual access

bool DirectedEdge::ferry() const {
  return attributes_.ferry;
}

bool DirectedEdge::railferry() const {
  return attributes_.railferry;
}

bool DirectedEdge::toll() const {
  return attributes_.toll;
}

bool DirectedEdge::destonly() const {
  return attributes_.dest_only;
}

bool DirectedEdge::unpaved() const {
  return attributes_.unpaved;
}

bool DirectedEdge::tunnel() const {
  return attributes_.tunnel;
}

// Gets the lanes
uint32_t DirectedEdge::lanes() const {
  return attributes_.lanecount;
}

// Gets the bike network mask
uint32_t DirectedEdge::bikenetwork() const {
  return attributes_.bikenetwork;
}

// Get the road class / importance.
RoadClass DirectedEdge::importance() const {
  return static_cast<RoadClass>(classification_.importance);
}

// Get the use of this edge.
Use DirectedEdge::use() const {
  return static_cast<Use>(classification_.use);
}

// Gets the speed in KPH.
float DirectedEdge::speed() const {
  return static_cast<float>(speed_);
}

}
}
