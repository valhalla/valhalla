#include "baldr/directededge.h"

namespace valhalla {
namespace baldr {

// Default constructor
DirectedEdge::DirectedEdge()
    : edgedataoffset_(0),
      geoattributes_{},
      forwardaccess_{},
      reverseaccess_{},
      speed_(0),
      attributes_{} {
}

// Destructor.
DirectedEdge::~DirectedEdge() {
}

// Gets the end node of this directed edge.
GraphId DirectedEdge::endnode() const {
  return endnode_;
}

// Get the offset to the common edge data.
uint32_t DirectedEdge::edgedataoffset() const {
  return edgedataoffset_;
}

// Gets the length of the edge in meters.
uint32_t DirectedEdge::length() const {
  return geoattributes_.length;
}

// Get the access modes in the forward direction (bit field).
uint8_t DirectedEdge::forwardaccess() const {
  return forwardaccess_.v;
}

// Get the access modes in the reverse direction (bit field).
uint8_t DirectedEdge::reverseaccess() const {
  return reverseaccess_.v;
}

// Gets the speed in KPH.
uint8_t DirectedEdge::speed() const {
  return speed_;
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

bool DirectedEdge::bridge() const {
  return attributes_.bridge;
}

bool DirectedEdge::roundabout() const {
  return attributes_.roundabout;
}

// Gets the lane count
uint32_t DirectedEdge::lanecount() const {
  return attributes_.lanecount;
}

// Does this edge represent a transition up one level in the hierarchy.
bool DirectedEdge::trans_up() const {
  return attributes_.trans_up;
}

// Does this edge represent a transition down one level in the hierarchy.
bool DirectedEdge::trans_down() const {
  return attributes_.trans_down;
}

// Does this edge represent a shortcut between 2 nodes?
bool DirectedEdge::shortcut() const {
 return attributes_.shortcut;
}

// Is this edge superseded by a shortcut edge?
bool DirectedEdge::superseded() const {
  return attributes_.superseded;
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

// Gets the bike network mask
uint32_t DirectedEdge::bikenetwork() const {
  return attributes_.bikenetwork;
}

// Get the road class / importance.
RoadClass DirectedEdge::importance() const {
  return static_cast<RoadClass>(classification_.importance);
}

// Is this edge a link / ramp?
bool DirectedEdge::link() const {
  return classification_.link;
}


// Get the use of this edge.
Use DirectedEdge::use() const {
  return static_cast<Use>(classification_.use);
}

}
}
