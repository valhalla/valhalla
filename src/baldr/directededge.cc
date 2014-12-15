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
unsigned int DirectedEdge::edgedataoffset() const {
  return edgedataoffset_;
}

// TODO - methods for access

// Gets the speed in KPH.
float DirectedEdge::speed() const {
  return static_cast<float>(speed_);
}

}
}
