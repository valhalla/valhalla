#include "mjolnir/directededgebuilder.h"

namespace valhalla {
namespace mjolnir {

// Default constructor
DirectedEdgeBuilder::DirectedEdgeBuilder()
    : speed_(0),
      length_(0) {
}

// Sets the length of the edge in kilometers.
void DirectedEdgeBuilder::length(const float length) {
  length_ = length;
}

// Sets the end node of this directed edge.
void DirectedEdgeBuilder::endnode(const GraphId& endnode) {
  endnode_ = endnode;
}

// TODO - methods for access

// Sets the speed in KPH.
void DirectedEdgeBuilder::speed(const float speed) const {
  // TODO - protect against exceeding max speed
  speed_ = static_cast<unsigned char>(speed + 0.5f);
}

}
}
