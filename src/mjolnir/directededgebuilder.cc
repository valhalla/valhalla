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

// TODO - methods for access

// Sets the speed in KPH.
void DirectedEdgeBuilder::set_speed(const float speed) {
  // TODO - protect against exceeding max speed
  speed_ = static_cast<unsigned char>(speed + 0.5f);
}

}
}
