#include "baldr/nodeinfo.h"

namespace valhalla {
namespace baldr {

// Default constructor
NodeInfo::NodeInfo() {
  latlng_.Set(0.0f, 0.0f);
}

// Destructor.
NodeInfo::~NodeInfo() {
}

// Get the latitude, longitude
const PointLL& NodeInfo::latlng() const {
  return latlng_;
}

// Get the index in this tile of the first outbound directed edge
uint32_t NodeInfo::edge_index() const {
  return attributes_.edge_index_;
}

// Get the number of outbound edges from this node.
uint32_t NodeInfo::edge_count() const {
  return attributes_.edge_count_;
}

// Get the best road class of any outbound edges.
uint32_t NodeInfo::bestrc() const {
  return attributes_.bestrc_;
}

}
}
