#include "mjolnir/nodeinfobuilder.h"

namespace valhalla {
namespace mjolnir {

NodeInfoBuilder::NodeInfoBuilder()
    : NodeInfo() {
}

// Sets the latitude and longitude.
void NodeInfoBuilder::set_latlng(const PointLL& ll) {
  latlng_ = ll;
}

// Set the index in the node's tile of its first outbound edge.
void NodeInfoBuilder::set_edge_index(const unsigned int edge_index) {
  edge_index_ = edge_index;
}

// Set the number of outbound directed edges.
void NodeInfoBuilder::set_edge_count(const unsigned int edge_count) {
  edge_count_ = edge_count;
}

}
}
