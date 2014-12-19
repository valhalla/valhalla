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
void NodeInfoBuilder::set_edge_index(const uint32_t edge_index) {
  attributes_.edge_index_ = edge_index;
}

// Set the number of outbound directed edges.
void NodeInfoBuilder::set_edge_count(const uint32_t edge_count) {
  attributes_.edge_count_ = edge_count;
}

void NodeInfoBuilder::set_bestrc(const uint32_t bestrc) {
   attributes_.bestrc_= bestrc;
}

}
}
