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

// Set the GraphId of the first outbound edge from this node.
void NodeInfoBuilder::set_edge_id(const baldr::GraphId& edge_id) {
  edge_id_ = edge_id;
}

// Set the number of outbound directed edges.
void NodeInfoBuilder::set_edge_count(const unsigned int edge_count) {
  edge_count_ = edge_count;
}

}
}
