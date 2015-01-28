#include "mjolnir/nodeinfobuilder.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

NodeInfoBuilder::NodeInfoBuilder()
    : NodeInfo() {
}

NodeInfoBuilder::NodeInfoBuilder(const PointLL& ll, const uint32_t edge_index,
                const uint32_t edge_count, const RoadClass rc) {
  set_latlng(ll);
  set_edge_index(edge_index);
  set_edge_count(edge_count);
  set_bestrc(rc);
}


// Sets the latitude and longitude.
void NodeInfoBuilder::set_latlng(const PointLL& ll) {
  latlng_ = ll;
}

// Set the index in the node's tile of its first outbound edge.
void NodeInfoBuilder::set_edge_index(const uint32_t edge_index) {
  if (edge_index > kMaxTileEdgeCount) {
    throw std::runtime_error("NodeInfoBuilder: edge index exceeds max");
  }
  attributes_.edge_index_ = edge_index;
}

// Set the number of outbound directed edges.
void NodeInfoBuilder::set_edge_count(const uint32_t edge_count) {
  if (edge_count > kMaxEdgesPerNode) {
    // TODO - thro an error or just log it and set count to max?
    throw std::runtime_error("NodeInfoBuilder: edge count exceeds max");
  }
  attributes_.edge_count_ = edge_count;
}

void NodeInfoBuilder::set_bestrc(const RoadClass bestrc) {
   attributes_.bestrc_= static_cast<uint32_t>(bestrc);
}

}
}
