#include "mjolnir/nodeinfobuilder.h"
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

NodeInfoBuilder::NodeInfoBuilder()
    : NodeInfo() {
}

NodeInfoBuilder::NodeInfoBuilder(const std::pair<float, float>& ll,
                                 const uint32_t edge_index,
                                 const uint32_t edge_count,
                                 const RoadClass rc) {
  set_latlng(ll);
  set_edge_index(edge_index);
  set_edge_count(edge_count);
  set_bestrc(rc);
}


// Sets the latitude and longitude.
void NodeInfoBuilder::set_latlng(const std::pair<float, float>& ll) {
  latlng_ = ll;
}

// Set the index in the node's tile of its first outbound edge.
void NodeInfoBuilder::set_edge_index(const uint32_t edge_index) {
  if (edge_index > kMaxTileEdgeCount) {
    // Consider this a catastrophic error
    throw std::runtime_error("NodeInfoBuilder: edge index exceeds max");
  }
  attributes_.edge_index_ = edge_index;
}

// Set the number of outbound directed edges.
void NodeInfoBuilder::set_edge_count(const uint32_t edge_count) {
  if (edge_count > kMaxEdgesPerNode) {
    // Log an error and set count to max.
    LOG_ERROR("NodeInfoBuilder: edge count exceeds max: " +
              std::to_string(edge_count));
    attributes_.edge_count_ = kMaxEdgesPerNode;
  } else {
    attributes_.edge_count_ = edge_count;
  }
}

void NodeInfoBuilder::set_bestrc(const RoadClass bestrc) {
   attributes_.bestrc_= static_cast<uint32_t>(bestrc);
}

}
}
