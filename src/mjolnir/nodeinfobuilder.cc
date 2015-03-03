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
                                 const uint32_t driveable,
                                 const RoadClass rc,
                                 const uint32_t access,
                                 const NodeType type,
                                 const bool end,
                                 const bool traffic_signal) {
  set_latlng(ll);
  set_edge_index(edge_index);
  set_edge_count(edge_count);
  set_bestrc(rc);
  set_local_driveable(driveable);
  set_access(access);
  set_type(type);
  set_end(end);
  set_traffic_signal(traffic_signal);

  // Not populated yet.
  set_admin_index(0);
  set_timezone(0);
  set_dst(false);
  set_density(0);
  set_parent(false);
  set_child(false);
  set_mode_change(false);
  set_stop_id(0);
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

// Set the best road class of the outbound directed edges.
void NodeInfoBuilder::set_bestrc(const RoadClass bestrc) {
   attributes_.bestrc_ = static_cast<uint32_t>(bestrc);
}

// Set the access modes (bit mask) allowed to pass through the node.
void NodeInfoBuilder::set_access(const uint32_t access) {
  access_.v = access;
}

// Set the intersection type.
void NodeInfoBuilder::set_intersection(const IntersectionType type) {
  intersection_ = type;
}

// Set the index of the administrative information within this tile.
void NodeInfoBuilder::set_admin_index(const uint16_t admin_index) {
  admin_.admin_index = admin_index;
}

// Set the timezone index.
void NodeInfoBuilder::set_timezone(const uint16_t timezone) {
  admin_.timezone = timezone;
}

// Set the daylight saving time flag
void NodeInfoBuilder::set_dst(const bool dst) {
  admin_.dst = dst;
}

// Set the relative density
void NodeInfoBuilder::set_density(const uint32_t density) {
  if (density > kMaxDensity) {
    LOG_INFO("Exceeding max. density: " + std::to_string(density));
    type_.local_driveable = kMaxDensity;
  }
  type_.density = density;
}

// Set the node type.
void NodeInfoBuilder::set_type(const NodeType type) {
  type_.type =  static_cast<uint8_t>(type);
}

// Set the number of driveable edges on the local level.
void NodeInfoBuilder::set_local_driveable(const uint32_t n) {
  if (n > kMaxLocalDriveable) {
    LOG_INFO("Exceeding max. local driveable count: " + std::to_string(n));
    type_.local_driveable = kMaxLocalDriveable;
  }
  type_.local_driveable = n;
}

// Set the dead-end node flag.
void NodeInfoBuilder::set_end(const bool end) {
  type_.end = end;
}

// Set the parent node flag (e.g. a parent transit stop).
void NodeInfoBuilder::set_parent(const bool parent) {
  type_.parent = parent;
}

// Set the child node flag (e.g. a child transit stop).
void NodeInfoBuilder::set_child(const bool child) {
  type_.child = child;
}

// Sets the flag indicating a mode change is allowed at this node.
// The access data tells which modes are allowed at the node. Examples
// include transit stops, bike share locations, and parking locations.
void NodeInfoBuilder::set_mode_change(const bool mc) {
  type_.mode_change = mc;
}

// Set the traffic signal flag.
void NodeInfoBuilder::set_traffic_signal(const bool traffic_signal) {
  type_.traffic_signal = traffic_signal;
}

// Set the transit stop Id.
void NodeInfoBuilder::set_stop_id(const uint32_t stop_id) {
  stop_id_ = stop_id;
}

}
}
