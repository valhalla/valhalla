#include "mjolnir/nodeinfobuilder.h"
#include <valhalla/midgard/logging.h>
#include <cmath>

using namespace valhalla::baldr;

namespace {

const uint32_t ContinuityLookup[] = {0, 7, 13, 18, 22, 25, 27};

/**
 * Get the updated bit field.
 * @param dst  Data member to be updated.
 * @param src  Value to be updated.
 * @param pos  Position (pos element within the bit field).
 * @param len  Length of each element within the bit field.
 * @return  Returns an updated value for the bit field.
 */
uint32_t OverwriteBits(const uint32_t dst, const uint32_t src,
                       const uint32_t pos, const uint32_t len) {
  uint32_t shift = (pos * len);
  uint32_t mask  = (((uint32_t)1 << len) - 1) << shift;
  return (dst & ~mask) | (src << shift);
}

}

namespace valhalla {
namespace mjolnir {

NodeInfoBuilder::NodeInfoBuilder()
    : NodeInfo() {
}

NodeInfoBuilder::NodeInfoBuilder(const std::pair<float, float>& ll,
                                 const RoadClass rc,
                                 const uint32_t access,
                                 const NodeType type,
                                 const bool traffic_signal)
    : NodeInfo() {
  set_latlng(ll);
  set_bestrc(rc);
  set_access(access);
  set_type(type);
  set_traffic_signal(traffic_signal);
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
  edge_index_ = edge_index;
}

// Set the number of outbound directed edges.
void NodeInfoBuilder::set_edge_count(const uint32_t edge_count) {
  if (edge_count > kMaxEdgesPerNode) {
    // Log an error and set count to max.
    LOG_ERROR("NodeInfoBuilder: edge count exceeds max: " +
              std::to_string(edge_count));
    edge_count_ = kMaxEdgesPerNode;
  } else {
    edge_count_ = edge_count;
  }
}

// Set the best road class of the outbound directed edges.
void NodeInfoBuilder::set_bestrc(const RoadClass bestrc) {
   bestrc_ = static_cast<uint32_t>(bestrc);
}

// Set the access modes (bit mask) allowed to pass through the node.
void NodeInfoBuilder::set_access(const uint32_t access) {
  access_ = access;
}

// Set the intersection type.
void NodeInfoBuilder::set_intersection(const IntersectionType type) {
  intersection_ = static_cast<uint32_t>(type);
}

// Set the index of the administrative information within this tile.
void NodeInfoBuilder::set_admin_index(const uint16_t admin_index) {
  if (admin_index > kMaxAdminsPerTile) {
    // Log an error and set count to max.
    LOG_ERROR("NodeInfoBuilder: admin index exceeds max: " +
              std::to_string(admin_index));
    admin_index_ = kMaxAdminsPerTile;
  } else {
    admin_index_ = admin_index;
  }
}

// Set the timezone index.
void NodeInfoBuilder::set_timezone(const uint16_t timezone) {
  if (timezone > kMaxTimeZonesPerTile) {
    // Log an error and set count to max.
    LOG_ERROR("NodeInfoBuilder: timezone index exceeds max: " +
              std::to_string(timezone));
    timezone_ = kMaxTimeZonesPerTile;
  } else {
    timezone_ = timezone;
  }
}

// Set the driveability of the local directed edge given a local
// edge index.
void NodeInfoBuilder::set_local_driveability(const uint32_t localidx,
                                             const Traversability t) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Exceeding max local index on set_local_driveability - skip");
  } else {
    local_driveability_ = OverwriteBits(local_driveability_,
                 static_cast<uint32_t>(t), localidx, 2);
  }
}

// Set the relative density
void NodeInfoBuilder::set_density(const uint32_t density) {
  if (density > kMaxDensity) {
    LOG_WARN("Exceeding max. density: " + std::to_string(density));
    density_ = kMaxDensity;
  } else {
    density_ = density;
  }
}

// Set the node type.
void NodeInfoBuilder::set_type(const NodeType type) {
  type_ = static_cast<uint32_t>(type);
}

// Set the number of driveable edges on the local level. Subtract 1 so
// a value up to kMaxLocalEdgeIndex+1 can be stored.
void NodeInfoBuilder::set_local_edge_count(const uint32_t n) {
  if (n > kMaxLocalEdgeIndex+1) {
    LOG_INFO("Exceeding max. local edge count: " + std::to_string(n));
    local_edge_count_ = kMaxLocalEdgeIndex;
  } else if (n == 0) {
    LOG_ERROR("Node with 0 local edges found");
  } else {
    local_edge_count_ = n - 1;
  }
}

// Set the parent node flag (e.g. a parent transit stop).
void NodeInfoBuilder::set_parent(const bool parent) {
  parent_ = parent;
}

// Set the child node flag (e.g. a child transit stop).
void NodeInfoBuilder::set_child(const bool child) {
  child_ = child;
}

// Sets the flag indicating a mode change is allowed at this node.
// The access data tells which modes are allowed at the node. Examples
// include transit stops, bike share locations, and parking locations.
void NodeInfoBuilder::set_mode_change(const bool mc) {
  mode_change_ = mc;
}

// Set the traffic signal flag.
void NodeInfoBuilder::set_traffic_signal(const bool traffic_signal) {
  traffic_signal_ = traffic_signal;
}

// Set the transit stop Id.
void NodeInfoBuilder::set_stop_id(const uint32_t stop_id) {
  stop_.stop_id = stop_id;
}

/**
 * Set the name consistency between a pair of local edges. This is limited
 * to the first 8 local edge indexes.
 * @param  from  Local index of the from edge.
 * @param  to    Local index of the to edge.
 * @param  c     Are names consistent between the 2 edges?
 */
void NodeInfoBuilder::set_name_consistency(const uint32_t from,
                                           const uint32_t to,
                                           const bool c) {
  if (from == to) {
    return;
  } else if (from > kMaxLocalEdgeIndex || to > kMaxLocalEdgeIndex) {
    LOG_WARN("Local index exceeds max in set_name_consistency, skip");
  } else {
    if (from < to) {
      stop_.name_consistency = OverwriteBits(stop_.name_consistency, c,
                   (ContinuityLookup[from] + (to-from-1)), 1);
    } else {
      stop_.name_consistency = OverwriteBits(stop_.name_consistency, c,
                   (ContinuityLookup[to] + (from-to-1)), 1);
    }
  }
}

// Set the heading of the local edge given its local index. Supports
// up to 8 local edges. Headings are reduced to 8 bits.
void NodeInfoBuilder::set_heading(uint32_t localidx, uint32_t heading) {
  if (localidx > kMaxLocalEdgeIndex) {
    LOG_WARN("Local index exceeds max in set_heading, skip");
  } else {
    // Has to be 64 bit!
    uint64_t hdg = static_cast<uint64_t>(std::round(
        (heading % 360) * kHeadingShrinkFactor));
    headings_ |= hdg << static_cast<uint64_t>(localidx * 8);
  }
}

}
}
