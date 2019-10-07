#pragma once
#include <cstdint>

#include <valhalla/baldr/directededge.h>
#include <valhalla/loki/search.h>

namespace valhalla {
namespace loki {

struct directed_reach {
  uint32_t outbound_reach : 16;
  uint32_t inbound_reach : 16;
};

struct reach_cache {
  // TODO: this will hold islands of edges which have equal inbound/outbound reach
  // it will be similar (or the same) as the reach which is current computed in loki
  // but will only count edges which are reachable in both directions, ie not on a
  // path where direction of travel matters when it comes to edge/opp edge pairs
};

const uint8_t kInbound = 1;
const uint8_t kOutbound = 2;

directed_reach SimpleReach(const valhalla::baldr::DirectedEdge* edge,
                           uint32_t max_reach,
                           valhalla::baldr::GraphReader& reader,
                           uint8_t direction = kInbound | kOutbound,
                           const sif::EdgeFilter& edge_filter = PassThroughEdgeFilter,
                           const sif::NodeFilter& node_filter = PassThroughNodeFilter,
                           reach_cache* cache = nullptr);

directed_reach Reach(const valhalla::baldr::DirectedEdge* edge,
                     uint32_t max_reach,
                     valhalla::baldr::GraphReader& reader,
                     uint8_t direction = kInbound | kOutbound,
                     const sif::EdgeFilter& edge_filter = PassThroughEdgeFilter,
                     const sif::NodeFilter& node_filter = PassThroughNodeFilter,
                     reach_cache* cache = nullptr);

} // namespace loki
}