#pragma once
#include <cstdint>

#include <valhalla/baldr/directededge.h>
#include <valhalla/loki/search.h>

constexpr uint8_t kInbound = 1;
constexpr uint8_t kOutbound = 2;

namespace valhalla {
namespace loki {

struct directed_reach {
  uint32_t outbound : 16;
  uint32_t inbound : 16;
};

directed_reach SimpleReach(const valhalla::baldr::DirectedEdge* edge,
                           uint32_t max_reach,
                           valhalla::baldr::GraphReader& reader,
                           const sif::EdgeFilter& edge_filter,
                           const sif::NodeFilter& node_filter,
                           uint8_t direction = kInbound | kOutbound);

} // namespace loki
} // namespace valhalla