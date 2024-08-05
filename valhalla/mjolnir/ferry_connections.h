#ifndef VALHALLA_MJOLNIR_FERRY_CONNECTIONS_H_
#define VALHALLA_MJOLNIR_FERRY_CONNECTIONS_H_

#include <cstdint>
#include <string>

#include <valhalla/mjolnir/dataquality.h>
#include <valhalla/mjolnir/node_expander.h>
#include <valhalla/mjolnir/osmdata.h>

namespace valhalla {
namespace mjolnir {

// ---------------------------------------------------------------------------
// This file contains methods to reclassify edges that connect to ferries such
// that a path from the highway hierarchy (configurable) to the ferry. This is
// needed on long routes so the hierarchy pruning doesn't cut out ferry
// connections which in OSM are often tagged as "service" (lowest
// classification).
// ---------------------------------------------------------------------------

// Unreached - not yet encountered in search
constexpr uint32_t kUnreached = 0;

// Permanent - shortest path to this edge has been found
constexpr uint32_t kPermanent = 1;

// Temporary - edge has been encountered but there could
// still be a shorter path to this node. This node will
// be "adjacent" to an node that is permanently labeled.
constexpr uint32_t kTemporary = 2;

constexpr uint32_t kFerryUpClass = static_cast<uint32_t>(baldr::RoadClass::kPrimary);

// NodeLabel - for simple shortest path
struct NodeLabel {
  float cost;
  uint32_t node_index;
  uint32_t pred_node_index;
  uint32_t way_index;
  bool dest_only;

  NodeLabel(const float c, const uint32_t n, const uint32_t p, const uint32_t w, const bool d)
      : cost(c), node_index(n), pred_node_index(p), way_index(w), dest_only(d) {
  }
};

// Store the node label status and its index in the NodeLabels list
struct NodeStatusInfo {
  uint32_t set;
  uint32_t index;
  NodeStatusInfo() : set(kUnreached), index(0) {
  }
  NodeStatusInfo(const uint32_t s, const uint32_t idx) : set(s), index(idx) {
  }
};

/**
 * Reclassify edges from a ferry along the shortest path to the
 * specified road classification.
 */
void ReclassifyFerryConnections(const std::string& ways_file,
                                const std::string& way_nodes_file,
                                const std::string& nodes_file,
                                const std::string& edges_file);

} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_FERRY_CONNECTIONS_H_
