#pragma once

#include "baldr/graphid.h"
#include "baldr/graphreader.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace valhalla {
namespace mjolnir {

// Structure holding an edge Id and forward flag
struct EdgeAndDirection {
  bool forward;
  baldr::GraphId edgeid;
};

/**
 * Collects edges for ways in the graph.
 *
 * @param reader GraphReader to access graph data
 * @param filename Optional file path for additional edge information
 * @return Map of way IDs to their associated edges and directions
 */
std::unordered_map<uint64_t, std::vector<EdgeAndDirection>>
collect_way_edges(baldr::GraphReader& reader, const std::string& filename = "");

} // namespace mjolnir
} // namespace valhalla
