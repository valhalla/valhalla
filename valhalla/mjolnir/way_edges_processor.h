#ifndef VALHALLA_MJOLNIR_WAY_EDGES_PROCESSOR_H
#define VALHALLA_MJOLNIR_WAY_EDGES_PROCESSOR_H

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "baldr/directededge.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"

namespace valhalla {
namespace wayedges {

// Structure holding an edge Id and forward flag
struct EdgeAndDirection {
  bool forward;
  baldr::GraphId edgeid;

  EdgeAndDirection(const bool f, const baldr::GraphId& id);
};

// Process edges and collect way information
std::unordered_map<uint64_t, std::vector<EdgeAndDirection>>
collect_way_edges(baldr::GraphReader& reader);

// Write way edges to a file
void write_way_edges(const std::unordered_map<uint64_t, std::vector<EdgeAndDirection>>& ways_edges,
                     const boost::property_tree::ptree& config);

} // namespace wayedges
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_WAY_EDGES_PROCESSOR_H