#ifndef VALHALLA_MJOLNIR_OSMDATA_H
#define VALHALLA_MJOLNIR_OSMDATA_H

//#include <stxxl.h>
#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>

namespace valhalla {
namespace mjolnir {

using WayVector = std::vector<OSMWay>;

enum class OSMType : uint8_t {
    kNode,
    kWay,
    kRelation
 };

/**
 * Simple container for OSM data.
 * Populated by the PBF parser and sent into GraphBuilder.
 */
struct OSMData {
  size_t intersection_count;    // Count of intersection nodes
  size_t node_count;            // Count of all nodes
  size_t edge_count;            // Estimated count of edges

  // Stores all the ways that are part of the road network
  WayVector ways;

  // Map that stores all the nodes read
  std::unordered_map<uint64_t, OSMNode> nodes;

  // Map that stores all the ref info on a node
  std::unordered_map<uint64_t, std::string> node_ref;

  // Map that stores all the exit to info on a node
  std::unordered_map<uint64_t, std::string> node_exit_to;

  // Map that stores all the name info on a node
  std::unordered_map<uint64_t, std::string> node_name;
};

}
}

#endif  // VALHALLA_MJOLNIR_OSMDATA_H
