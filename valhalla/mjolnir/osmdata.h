#ifndef VALHALLA_MJOLNIR_OSMDATA_H
#define VALHALLA_MJOLNIR_OSMDATA_H

#include <stxxl.h>
#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>
#include <valhalla/mjolnir/osmrestriction.h>
#include <valhalla/mjolnir/uniquenames.h>


namespace valhalla {
namespace mjolnir {

using WayVector = std::vector<OSMWay>;
using NodeRefVector = std::vector<uint64_t>;
using RestrictionsVector = std::vector<OSMRestriction>;

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

  // Stores simple restrictions.
  RestrictionsVector restrictions;

  // Node references
  NodeRefVector noderefs;

  // Map that stores all the nodes read
  std::unordered_map<uint64_t, OSMNode> nodes;

  // Map that stores all the ref info on a node
  std::unordered_map<uint64_t, std::string> node_ref;

  // Map that stores all the exit to info on a node
  std::unordered_map<uint64_t, std::string> node_exit_to;

  // Map that stores all the name info on a node
  std::unordered_map<uint64_t, std::string> node_name;

  // Map that stores an updated ref for a way
  std::unordered_map<uint64_t, std::string> way_ref;

  // References
  UniqueNames ref_offset_map;

  // Names
  UniqueNames name_offset_map;

};

}
}

#endif  // VALHALLA_MJOLNIR_OSMDATA_H
