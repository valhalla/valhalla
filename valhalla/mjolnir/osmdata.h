#ifndef VALHALLA_MJOLNIR_OSMDATA_H
#define VALHALLA_MJOLNIR_OSMDATA_H

#include <cstdint>
#include <string>
#include <vector>
#include <map>

#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>
#include <valhalla/mjolnir/osmrestriction.h>
#include <valhalla/mjolnir/uniquenames.h>

namespace valhalla {
namespace mjolnir {

using WayVector = std::vector<OSMWay>;
using NodeRefVector = std::vector<uint64_t>;
using RestrictionsMap = std::unordered_multimap<uint64_t, OSMRestriction>;
using OSMNodeMap = std::unordered_map<uint64_t, OSMNode>;
using OSMStringMap = std::unordered_map<uint64_t, std::string>;

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

  // Stores simple restrictions. Indexed by the from way Id.
  RestrictionsMap restrictions;

  // Node references
  NodeRefVector noderefs;

  // Map that stores all the nodes read. Indexed by OSM node Id.
  std::vector<OSMNode> nodes;

//  OSMNodeMap nodes;

  // Map that stores all the ref info on a node
  OSMStringMap node_ref;

  // Map that stores all the exit_to info on a node
  OSMStringMap node_exit_to;

  // Map that stores all the name info on a node
  OSMStringMap node_name;

  // Map that stores an updated ref for a way
  OSMStringMap way_ref;

  // References
  UniqueNames ref_offset_map;

  // Names
  UniqueNames name_offset_map;

  const OSMNode* GetNode(const uint64_t osmid) const {
    OSMNode target(osmid, 0.f, 0.f);
    auto it = std::lower_bound(nodes.begin(), nodes.end(), target);
    if(it == nodes.end() || *it != target)
      return nullptr;
    return &*it;
  }

};

}
}

#endif  // VALHALLA_MJOLNIR_OSMDATA_H
