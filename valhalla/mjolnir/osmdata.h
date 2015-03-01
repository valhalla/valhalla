#ifndef VALHALLA_MJOLNIR_OSMDATA_H
#define VALHALLA_MJOLNIR_OSMDATA_H

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <memory>

#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmway.h>
#include <valhalla/mjolnir/osmrestriction.h>
#include <valhalla/mjolnir/uniquenames.h>


namespace valhalla {
namespace mjolnir {

using RestrictionsMap = std::unordered_multimap<uint64_t, OSMRestriction>;
using OSMStringMap = std::unordered_map<uint64_t, std::string>;

enum class OSMType : uint8_t {
    kNode,
    kWay,
    kRelation
 };

struct OSMWayNodeReference {
  uint64_t node_id;
  size_t way_index;
  size_t way_shape_node_index;
  OSMNode node;
};

/**
 * Simple container for OSM data.
 * Populated by the PBF parser and sent into GraphBuilder.
 */
struct OSMData {

  // Stores all the ways that are part of the road network
  std::string ways_file;
  // Node references, contain the actual nodes associated to ways
  std::string way_node_references_file;

  size_t osm_node_count;        // Count of osm nodes
  size_t osm_way_count;         // Count of osm ways
  size_t osm_way_node_ref_count;// Count of osm nodes on osm ways
  size_t intersection_count;    // Count of intersection nodes
  size_t node_count;            // Count of all nodes
  size_t edge_count;            // Estimated count of edges

  // Stores simple restrictions. Indexed by the from way Id.
  RestrictionsMap restrictions;

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

};

}
}

#endif  // VALHALLA_MJOLNIR_OSMDATA_H
