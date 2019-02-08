#ifndef VALHALLA_MJOLNIR_OSMDATA_H
#define VALHALLA_MJOLNIR_OSMDATA_H

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <valhalla/mjolnir/osmaccessrestriction.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmrestriction.h>
#include <valhalla/mjolnir/osmway.h>
#include <valhalla/mjolnir/uniquenames.h>

namespace valhalla {
namespace mjolnir {

struct OSMBike {
  uint8_t bike_network;
  size_t name_index;
  size_t ref_index;
};

struct OSMLaneConnectivity {
  uint32_t to_way_id;
  uint32_t from_way_id;
  std::string to_lanes;
  std::string from_lanes;
};

using RestrictionsMultiMap = std::unordered_multimap<uint32_t, OSMRestriction>;

using ViaSet = std::unordered_set<uint64_t>;

using AccessRestrictionsMultiMap = std::unordered_multimap<uint64_t, OSMAccessRestriction>;

using BikeMultiMap = std::unordered_multimap<uint64_t, OSMBike>;

using OSMStringMap = std::unordered_map<uint64_t, std::string>;

using OSMLaneConnectivityMultiMap = std::unordered_multimap<uint32_t, OSMLaneConnectivity>;

enum class OSMType : uint8_t { kNode, kWay, kRelation };

struct OSMWayNode {
  OSMNode node;
  size_t way_index;
  size_t way_shape_node_index;
};

/**
 * Simple container for OSM data.
 * Populated by the PBF parser and sent into GraphBuilder.
 */
struct OSMData {
  // TODO - add a method to serialize the OSMData to temporary files and another method
  // to read these temporary files back into the OSMData struct.
  // Also - probably want  a method that will clean up the temporary files.

  uint64_t max_changeset_id_; // The largest/newest changeset id encountered when parsing OSM data
  size_t osm_node_count;      // Count of osm nodes
  size_t osm_way_count;       // Count of osm ways
  size_t osm_way_node_count;  // Count of osm nodes on osm ways
  size_t intersection_count;  // Count of intersection nodes
  size_t node_count;          // Count of all nodes
  size_t edge_count;          // Estimated count of edges
  size_t node_ref_count;      // Number of node with ref
  size_t node_name_count;     // Number of nodes with names
  size_t node_exit_to_count;  // Number of nodes with exit_to

  // Stores simple restrictions. Indexed by the from way Id
  RestrictionsMultiMap restrictions;

  // unordered set used to find out if a wayid is in the vector of vias
  ViaSet via_set;

  // Stores access restrictions. Indexed by the from way Id.
  AccessRestrictionsMultiMap access_restrictions;

  // Stores bike information from the relations.  Indexed by the way Id.
  BikeMultiMap bike_relations;

  // Map that stores an updated ref for a way. This needs to remain a map, since relations
  // update many ways at a time (so we can't move this into OSMWay unless that is mapped by Id).
  OSMStringMap way_ref;

  // Unique names and string (includes road names, references, turn lane strings, exit refs, etc.)
  UniqueNames name_offset_map;

  // Lane connectivity, index by the to way Id
  OSMLaneConnectivityMultiMap lane_connectivity_map;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMDATA_H
