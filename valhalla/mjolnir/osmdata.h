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

// OSM record type
enum class OSMType : uint8_t { kNode, kWay, kRelation };

// Structure to store OSM node information and associate it to an OSM way
struct OSMWayNode {
  OSMNode node;
  uint32_t way_index;
  uint32_t way_shape_node_index;
};

// OSM bicycle data (stored within OSMData)
struct OSMBike {
  uint8_t bike_network;
  uint32_t name_index;
  uint32_t ref_index;
};

// OSM lane connectivity (stored within OSMData)
struct OSMLaneConnectivity {
  uint32_t to_way_id;
  uint32_t from_way_id;
  uint32_t to_lanes_index;   // Index to string in UniqueNames
  uint32_t from_lanes_index; // Index to string in UniqueNames
};

// Data types used within OSMData. Note that any maps using OSM way Id as a key can be
// 32 bit (OSM nodes require 64 bits, but ways do not)
using RestrictionsMultiMap = std::unordered_multimap<uint32_t, OSMRestriction>;
using ViaSet = std::unordered_set<uint32_t>;
using AccessRestrictionsMultiMap = std::unordered_multimap<uint32_t, OSMAccessRestriction>;
using BikeMultiMap = std::unordered_multimap<uint32_t, OSMBike>;
using OSMLaneConnectivityMultiMap = std::unordered_multimap<uint32_t, OSMLaneConnectivity>;

// OSMString map uses the way Id as the key and the name index into UniqueNames as the value
using OSMStringMap = std::unordered_map<uint32_t, uint32_t>;

/**
 * Simple container for OSM data.
 * Populated by the PBF parser and sent into GraphBuilder.
 */
struct OSMData {
  /**
   * Write data to temporary files.
   * @return Returns true if successful, false if an error occurs.
   */
  bool write_to_temp_files(const std::string& tile_dir);

  /**
   * Read data from temporary files.
   * @return Returns true if successful, false if an error occurs.
   */
  bool read_from_temp_files(const std::string& tile_dir);

  /**
   * Read data from temporary unique name file.
   * @return Returns true if successful, false if an error occurs.
   */
  bool read_from_unique_names_file(const std::string& tile_dir);

  /**
   * add the direction information to the forward or reverse map for relations.
   */
  void add_to_name_map(const uint32_t member_id,
                       const std::string& direction,
                       const std::string& reference,
                       const bool forward = true);

  /**
   * Cleanup temporary files.
   */
  static void cleanup_temp_files(const std::string& tile_dir);

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

  // unordered set used to find out if a wayid is included in any vias for complex restrictions
  ViaSet via_set;

  // Stores access restrictions. Indexed by the from way Id.
  AccessRestrictionsMultiMap access_restrictions;

  // Stores bike information from the relations.  Indexed by the way Id.
  BikeMultiMap bike_relations;

  // Map that stores an updated ref for a way. This needs to remain a map, since relations
  // update many ways at a time (so we can't move this into OSMWay unless that is mapped by Id).
  OSMStringMap way_ref;

  // Map that stores an updated reverse ref for a way. This needs to remain a map, since relations
  // update many ways at a time (so we can't move this into OSMWay unless that is mapped by Id).
  OSMStringMap way_ref_rev;

  // Unique names and strings for nodes. This is separate from other names/strings so that
  // the OSMNode (and OSMWayNode) structures can be made smaller.
  UniqueNames node_names;

  // Unique names and strings (includes road names, references, turn lane strings, etc.)
  UniqueNames name_offset_map;

  // Lane connectivity, index by the to way Id
  OSMLaneConnectivityMultiMap lane_connectivity_map;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMDATA_H
