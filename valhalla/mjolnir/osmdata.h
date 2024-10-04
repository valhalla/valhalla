#ifndef VALHALLA_MJOLNIR_OSMDATA_H
#define VALHALLA_MJOLNIR_OSMDATA_H

#include <cstdint>
#include <string>
#include <unordered_set>

#include <valhalla/baldr/conditional_speed_limit.h>
#include <valhalla/mjolnir/osmaccessrestriction.h>
#include <valhalla/mjolnir/osmlinguistic.h>
#include <valhalla/mjolnir/osmnode.h>
#include <valhalla/mjolnir/osmnodelinguistic.h>
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
  uint32_t way_index = 0;
  uint32_t way_shape_node_index = 0;
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

// Data types used within OSMData
using RestrictionsMultiMap = std::unordered_multimap<uint64_t, OSMRestriction>;
using ViaSet = std::unordered_set<uint64_t>;
using AccessRestrictionsMultiMap = std::unordered_multimap<uint64_t, OSMAccessRestriction>;
using BikeMultiMap = std::unordered_multimap<uint64_t, OSMBike>;
using OSMLaneConnectivityMultiMap = std::unordered_multimap<uint64_t, OSMLaneConnectivity>;
using LinguisticMultiMap = std::unordered_multimap<uint64_t, OSMLinguistic>;
using ConditionalSpeedLimitsMultiMap =
    std::unordered_multimap<uint64_t, baldr::ConditionalSpeedLimit>;

// OSMString map uses the way Id as the key and the name index into UniqueNames as the value
using OSMStringMap = std::unordered_map<uint64_t, uint32_t>;

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
  void add_to_name_map(const uint64_t member_id,
                       const std::string& direction,
                       const std::string& reference,
                       const bool forward = true);

  /**
   * Cleanup temporary files.
   */
  static void cleanup_temp_files(const std::string& tile_dir);

  uint64_t max_changeset_id_;     // The largest/newest changeset id encountered when parsing OSM data
  uint64_t osm_node_count;        // Count of osm nodes
  uint64_t osm_way_count;         // Count of osm ways
  uint64_t osm_way_node_count;    // Count of osm nodes on osm ways
  uint64_t node_count;            // Count of all nodes in the graph
  uint64_t edge_count;            // Estimated count of edges in the graph
  uint64_t node_ref_count;        // Number of node with ref
  uint64_t node_name_count;       // Number of nodes with names
  uint64_t node_exit_to_count;    // Number of nodes with exit_to
  uint64_t node_linguistic_count; // Number of nodes with linguistic info

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

  // Stores the pronunciations. Indexed by the way Id.
  LinguisticMultiMap pronunciations;

  // Stores the pronunciation languages. Indexed by the way Id.
  LinguisticMultiMap langs;

  // Stores the conditional speed limits ("maxspeed:conditional" osm key). Indexed by the way Id.
  ConditionalSpeedLimitsMultiMap conditional_speeds;

  bool initialized = false;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMDATA_H
