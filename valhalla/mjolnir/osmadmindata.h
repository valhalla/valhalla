#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <valhalla/midgard/pointll.h>
#include <valhalla/mjolnir/uniquenames.h>

namespace valhalla {
namespace mjolnir {

// OSM Admin
struct OSMAdmin {
  // The relations id
  uint64_t id;

  // List of ways/member ids
  std::vector<uint64_t> ways;

  // Parallel list of way roles (outer==true or inner==false)
  std::vector<bool> roles;

  // Names of country or state/prov
  uint32_t name_index;
  uint32_t name_en_index;

  // ISO code
  uint32_t iso_code_index;

  // Admin level.  2 = country; 4 = state.
  uint8_t admin_level;

  // drive on right side of the road in this country?
  bool drive_on_right;

  // do we call out intersection names at intersections?
  bool allow_intersection_names;
};

/**
 * Simple container for OSM data used by admin processing.
 * Populated by the PBF admin parser and sent into valhalla_build_admins.
 */
struct OSMAdminData {
  uint64_t max_changeset_id_; // The largest/newest changeset id encountered when parsing OSM data
  size_t osm_node_count;      // Count of osm nodes
  size_t osm_way_count;       // Count of osm ways
  size_t node_count;          // Count of all nodes

  // Unique names used by admin areas
  UniqueNames name_offset_map;

  // Map of member way id to node id
  std::unordered_map<uint64_t, std::vector<uint64_t>> way_map;

  // Map of node id to shape point
  std::unordered_map<uint64_t, midgard::PointLL> shape_map;

  // Vector of admins.
  std::vector<OSMAdmin> admins;
};

} // namespace mjolnir
} // namespace valhalla