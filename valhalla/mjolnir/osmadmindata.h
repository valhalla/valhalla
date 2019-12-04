#ifndef VALHALLA_MJOLNIR_OSMADMINDATA_H
#define VALHALLA_MJOLNIR_OSMADMINDATA_H

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <valhalla/mjolnir/osmadmin.h>
#include <valhalla/mjolnir/uniquenames.h>

namespace valhalla {
namespace mjolnir {

using OSMWayMap = std::unordered_map<uint64_t, std::list<uint64_t>>;
using OSMShapeMap = std::unordered_map<uint64_t, PointLL>;

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

  // Map used in admins to store the shape of the nodes.
  OSMShapeMap shape_map;

  // Map used in admins to store the ways.
  OSMWayMap way_map;

  // Vector of admins.
  std::vector<OSMAdmin> admins_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMADMINDATA_H
