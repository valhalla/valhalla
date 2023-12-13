#ifndef VALHALLA_MIDGARD_TILEHIERARCHY_H
#define VALHALLA_MIDGARD_TILEHIERARCHY_H

#include <cstdint>
#include <map>
#include <string>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/tiles.h>

namespace valhalla {
namespace baldr {

/**
 * TileLevel: Defines a level in the hierarchy of the tiles. Includes:
 *          Hierarchy level.
 *          Minimum (largest value) road class in this level.
 *          Name for the level.
 *          Lat,lon tiling (in particular, tile size) of this level
 */
struct TileLevel {
  uint8_t level;
  RoadClass importance;
  std::string name;
  midgard::Tiles<midgard::PointLL> tiles;
};

/**
 * Set of static methods used to get information the hierarchy of tiles. The
 * tile hierarchy levels are static.
 */
class TileHierarchy {
public:
  /**
   * Get the set of levels in this hierarchy.
   * @return set of TileLevel objects.
   */
  static const std::vector<TileLevel>& levels();

  /**
   * Get the transit level in this hierarchy.
   * @return the transit TileLevel object.
   */
  static const TileLevel& GetTransitLevel();

  /**
   * Returns the GraphId of the requested tile based on a lat,lng and a level.
   * If the level is not supported an invalid id will be returned.
   * @param pointll  Lat,lng location within the tile.
   * @param level    Level of the requested tile.
   */
  static GraphId GetGraphId(const midgard::PointLL& pointll, const uint8_t level);

  /**
   * Returns bounding box for the given GraphId .
   * @param id corresponding GraphId.
   * @return Bounding box of the tile which corresponding GraphId stands for.
   */
  static midgard::AABB2<midgard::PointLL> GetGraphIdBoundingBox(const GraphId& id);

  /**
   * Returns all the GraphIds of the tiles which intersect the given bounding
   * box at that level.
   * @param bbox  Bounding box of tiles to find.
   * @param level Level of the tiles to return.
   */
  static std::vector<GraphId> GetGraphIds(const midgard::AABB2<midgard::PointLL>& bbox,
                                          const uint8_t level);

  /**
   * Returns all the GraphIds of the tiles which intersect the given bounding
   * box at any level.
   * @param bbox  Bounding box of tiles to find.
   */
  static std::vector<GraphId> GetGraphIds(const midgard::AABB2<midgard::PointLL>& bbox);

  /**
   * Gets the hierarchy level given the road class.
   * @param  roadclass  Road classification.
   * @return Returns the level.
   */
  static uint8_t get_level(const RoadClass roadclass);

  /**
   * Gets the maximum level supported in the hierarchy.
   * @return  Returns the max. level.
   */
  static uint8_t get_max_level();

  /**
   * Get the tiling system for a specified level.
   * @param level  Level Id.
   * @return Returns a const reference to the tiling system for this level.
   */
  static const midgard::Tiles<midgard::PointLL>& get_tiling(const uint8_t level);

  /**
   * Returns the parent (containing tile with lower tile level) of a given tile id
   * @param child_tile_id the child tiles graph id
   * @return the tile id of the parent tile for the given child tiles id or invalid if already at the
   * lowest level
   */
  static GraphId parent(const GraphId& child_tile_id);
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_MIDGARD_TILEHIERARCHY_H
