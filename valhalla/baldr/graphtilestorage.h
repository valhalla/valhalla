#ifndef VALHALLA_BALDR_GRAPHTILESTORAGE_H_
#define VALHALLA_BALDR_GRAPHTILESTORAGE_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/tilehierarchy.h>

#include <vector>
#include <unordered_set>
#include <cstdint>

namespace valhalla {
namespace baldr {

/**
 * An abstract interface for graph tile storage.
 * The class provides methods for discovering available tiles
 * and for loading individual tiles.
 */
class GraphTileStorage {
 public:

  /**
   * Destructor
   */
  virtual ~GraphTileStorage() = default;

  /**
   * Gets the list of all tile ids available given tile hierarchy.
   * @param  tile_hierarchy The tile hierachy to use.
   * @return Returns the list of all available tile ids.
   */
  virtual std::unordered_set<GraphId> FindTiles(const TileHierarchy& tile_hierarchy) const = 0;

  /**
   * Checks if the specified tile exists.
   * @param  graphid        The tile id to check.
   * @param  tile_hierarchy The tile hierachy to use.
   * @return Returns true if the tile exists and false otherwise.
   */
  virtual bool DoesTileExist(const GraphId& graphid, const TileHierarchy& tile_hierarchy) const = 0;

  /**
   * Reads the specified tile.
   * @param  graphid        The tile id to read.
   * @param  tile_hierarchy The tile hierachy to use.
   * @param  tile_data      The buffer to use for storing the raw tile data.
   * @return Returns true if the tile exists and was successfully read and false otherwise.
   */
  virtual bool ReadTile(const GraphId& graphid, const TileHierarchy& tile_hierarchy, std::vector<char>& tile_data) const = 0;

  /**
   * Reads the optional Real-Time-Speeds associated with the tile.
   * @param  graphid        The tile id to read.
   * @param  tile_hierarchy The tile hierachy to use.
   * @param  rts_data       The buffer to use for storing real-time-speed data.
   * @return Returns true if the RTS data exists for the tile and was successfully read and false otherwise.
   */
  virtual bool ReadTileRealTimeSpeeds(const GraphId& graphid, const TileHierarchy& tile_hierarchy, std::vector<uint8_t>& rts_data) const = 0;
};

}
}

#endif  // VALHALLA_BALDR_GRAPHTILESTORAGE_H_
