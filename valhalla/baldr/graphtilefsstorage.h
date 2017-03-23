#ifndef VALHALLA_BALDR_GRAPHTILEFSSTORAGE_H_
#define VALHALLA_BALDR_GRAPHTILEFSSTORAGE_H_

#include <valhalla/baldr/graphtilestorage.h>

namespace valhalla {
namespace baldr {

/**
 * A file-system based graph tile storage.
 * The following tile scheme is assumed:
 * {tile_dir}/{level}/{id1}/{id2}/.../{idN}.gph
 */
class GraphTileFsStorage : public GraphTileStorage {
 public:

  /**
   * Constructor
   * @param pt The configuration for the storage (tile_dir, tile_extract).
   */
  GraphTileFsStorage(const boost::property_tree::ptree& pt);

  /**
   * Destructor
   */
  ~GraphTileFsStorage() = default;

  /**
   * Gets the list of all tile ids available given tile hierarchy.
   * @param  tile_hierarchy The tile hierachy to use.
   * @return Returns the list of all available tile ids.
   */
  std::unordered_set<GraphId> FindTiles(const TileHierarchy& tile_hierarchy) const override;

  /**
   * Checks if the specified tile exists.
   * @param  graphid        The tile id to check.
   * @param  tile_hierarchy The tile hierachy to use.
   * @return Returns true if the tile exists and false otherwise.
   */
  bool DoesTileExist(const GraphId& tileid, const TileHierarchy& tile_hierarchy) const override;

  /**
   * Reads the specified tile.
   * @param graphid        The tile id to read.
   * @param tile_hierarchy The tile hierachy to use.
   * @param tile_data      The buffer to use for storing the raw tile data.
   * @return Returns true if the tile exists and false otherwise.
   */
  bool ReadTile(const GraphId& tileid, const TileHierarchy& tile_hierarchy, std::vector<char>& tile_data) const override;

  /**
   * Reads the optional Real-Time-Speeds associated with the tile.
   * @param  graphid        The tile id to read.
   * @param  tile_hierarchy The tile hierachy to use.
   * @param  rts_data       The buffer to use for storing real-time-speed data.
   * @return Returns true if the RTS data exists for the tile and was successfully read and false otherwise.
   */
  bool ReadTileRealTimeSpeeds(const GraphId& graphid, const TileHierarchy& tile_hierarchy, std::vector<uint8_t>& rts_data) const override;

  /**
   * Gets the tile directory.
   * @return The tile directory.
   */
  const std::string& GetTileDir() const;

  /**
   * Gets the tile Id given the full path to the file.
   * @param  fname    Filename with complete path.
   * @param  tile_dir Base tile directory.
   * @return  Returns the tile Id.
   */
  static GraphId GetTileId(const std::string& fname, const std::string& tile_dir);

  /**
   * Gets the directory like filename suffix given the graphId
   * @param  graphid  Graph Id to construct filename.
   * @param  hierarchy The tile hierarchy structure to get info about how many tiles can exist at this level
   * @return  Returns a filename including directory path as a suffix to be appended to another uri
   */
  static std::string FileSuffix(const GraphId& graphid, const TileHierarchy& tile_hierarchy);

 private:

  // (Tar) extract of tiles - the contents are empty if not being used
  struct tile_extract_t;
  std::shared_ptr<const tile_extract_t> tile_extract_;
  static std::shared_ptr<const tile_extract_t> get_extract_instance(const boost::property_tree::ptree& pt);

  std::string tile_dir_;

};

}
}

#endif  // VALHALLA_BALDR_GRAPHTILEFSSTORAGE_H_
