#ifndef VALHALLA_BALDR_DISKGRAPHREADER_H_
#define VALHALLA_BALDR_DISKGRAPHREADER_H_

#include <mutex>
#include <unordered_map>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/curler.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/tilegetter.h>

#include <valhalla/midgard/sequence.h>

namespace valhalla {
namespace baldr {

class TileCache;

/**
 * GraphReader implementation that reads files from disk, and can optionally download them from
 * the internet. Uses TileCache to keep a cache of tiles in memory.
 */
class DiskGraphReader final : public GraphReader {
public:
  /**
   * Constructor using tiles as separate files.
   * @param pt  Property tree listing the configuration for the tile storage
   * @param tile_ getter Object responsible for getting tiles by url. If nullptr default implementation
   * is in use.
   */
  explicit DiskGraphReader(const boost::property_tree::ptree& pt,
                           std::unique_ptr<tile_getter_t>&& tile_getter = nullptr);

  ~DiskGraphReader() override;

  void SetInterrupt(const TileLoadInterrupt* interrupt) override {
    if (tile_getter_) {
      tile_getter_->set_interrupt(interrupt);
    }
  }

  /**
   * Test if tile exists
   * @param  graphid  GraphId of the tile to test (tile id and level).
   */
  bool DoesTileExist(const GraphId& graphid) const override;
  static bool DoesTileExist(const boost::property_tree::ptree& pt, const GraphId& graphid);

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const GraphId& graphid) override;

  /**
   * Clears the cache
   */
  void Clear() override;

  /**
   * Tries to ensure the cache footprint below allowed maximum
   * In some cases may even remove the entire cache.
   */
  void Trim() override;

  /**
   * Returns the maximum number of threads that can
   * use the reader concurrently without blocking
   */
  size_t MaxConcurrentUsers() const override {
    return max_concurrent_users_;
  }

  /**
   * Lets you know if the cache is too large
   * @return true if the cache is over committed with respect to the limit
   */
  bool OverCommitted() const override;

  /**
   * Gets back a set of available tiles
   * @return  returns the list of available tiles
   *          Note: this will grab all road tiles
   *          and transit tiles.
   *
   */
  std::unordered_set<GraphId> GetTileSet() const override;

  /**
   * Gets back a set of available tiles on the specified level
   * @param  level  Level to get tile set.
   * @return  returns the list of available tiles on this level
   */
  std::unordered_set<GraphId> GetTileSet(const uint8_t level) const override;

  /**
   * Returns the tile directory.
   * @return  Returns the tile directory.
   */
  std::string GetTileDir() const {
    return tile_dir_;
  }

protected:
  // (Tar) extract of tiles - the contents are empty if not being used
  struct tile_extract_t {
    tile_extract_t(const boost::property_tree::ptree& pt);
    // TODO: dont remove constness, and actually make graphtile read only?
    std::unordered_map<uint64_t, std::pair<char*, size_t>> tiles;
    std::unordered_map<uint64_t, std::pair<char*, size_t>> traffic_tiles;
    std::shared_ptr<midgard::tar> archive;
    std::shared_ptr<midgard::tar> traffic_archive;
  };
  std::shared_ptr<const tile_extract_t> tile_extract_;
  static std::shared_ptr<const DiskGraphReader::tile_extract_t>
  get_extract_instance(const boost::property_tree::ptree& pt);

  // Information about where the tiles are kept
  const std::string tile_dir_;

  // Stuff for getting at remote tiles
  std::unique_ptr<tile_getter_t> tile_getter_;
  const size_t max_concurrent_users_;
  const std::string tile_url_;

  std::mutex _404s_lock;
  std::unordered_set<GraphId> _404s;

  std::unique_ptr<TileCache> cache_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_DISKGRAPHREADER_H_
