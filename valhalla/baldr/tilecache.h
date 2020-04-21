#ifndef VALHALLA_BALDR_TILECACHE_H_
#define VALHALLA_BALDR_TILECACHE_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>

namespace valhalla {
namespace baldr {

/**
 * Tile cache interface.
 */
class TileCache {
public:
  /**
   * Destructor.
   */
  virtual ~TileCache() = default;

  /**
   * Reserves enough cache to hold (max_cache_size / tile_size) items.
   * @param tile_size appeoximate size of one tile
   */
  virtual void Reserve(size_t tile_size) = 0;

  /**
   * Checks if tile exists in the cache.
   * @param graphid  the graphid of the tile
   * @return true if tile exists in the cache
   */
  virtual bool Contains(const GraphId& graphid) const = 0;

  /**
   * Puts a copy of a tile of into the cache.
   * @param graphid  the graphid of the tile
   * @param tile the graph tile
   * @param size size of the tile in memory
   */
  virtual const GraphTile* Put(const GraphId& graphid, const GraphTile& tile, size_t size) = 0;

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  virtual const GraphTile* Get(const GraphId& graphid) const = 0;

  /**
   * Lets you know if the cache is too large.
   * @return true if the cache is over committed with respect to the limit
   */
  virtual bool OverCommitted() const = 0;

  /**
   * Clears the cache.
   */
  virtual void Clear() = 0;

  /**
   *  Does its best to reduce the cache size to remove overcommitted state.
   *  Some implementations may simply clear the entire cache
   */
  virtual void Trim() = 0;
};

/**
 * Class that manages simple tile cache.
 * It is NOT thread-safe!
 */
class SimpleTileCache : public TileCache {
public:
  /**
   * Constructor.
   * @param max_size  maximum size of the cache
   */
  SimpleTileCache(size_t max_size);

  /**
   * Reserves enough cache to hold (max_cache_size / tile_size) items.
   * @param tile_size appeoximate size of one tile
   */
  void Reserve(size_t tile_size) override;

  /**
   * Checks if tile exists in the cache.
   * @param graphid  the graphid of the tile
   * @return true if tile exists in the cache
   */
  bool Contains(const GraphId& graphid) const override;

  /**
   * Puts a copy of a tile of into the cache.
   * @param graphid  the graphid of the tile
   * @param tile the graph tile
   * @param size size of the tile in memory
   */
  const GraphTile* Put(const GraphId& graphid, const GraphTile& tile, size_t size) override;

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* Get(const GraphId& graphid) const override;

  /**
   * Lets you know if the cache is too large.
   * @return true if the cache is over committed with respect to the limit
   */
  bool OverCommitted() const override;

  /**
   * Clears the cache.
   */
  void Clear() override;

  /**
   *  Does its best to reduce the cache size to remove overcommitted state.
   *  Some implementations may simply clear the entire cache
   */
  void Trim() override;

protected:
  // The actual cached GraphTile objects
  std::unordered_map<GraphId, GraphTile> cache_;

  // The current cache size in bytes
  size_t cache_size_;

  // The max cache size in bytes
  size_t max_cache_size_;
};

/**
 * Class that manages simple tile cache and makes sure it's never overcommited.
 * The eviction policy is least
 * It is NOT thread-safe!
 */
class TileCacheLRU : public TileCache {
public:
  enum class MemoryLimitControl {
    SOFT, // no eviction is done by the cache; should be triggered by clients
    HARD, // strict memory control on every Put operation
  };

  /**
   * Constructor.
   * @param max_size     maximum size of the cache
   * @param mem_control  strategy our cache will use to control its memory
   */
  TileCacheLRU(size_t max_size, MemoryLimitControl mem_control);

  /**
   * Reserves enough cache to hold (max_cache_size / tile_size) items.
   * @param tile_size appeoximate size of one tile
   */
  void Reserve(size_t tile_size) override;

  /**
   * Checks if tile exists in the cache.
   * @param graphid  the graphid of the tile
   * @return true if tile exists in the cache
   */
  bool Contains(const GraphId& graphid) const override;

  /**
   * Puts a copy of a tile of into the cache.
   * @param graphid  the graphid of the tile
   * @param tile the graph tile
   * @param size size of the tile in memory
   */
  const GraphTile* Put(const GraphId& graphid, const GraphTile& tile, size_t tile_size) override;

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* Get(const GraphId& graphid) const override;

  /**
   * Lets you know if the cache is too large.
   * @return true if the cache is over committed with respect to the limit
   */
  bool OverCommitted() const override;

  /**
   * Clears the cache.
   */
  void Clear() override;

  /**
   *  Does its best to reduce the cache size to remove overcommitted state.
   *  Some implementations may simply clear the entire cache
   */
  void Trim() override;

protected:
  struct KeyValue {
    GraphId id;
    GraphTile tile;
  };
  using KeyValueIter = std::list<KeyValue>::iterator;

  /**
   * If needed, delete cache items until required_size in bytes is free in cache.
   * The deletion starts from the items that have been unaccessed longer than others.
   * Can potentially clean the entire cache.
   *
   * @param  required_size   size in bytes that should be free in the cache
   *
   * @return  bytes freed by the eviction
   */
  size_t TrimToFit(const size_t required_size);

  /**
   * Mark provided cache entry as most recently used.
   *
   * @param entry_iter   list entry inside LRU list
   */
  void MoveToLruHead(const KeyValueIter& entry_iter) const;

  // The GraphId -> Iterator into the linked list which owns the cached objects
  std::unordered_map<GraphId, KeyValueIter> cache_;

  // Linked list of <GraphId, Tile> pairs.
  // The most recently used item is at the beginning and the least one - at the back.
  mutable std::list<KeyValue> key_val_lru_list_;

  // Determines how we deal with
  MemoryLimitControl mem_control_;

  // The current cache size in bytes
  size_t cache_size_;

  // The max cache size in bytes
  size_t max_cache_size_;
};

/**
 * TileCache wrapper synchronized using external mutex.
 * It is thread-safe.
 */
class SynchronizedTileCache : public TileCache {
public:
  /**
   * Constructor.
   * @param cache reference to an external cache
   * @param mutex reference to an external mutex
   */
  SynchronizedTileCache(TileCache& cache, std::mutex& mutex);
  /**
   * Reserves enough cache to hold (max_cache_size / tile_size) items.
   * @param tile_size appeoximate size of one tile
   */
  void Reserve(size_t tile_size) override;

  /**
   * Checks if tile exists in the cache.
   * @param graphid  the graphid of the tile
   * @return true if tile exists in the cache
   */
  bool Contains(const GraphId& graphid) const override;

  /**
   * Puts a copy of a tile of into the cache.
   * @param graphid  the graphid of the tile
   * @param tile the graph tile
   * @param size size of the tile in memory
   */
  const GraphTile* Put(const GraphId& graphid, const GraphTile& tile, size_t size) override;

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* Get(const GraphId& graphid) const override;

  /**
   * Lets you know if the cache is too large.
   * @return true if the cache is over committed with respect to the limit
   */
  bool OverCommitted() const override;

  /**
   * Clears the cache.
   */
  void Clear() override;

  /**
   *  Does its best to reduce the cache size to remove overcommitted state.
   *  Some implementations may simply clear the entire cache
   */
  void Trim() override;

private:
  TileCache& cache_;
  std::mutex& mutex_ref_;
};

/**
 * Creates tile caches.
 */
class TileCacheFactory final {
  TileCacheFactory() = delete;

public:
  /**
   * Constructs tile cache.
   * @param pt  Property tree listing the configuration for the cahce configuration
   */
  static TileCache* createTileCache(const boost::property_tree::ptree& pt);
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_TILECACHE_H_
