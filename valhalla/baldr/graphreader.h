#ifndef VALHALLA_BALDR_GRAPHREADER_H_
#define VALHALLA_BALDR_GRAPHREADER_H_

#include <unordered_map>
#include <mutex>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace baldr {

/**
 * Class that manages simple tile cache.
 * It is NOT thread-safe!
 */
class TileCache {
 public:
  /**
  * Constructor.
  * @param max_size  maximum size of the cache
  */
  TileCache(size_t max_size);

  /**
  * Destructor.
  */
  virtual ~TileCache() = default;

  /**
   * Reserves enough cache to hold (max_cache_size / tile_size) items.
   * @param tile_size appeoximate size of one tile
   */
  virtual void Reserve(size_t tile_size);

  /**
   * Checks if tile exists in the cache.
   * @param graphid  the graphid of the tile
   * @return true if tile exists in the cache
   */
  virtual bool Contains(const GraphId& graphid) const;

  /**
   * Puts a copy of a tile of into the cache.
   * @param graphid  the graphid of the tile
   * @param tile the graph tile
   * @param size size of the tile in memory
   */
  virtual const GraphTile* Put(const GraphId& graphid, const GraphTile& tile, size_t size);

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  virtual const GraphTile* Get(const GraphId& graphid) const;

  /**
   * Lets you know if the cache is too large.
   * @return true if the cache is over committed with respect to the limit
   */
  virtual bool OverCommitted() const;

  /**
   * Clears the cache.
   */
  virtual void Clear();

 protected:
  // The actual cached GraphTile objects
  std::unordered_map<GraphId, GraphTile> cache_;

  // The current cache size in bytes
  size_t cache_size_;

  // The max cache size in bytes
  size_t max_cache_size_;
};

/**
 * Tile cache synchronized using external mutex.
 * It is thread-safe.
 */
class SynchronizedTileCache : public TileCache {
 public:
  /**
  * Constructor.
  * @param max_size  maximum size of the cache
  * @param mutex reference to an external mutex
  */
  SynchronizedTileCache(std::mutex& mutex, size_t max_size);
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

 protected:
  /**
   * Puts a copy of a tile of into the cache without locking.
   * @param graphid  the graphid of the tile
   * @param tile the graph tile
   * @param size size of the tile in memory
   */
  const GraphTile* PutNoLock(const GraphId& graphid, const GraphTile& tile, size_t size);

 private:
  std::mutex& mutex_ref_;
};

/**
 * Cache that fowards copies of tiles to other instances.
 * It is thread-safe.
 */
class CopyForwardingTileCache final : public SynchronizedTileCache {
 public:
  /**
  * Constructor.
  * @param max_size  maximum size of the cache
  */
  CopyForwardingTileCache(size_t max_size);

  /**
  * Destructor.
  */
  ~CopyForwardingTileCache();

  /**
   * Puts a copy of a tile of into all caches.
   * @param graphid  the graphid of the tile
   * @param tile the graph tile
   * @param size size of the tile in memory
   */
  const GraphTile* Put(const GraphId& graphid, const GraphTile& tile, size_t size) override;

 private:
  static std::mutex mutex_;
  static std::unordered_set<CopyForwardingTileCache*> members_;
};

/**
 * Creates tile caches.
 */
class TileCacheFactory final {
  TileCacheFactory() = delete;
 public:
  /**
   * Constructs tile cache.
   * @param pt  Property tree listing the configuration for the cahce configration
   */
  static TileCache* createTileCache(const boost::property_tree::ptree& pt);
};

/**
 * Class that manages access to GraphTiles.
 * Uses TileCache to keep a cache of tiles.
 */
class GraphReader {
 public:
  /**
   * Constructor using tiles as separate files.
   * @param pt  Property tree listing the configuration for the tile hierarchy
   */
  GraphReader(const boost::property_tree::ptree& pt);

  /**
   * Test if tile exists
   * @param  graphid  GraphId of the tile to test (tile id and level).
   */
  bool DoesTileExist(const GraphId& graphid) const;
  static bool DoesTileExist(const boost::property_tree::ptree& pt, const GraphId& graphid);

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const GraphId& graphid);

  /**
   * Get a pointer to a graph tile object given a GraphId. This method also
   * supplies the current graph tile - so if the same tile is requested in
   * succession it does not have to look up the tile in the cache.
   * @param graphid  the graphid of the tile
   * @param tile the tile pointer that may already contain a graphtile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const GraphId& graphid, const GraphTile*& tile);

  /**
   * Get a pointer to a graph tile object given a PointLL and a Level
   * @param pointll  the lat,lng that the tile covers
   * @param level    the hierarchy level to use when getting the tile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const PointLL& pointll, const uint8_t level);

  /**
   * Get a pointer to a graph tile object given a PointLL and using the highest level in the hierarchy
   * @param pointll  the lat,lng that the tile covers
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const PointLL& pointll);

  /**
   * Get the tile hierarchy used in this graph reader
   * @return hierarchy
   */
  const TileHierarchy& GetTileHierarchy() const;

  /**
   * Clears the cache
   */
  void Clear();

  /**
   * Lets you know if the cache is too large
   * @return true if the cache is over committed with respect to the limit
   */
  bool OverCommitted() const;

  /**
   * Convenience method to get an opposing directed edge.
   * @param  edgeid  Graph Id of the directed edge.
   * @return  Returns the graph Id of the opposing directed edge. An
   *          invalid graph Id is returned if the opposing edge does not
   *          exist (can occur with a regional extract where adjacent tile
   *          is missing).
   */
  GraphId GetOpposingEdgeId(const GraphId& edgeid);
  GraphId GetOpposingEdgeId(const GraphId& edgeid, const GraphTile*& tile);

  /**
   * Convenience method to get an opposing directed edge.
   * @param  edgeid  Graph Id of the directed edge.
   * @return  Returns the opposing directed edge or nullptr if the
   *          opposing edge does not exist (can occur with a regional extract
   *          where the adjacent tile is missing)
   */
  const DirectedEdge* GetOpposingEdge(const GraphId& edgeid);
  const DirectedEdge* GetOpposingEdge(const GraphId& edgeid, const GraphTile*& tile);

  /**
   * Convenience method to determine if 2 directed edges are connected.
   * @param   edge1  GraphId of first directed edge.
   * @param   edge2  GraphId of second directed edge.
   * @return  Returns true if the directed edges are directly connected
   *          at a node, false if not.
   */
  bool AreEdgesConnected(const GraphId& edge1, const GraphId& edge2);

  /**
   * Gets the shortcut edge that includes the specified edge.
   * @param  edgeid  Graph Id of a directed edge.
   * @return Returns the graph Id of the shortcut directed edge that include
   *         the specified edge. Returns an invalid GraphId if the edge is not
   *         part of a shortcut.
   */
  GraphId GetShortcut(const GraphId& id);

  /**
   * Convenience method to get the relative edge density (from the
   * begin node of an edge).
   * @param   edgeid  Graph Id of the directed edge.
   * @return  Returns the relative edge density at the begin node of the edge.
   */
  uint32_t GetEdgeDensity(const GraphId& edgeid);

  /**
   * Gets back a set of available tiles
   * @return  returns the list of available tiles
   */
  std::unordered_set<GraphId> GetTileSet() const;

 protected:
  // (Tar) extract of tiles - the contents are empty if not being used
  struct tile_extract_t;
  std::shared_ptr<const tile_extract_t> tile_extract_;
  static std::shared_ptr<const GraphReader::tile_extract_t> get_extract_instance(const boost::property_tree::ptree& pt);

  // Information about where the tiles are kept
  const TileHierarchy tile_hierarchy_;

  std::unique_ptr<TileCache> cache_;
};

}
}

#endif  // VALHALLA_BALDR_GRAPHREADER_H_
