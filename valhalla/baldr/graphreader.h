#ifndef VALHALLA_BALDR_GRAPHREADER_H_
#define VALHALLA_BALDR_GRAPHREADER_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <mutex>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <boost/property_tree/ptree.hpp>

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
   * @param pt  Property tree listing the configuration for the tile storage.
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
  const GraphTile* GetGraphTile(const GraphId& graphid, const GraphTile*& tile) {
    if (!tile || tile->id() != graphid.Tile_Base())
      tile = GetGraphTile(graphid);
    return tile;
  }

  /**
   * Get a pointer to a graph tile object given a PointLL and a Level
   * @param pointll  the lat,lng that the tile covers
   * @param level    the hierarchy level to use when getting the tile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const PointLL& pointll, const uint8_t level){
    GraphId id = TileHierarchy::GetGraphId(pointll, level);
    return id.Is_Valid() ? GetGraphTile(id) : nullptr;
  }

  /**
   * Get a pointer to a graph tile object given a PointLL and using the highest
   * level in the hierarchy
   * @param pointll  the lat,lng that the tile covers
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const PointLL& pointll){
    return GetGraphTile(pointll, TileHierarchy::levels().rbegin()->second.level);
  }

  /**
   * Clears the cache
   */
  void Clear() {
    cache_->Clear();
  }

  /**
   * Lets you know if the cache is too large
   * @return true if the cache is over committed with respect to the limit
   */
  bool OverCommitted() const {
    return cache_->OverCommitted();
  }

  /**
   * Convenience method to get an opposing directed edge.
   * @param  edgeid  Graph Id of the directed edge.
   * @return  Returns the graph Id of the opposing directed edge. An
   *          invalid graph Id is returned if the opposing edge does not
   *          exist (can occur with a regional extract where adjacent tile
   *          is missing).
   */
  GraphId GetOpposingEdgeId(const GraphId& edgeid) {
    const GraphTile* NO_TILE = nullptr;
    return GetOpposingEdgeId(edgeid, NO_TILE);
  }

  /**
   * Convenience method to get an opposing directed edge.
   * @param  edgeid  Graph Id of the directed edge.
   * @param  tile    Reference to a pointer to a const tile.
   * @return  Returns the graph Id of the opposing directed edge. An
   *          invalid graph Id is returned if the opposing edge does not
   *          exist (can occur with a regional extract where adjacent tile
   *          is missing).
   */
  GraphId GetOpposingEdgeId(const GraphId& edgeid, const GraphTile*& tile);

  /**
   * Convenience method to get an opposing directed edge.
   * @param  edgeid  Graph Id of the directed edge.
   * @return  Returns the opposing directed edge or nullptr if the
   *          opposing edge does not exist (can occur with a regional extract
   *          where the adjacent tile is missing)
   */
  const DirectedEdge* GetOpposingEdge(const GraphId& edgeid) {
    const GraphTile* NO_TILE = nullptr;
    return GetOpposingEdge(edgeid, NO_TILE);
  }

  /**
   * Convenience method to get an opposing directed edge.
   * @param  edgeid  Graph Id of the directed edge.
   * @param  tile    Reference to a pointer to a const tile.
   * @return  Returns the opposing directed edge or nullptr if the
   *          opposing edge does not exist (can occur with a regional extract
   *          where the adjacent tile is missing)
   */
  const DirectedEdge* GetOpposingEdge(const GraphId& edgeid, const GraphTile*& tile) {
    GraphId oppedgeid = GetOpposingEdgeId(edgeid, tile);
    return oppedgeid.Is_Valid() ? tile->directededge(oppedgeid) : nullptr;
  }

  /**
   * Convenience method to get an end node.
   * @param edge  the edge whose end node you want
   * @param  tile    Reference to a pointer to a const tile.
   * @return returns the end node of edge or nullptr if it couldnt
   */
  const NodeInfo* GetEndNode(const DirectedEdge* edge, const GraphTile*& tile) {
    return GetGraphTile(edge->endnode(), tile) ? tile->node(edge->endnode()) : nullptr;
  }

  /**
   * Convenience method to determine if 2 directed edges are connected.
   * @param   edge1  GraphId of first directed edge.
   * @param   edge2  GraphId of second directed edge.
   * @return  Returns true if the directed edges are directly connected
   *          at a node, false if not.
   */
  bool AreEdgesConnected(const GraphId& edge1, const GraphId& edge2);

  /**
   * Convenience method to determine if 2 directed edges are connected from
   * end node of edge1 to the start node of edge2.
   * @param   edge1  GraphId of first directed edge.
   * @param   edge2  GraphId of second directed edge.
   * @param   tile    Reference to a pointer to a const tile.
   * @return  Returns true if the directed edges are directly connected
   *          at a node, false if not.
   */
  bool AreEdgesConnectedForward(const GraphId& edge1, const GraphId& edge2,
                                const GraphTile*& tile);

  /**
   * Convenience method to determine if 2 directed edges are connected from
   * end node of edge1 to the start node of edge2.
   * @param   edge1  GraphId of first directed edge.
   * @param   edge2  GraphId of second directed edge.
   * @return  Returns true if the directed edges are directly connected
   *          at a node, false if not.
   */
  bool AreEdgesConnectedForward(const GraphId& edge1, const GraphId& edge2) {
    const GraphTile* NO_TILE = nullptr;
    return AreEdgesConnectedForward(edge1, edge2, NO_TILE);
  }


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
   * Get node information for the specified node.
   * @param  nodeid  Node Id (GraphId)
   * @param  tile    Reference to a pointer to a const tile.
   * @return Returns a pointer to the node information.
   */
  const NodeInfo* nodeinfo(const GraphId& nodeid,
                const GraphTile*& tile) {
    return GetGraphTile(nodeid, tile) ? tile->node(nodeid) : nullptr;
  }

  /**
   * Get node information for the specified node.
   * @param  nodeid  Node Id (GraphId)
   * @return Returns a pointer to the node information.
   */
  const NodeInfo* nodeinfo(const GraphId& nodeid) {
    const GraphTile* NO_TILE = nullptr;
    return nodeinfo(nodeid, NO_TILE);
  }

  /**
   * Get the directed edge given its GraphId.
   * @param  edgeid  Directed edge Id.
   * @param  tile    Reference to a pointer to a const tile.
   * @return Returns a pointer to the directed edge.
   */
  const DirectedEdge* directededge(const GraphId& edgeid,
                                   const GraphTile*& tile) {
    return GetGraphTile(edgeid, tile) ? tile->directededge(edgeid) : nullptr;
  }

  /**
   * Get the directed edge given its GraphId.
   * @param  edgeid  Directed edge Id.
   * @return Returns a pointer to the directed edge.
   */
  const DirectedEdge* directededge(const GraphId& edgeid) {
    const GraphTile* NO_TILE = nullptr;
    return directededge(edgeid, NO_TILE);
  }

  /**
   * Get the end nodes of a directed edge.
   * @param  tile  Tile of the directed edge (tile of the start node).
   * @param  edge  Directed edge.
   * @return Returns a pair of GraphIds: the first is the start node
   *         and the second is the end node. An invalid start node
   *         can occur in regional extracts (where the end node tile
   *         is not available).
   */
  std::pair<GraphId, GraphId> GetDirectedEdgeNodes(const GraphTile* tile,
                       const DirectedEdge* edge);

  /**
   * Get the end nodes of a directed edge.
   * @param  edgeid  Directed edge Id.
   * @param  tile    Current tile.
   * @return Returns a pair of GraphIds: the first is the start node
   *         and the second is the end node. An invalid start node
   *         can occur in regional extracts (where the end node tile
   *         is not available).
   */
  std::pair<GraphId, GraphId> GetDirectedEdgeNodes(const GraphId& edgeid,
                 const GraphTile*& tile) {
    if (tile && tile->id().Tile_Base() == edgeid.Tile_Base()) {
      return GetDirectedEdgeNodes(tile, tile->directededge(edgeid));
    } else {
      tile = GetGraphTile(edgeid);
      return GetDirectedEdgeNodes(tile, tile->directededge(edgeid));
    }
  }

  /**
   * Get the end node of an edge.
   * @param  edgeid  Edge Id.
   * @return  Returns the end node of the edge.
   */
  GraphId edge_endnode(const GraphId& edgeid) {
    const GraphTile* NO_TILE = nullptr;
    return edge_endnode(edgeid, NO_TILE);
  }

  /**
   * Get the end node of an edge. The current tile is accepted as an
   * argiment.
   * @param  edgeid  Edge Id.
   * @return  Returns the end node of the edge.
   */
  GraphId edge_endnode(const GraphId& edgeid, const GraphTile*& tile) {
    const DirectedEdge* de = directededge(edgeid, tile);
    if (de) {
      return de->endnode();
    } else {
      return {};
    }
  }

  /**
   * Get the start node of an edge.
   * @param edgeid Edge Id (Graph Id)
   * @param tile   Current tile.
   * @return  Returns the start node of the edge.
   */
  GraphId edge_startnode(const GraphId& edgeid, const GraphTile*& tile) {
    GraphId opp_edgeid = GetOpposingEdgeId(edgeid, tile);
    if (opp_edgeid.Is_Valid()) {
      const auto de = directededge(opp_edgeid, tile);
      if (de) {
        return de->endnode();
      }
    }
    return {};
  }

  /**
   * Get the start node of an edge.
   * @param edgeid Edge Id (Graph Id)
   * @param tile   Current tile.
   * @return  Returns the start node of the edge.
   */
  GraphId edge_startnode(const GraphId& edgeid) {
    const GraphTile* NO_TILE = nullptr;
    return edge_startnode(edgeid, NO_TILE);
  }

  /**
   * Gets back a set of available tiles
   * @return  returns the list of available tiles
   *          Note: this will grab all road tiles
   *          and transit tiles.
   *
   */
  std::unordered_set<GraphId> GetTileSet() const;

  /**
   * Returns the tile directory.
   * @return  Returns the tile directory.
   */
  const std::string& tile_dir() const {
    return tile_dir_;
  }

 protected:
  // (Tar) extract of tiles - the contents are empty if not being used
  struct tile_extract_t;
  std::shared_ptr<const tile_extract_t> tile_extract_;
  static std::shared_ptr<const GraphReader::tile_extract_t> get_extract_instance(const boost::property_tree::ptree& pt);

  // Information about where the tiles are kept
  std::string tile_dir_;

  std::unique_ptr<TileCache> cache_;
};

}
}

#endif  // VALHALLA_BALDR_GRAPHREADER_H_
