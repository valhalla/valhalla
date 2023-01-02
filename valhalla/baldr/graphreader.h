#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/curler.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/tilegetter.h>
#include <valhalla/baldr/tilehierarchy.h>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/sequence.h>

#include <valhalla/proto/incidents.pb.h>

namespace valhalla {
namespace baldr {

struct tile_gone_error_t : public std::runtime_error {
  explicit tile_gone_error_t(const std::string& errormessage);
  tile_gone_error_t(std::string prefix, baldr::GraphId edgeid);
};

struct IncidentResult {
  std::shared_ptr<const IncidentsTile> tile;
  // Index into the Location array
  int start_index;
  // Index into the Location array
  int end_index;
};

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
  virtual graph_tile_ptr Put(const GraphId& graphid, graph_tile_ptr tile, size_t size) = 0;

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  virtual graph_tile_ptr Get(const GraphId& graphid) const = 0;

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
 * Class that manages flat tile cache without hash lookup
 * It is NOT thread-safe!
 */
class FlatTileCache : public TileCache {
public:
  /**
   * Constructor.
   * @param max_size  maximum size of the cache
   */
  FlatTileCache(size_t max_size);

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
  graph_tile_ptr Put(const GraphId& graphid, graph_tile_ptr tile, size_t size) override;

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  graph_tile_ptr Get(const GraphId& graphid) const override;

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
  inline uint32_t get_offset(const GraphId& graphid) const {
    return graphid.level() < 4 ? index_offsets_[graphid.level()] + graphid.tileid()
                               : cache_indices_.size();
  }
  inline uint32_t get_index(const GraphId& graphid) const {
    auto offset = get_offset(graphid);
    return offset < cache_indices_.size() ? cache_indices_[offset] : -1;
  }

  // The actual cached GraphTile objects
  std::vector<graph_tile_ptr> cache_;

  // Indicies into the array of actual cached items
  std::vector<uint32_t> cache_indices_;

  // Offsets in the indices list for where a set of tile indices begin
  std::array<uint32_t, 8> index_offsets_;

  // The current cache size in bytes
  size_t cache_size_;

  // The max cache size in bytes
  size_t max_cache_size_;
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
  graph_tile_ptr Put(const GraphId& graphid, graph_tile_ptr tile, size_t size) override;

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  graph_tile_ptr Get(const GraphId& graphid) const override;

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
  std::unordered_map<uint64_t, graph_tile_ptr> cache_;

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
  graph_tile_ptr Put(const GraphId& graphid, graph_tile_ptr tile, size_t tile_size) override;

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  graph_tile_ptr Get(const GraphId& graphid) const override;

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
    KeyValue(GraphId id_, graph_tile_ptr tile_) : id(id_), tile(std::move(tile_)) {
    }
    GraphId id;
    graph_tile_ptr tile;
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
  std::unordered_map<uint64_t, KeyValueIter> cache_;

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
  graph_tile_ptr Put(const GraphId& graphid, graph_tile_ptr tile, size_t size) override;

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  graph_tile_ptr Get(const GraphId& graphid) const override;

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

/**
 * Class that manages access to GraphTiles.
 * Uses TileCache to keep a cache of tiles.
 */
class GraphReader {
public:
  /**
   * Constructor using tiles as separate files.
   * @param pt  Property tree listing the configuration for the tile storage
   * @param tile_getter Object responsible for getting tiles by url. If nullptr default implementation
   * is in use.
   * @param traffic_readonly Flag to indicate if memory-mapped traffic extract should be writeable or
   * read-only (default).
   */
  explicit GraphReader(const boost::property_tree::ptree& pt,
                       std::unique_ptr<tile_getter_t>&& tile_getter = nullptr,
                       bool traffic_readonly = true);

  virtual ~GraphReader() = default;

  virtual void SetInterrupt(const tile_getter_t::interrupt_t* interrupt) {
    if (tile_getter_) {
      tile_getter_->set_interrupt(interrupt);
    }
  }

  /**
   * Test if tile exists
   * @param  graphid  GraphId of the tile to test (tile id and level).
   */
  virtual bool DoesTileExist(const GraphId& graphid) const;

  /**
   * Test if traffic tiles exist.   *
   */
  bool HasLiveTraffic() {
    return !tile_extract_->traffic_tiles.empty();
  }

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  virtual graph_tile_ptr GetGraphTile(const GraphId& graphid);

  /**
   * Get a pointer to a graph tile object given a GraphId. This method also
   * supplies the current graph tile - so if the same tile is requested in
   * succession it does not have to look up the tile in the cache.
   * @param graphid  the graphid of the tile
   * @param tile the tile pointer that may already contain a graphtile, output value
   * @return graph_tile_ptr& reference to the tile parameter
   */
  graph_tile_ptr& GetGraphTile(const GraphId& graphid, graph_tile_ptr& tile) {
    return !tile || tile->id() != graphid.Tile_Base() ? tile = GetGraphTile(graphid) : tile;
  }

  /**
   * Get a pointer to a graph tile object given a PointLL and a Level
   * @param pointll  the lat,lng that the tile covers
   * @param level    the hierarchy level to use when getting the tile
   * @return GraphTile* a pointer to the graph tile
   */
  graph_tile_ptr GetGraphTile(const midgard::PointLL& pointll, const uint8_t level) {
    GraphId id = TileHierarchy::GetGraphId(pointll, level);
    return id.Is_Valid() ? GetGraphTile(id) : nullptr;
  }

  /**
   * Get a pointer to a graph tile object given a PointLL and using the highest
   * level in the hierarchy
   * @param pointll  the lat,lng that the tile covers
   * @return GraphTile* a pointer to the graph tile
   */
  graph_tile_ptr GetGraphTile(const midgard::PointLL& pointll) {
    return GetGraphTile(pointll, TileHierarchy::levels().back().level);
  }

  /**
   * Clears the cache
   */
  virtual void Clear() {
    cache_->Clear();
  }

  /**
   * Tries to ensure the cache footprint below allowed maximum
   * In some cases may even remove the entire cache.
   */
  virtual void Trim() {
    cache_->Trim();
  }

  /**
   * Returns the maximum number of threads that can
   * use the reader concurrently without blocking
   */
  size_t MaxConcurrentUsers() const {
    return max_concurrent_users_;
  }

  /**
   * Lets you know if the cache is too large
   * @return true if the cache is over committed with respect to the limit
   */
  virtual bool OverCommitted() const {
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
    graph_tile_ptr NO_TILE = nullptr;
    return GetOpposingEdgeId(edgeid, NO_TILE);
  }

  /**
   * Convenience method to get an opposing directed edge.
   * @param  edgeid  Graph Id of the directed edge.
   * @param  opp_tile  Reference to a pointer to a const tile where the opposing edge should be. If
   * not we modify it to be the right tile.
   * @return  Returns the graph Id of the opposing directed edge. An
   *          invalid graph Id is returned if the opposing edge does not
   *          exist (can occur with a regional extract where adjacent tile
   *          is missing).
   */
  GraphId GetOpposingEdgeId(const GraphId& edgeid, graph_tile_ptr& opp_tile);

  /**
   * Helper method to get an opposing directed edge with it's id.
   * @param  edgeid      Graph Id of the directed edge.
   * @param  opp_edge    Reference to a pointer to a const directed edge.
   * @param  tile        Reference to a pointer to a const tile.
   * @return  Returns the graph Id of the opposing directed edge. An
   *          invalid graph Id is returned if the opposing edge does not
   *          exist (can occur with a regional extract where adjacent tile
   *          is missing).
   */
  GraphId
  GetOpposingEdgeId(const GraphId& edgeid, const DirectedEdge*& opp_edge, graph_tile_ptr& tile) {
    GraphId opp_edgeid = GetOpposingEdgeId(edgeid, tile);
    if (opp_edgeid)
      opp_edge = tile->directededge(opp_edgeid);
    return opp_edgeid;
  }

  /**
   * Convenience method to get an opposing directed edge.
   * @param  edgeid  Graph Id of the directed edge.
   * @return  Returns the opposing directed edge or nullptr if the
   *          opposing edge does not exist (can occur with a regional extract
   *          where the adjacent tile is missing)
   */
  const DirectedEdge* GetOpposingEdge(const GraphId& edgeid) {
    graph_tile_ptr NO_TILE = nullptr;
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
  const DirectedEdge* GetOpposingEdge(const GraphId& edgeid, graph_tile_ptr& tile) {
    GraphId oppedgeid = GetOpposingEdgeId(edgeid, tile);
    return oppedgeid.Is_Valid() ? tile->directededge(oppedgeid) : nullptr;
  }

  /**
   * Convenience method to get an opposing directed edge.
   * @param  edge    edge of the directed edge.
   * @param  tile    Reference to a pointer to a const tile.
   * @return  Returns the opposing directed edge or nullptr if the
   *          opposing edge does not exist (can occur with a regional extract
   *          where the adjacent tile is missing)
   */
  const DirectedEdge* GetOpposingEdge(const DirectedEdge* edge, graph_tile_ptr& tile) {
    if (GetGraphTile(edge->endnode(), tile)) {
      const auto* node = tile->node(edge->endnode());
      return tile->directededge(node->edge_index() + edge->opp_index());
    }
    return nullptr;
  }

  /**
   * Convenience method to get an end node.
   * @param edge  the edge whose end node you want
   * @param  tile    Reference to a pointer to a const tile.
   * @return returns the end node of edge or nullptr if it couldn't
   */
  const NodeInfo* GetEndNode(const DirectedEdge* edge, graph_tile_ptr& tile) {
    return GetGraphTile(edge->endnode(), tile) ? tile->node(edge->endnode()) : nullptr;
  }

  /**
   * Method to get the begin node of an edge by using its opposing edges end node
   * @param edge    the edge whose begin node you want
   * @param tile    reference to a pointer to a const tile
   * @return        returns GraphId of begin node of the edge (empty if couldn't find)
   */
  GraphId GetBeginNodeId(const DirectedEdge* edge, graph_tile_ptr& tile) {
    // grab the node
    if (!GetGraphTile(edge->endnode(), tile))
      return {};
    const auto* node = tile->node(edge->endnode());
    // grab the opp edges end node
    const auto* opp_edge = tile->directededge(node->edge_index() + edge->opp_index());
    return opp_edge->endnode();
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
  bool AreEdgesConnectedForward(const GraphId& edge1, const GraphId& edge2, graph_tile_ptr& tile);

  /**
   * Convenience method to determine if 2 directed edges are connected from
   * end node of edge1 to the start node of edge2.
   * @param   edge1  GraphId of first directed edge.
   * @param   edge2  GraphId of second directed edge.
   * @return  Returns true if the directed edges are directly connected
   *          at a node, false if not.
   */
  bool AreEdgesConnectedForward(const GraphId& edge1, const GraphId& edge2) {
    graph_tile_ptr NO_TILE = nullptr;
    return AreEdgesConnectedForward(edge1, edge2, NO_TILE);
  }

  /**
   * Gets the shortcut edge that includes the specified edge.
   * @param  edgeid  Graph Id of a directed edge.
   * @return Returns the graph Id of the shortcut directed edge that include
   *         the specified edge. Returns an invalid GraphId if the edge is not
   *         part of a shortcut.
   */
  GraphId GetShortcut(const GraphId& edgeid);

  /**
   * Recovers the edges comprising a shortcut edge.
   * @param  shortcutid  Graph Id of the shortcut edge.
   * @return Returns the edgeids of the directed edges this shortcut represents.
   */
  std::vector<GraphId> RecoverShortcut(const GraphId& shortcutid);

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
  const NodeInfo* nodeinfo(const GraphId& nodeid, graph_tile_ptr& tile) {
    return GetGraphTile(nodeid, tile) ? tile->node(nodeid) : nullptr;
  }

  /**
   * Get node information for the specified node.
   * @param  nodeid  Node Id (GraphId)
   * @return Returns a pointer to the node information.
   */
  const NodeInfo* nodeinfo(const GraphId& nodeid) {
    graph_tile_ptr NO_TILE = nullptr;
    return nodeinfo(nodeid, NO_TILE);
  }

  /**
   * Get the directed edge given its GraphId.
   * @param  edgeid  Directed edge Id.
   * @param  tile    Reference to a pointer to a const tile.
   * @return Returns a pointer to the directed edge.
   */
  const DirectedEdge* directededge(const GraphId& edgeid, graph_tile_ptr& tile) {
    return GetGraphTile(edgeid, tile) ? tile->directededge(edgeid) : nullptr;
  }

  /**
   * Get the directed edge given its GraphId.
   * @param  edgeid  Directed edge Id.
   * @return Returns a pointer to the directed edge.
   */
  const DirectedEdge* directededge(const GraphId& edgeid) {
    graph_tile_ptr NO_TILE = nullptr;
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
  std::pair<GraphId, GraphId> GetDirectedEdgeNodes(graph_tile_ptr tile, const DirectedEdge* edge);

  /**
   * Get the end nodes of a directed edge.
   * @param  edgeid  Directed edge Id.
   * @param  tile    Current tile.
   * @return Returns a pair of GraphIds: the first is the start node
   *         and the second is the end node. An invalid start node
   *         can occur in regional extracts (where the end node tile
   *         is not available).
   */
  std::pair<GraphId, GraphId> GetDirectedEdgeNodes(const GraphId& edgeid, graph_tile_ptr& tile) {
    if (tile && tile->id().Tile_Base() == edgeid.Tile_Base()) {
      return GetDirectedEdgeNodes(tile, tile->directededge(edgeid));
    } else {
      tile = GetGraphTile(edgeid);
      if (!tile)
        return {};
      return GetDirectedEdgeNodes(tile, tile->directededge(edgeid));
    }
  }

  /**
   * Get the end node of an edge.
   * @param  edgeid  Edge Id.
   * @return  Returns the end node of the edge.
   */
  GraphId edge_endnode(const GraphId& edgeid) {
    graph_tile_ptr NO_TILE = nullptr;
    return edge_endnode(edgeid, NO_TILE);
  }

  /**
   * Get the end node of an edge. The current tile is accepted as an
   * argiment.
   * @param  edgeid  Edge Id.
   * @return  Returns the end node of the edge.
   */
  GraphId edge_endnode(const GraphId& edgeid, graph_tile_ptr& tile) {
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
  GraphId edge_startnode(const GraphId& edgeid, graph_tile_ptr& tile) {
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
   * @return  Returns the start node of the edge.
   */
  GraphId edge_startnode(const GraphId& edgeid) {
    graph_tile_ptr NO_TILE = nullptr;
    return edge_startnode(edgeid, NO_TILE);
  }

  /**
   * Get the edgeinfo of an edge
   * @param edgeid Edge Id (Graph Id)
   * @param tile   Current tile.
   * @returns Returns the edgeinfo for the specified id.
   */
  EdgeInfo edgeinfo(const GraphId& edgeid, graph_tile_ptr& tile) {
    auto* edge = directededge(edgeid, tile);
    if (edge == nullptr) {
      throw std::runtime_error("Cannot find edgeinfo for edge: " + std::to_string(edgeid));
    }
    return tile->edgeinfo(edge);
  }

  /**
   * Get the edgeinfo of an edge
   * @param edgeid Edge Id (Graph Id)
   * @returns Returns the edgeinfo for the specified id.
   */
  EdgeInfo edgeinfo(const GraphId& edgeid) {
    graph_tile_ptr NO_TILE = nullptr;
    return edgeinfo(edgeid, NO_TILE);
  }

  /**
   * Get the shape of an edge
   * @param edgeid
   * @return the encoded shape (string) for specified id.
   */
  std::string encoded_edge_shape(const valhalla::baldr::GraphId& edgeid);

  /**
   * Gets back a set of available tiles
   * @return  returns the list of available tiles
   *          Note: this will grab all road tiles
   *          and transit tiles.
   *
   */
  std::unordered_set<GraphId> GetTileSet() const;

  /**
   * Gets back a set of available tiles on the specified level
   * @param  level  Level to get tile set.
   * @return  returns the list of available tiles on this level
   */
  std::unordered_set<GraphId> GetTileSet(const uint8_t level) const;

  /**
   * Returns the tile directory.
   * @return  Returns the tile directory.
   */
  const std::string& tile_dir() const {
    return tile_dir_;
  }

  /**
   * Returns the location of the tile extract
   * @return  Returns the tile extract file path.
   */
  const std::string& tile_extract() const {
    static std::string empty_str;
    if (tile_extract_->tiles.empty())
      return empty_str;
    return tile_extract_->archive->tar_file;
  }

  /**
   * Returns the tilesets location whether thats a tile_dir or a tile_extract. Purely url
   * based configurations will return the url but if they have file storage they return
   * tile_dir
   */
  const std::string& GetTileSetLocation() const {
    if (!tile_extract_->tiles.empty())
      return tile_extract();
    if (!tile_dir_.empty())
      return tile_dir_;
    return tile_url_;
  }

  /**
   * Given an input bounding box, the reader will query the tile set to find the minimum
   * bounding box which entirely encloses all the edges who have begin nodes in the input
   * bounding box. If there is no data enclosed in the region the bounding box will have
   * invalid coordinates.
   *
   * @param bb   the input bounding box which is used to find begin nodes of edges
   */
  midgard::AABB2<midgard::PointLL> GetMinimumBoundingBox(const midgard::AABB2<midgard::PointLL>& bb);

  /**
   * Convenience method to get the timezone index at a node.
   * @param node   GraphId of the node to get the timezone index.
   * @param tile   Current tile.
   * @return Returns the timezone index. A value of 0 indicates an invalid timezone.
   */
  int GetTimezone(const baldr::GraphId& node, graph_tile_ptr& tile);

  /**
   * Returns an incident tile for the given tile id
   * @param tile_id  the tile id for which incidents should be returned
   * @return the incident tile for the tile id
   */
  virtual std::shared_ptr<const IncidentsTile> GetIncidentTile(const GraphId& tile_id) const;

  /**
   * Returns a vector of incidents for the given edge
   * @param edge_id   which edge you need incidents for
   * @param tile      which tile the edge lives in, is updated if not correct
   * @return IncidentResult
   */
  IncidentResult GetIncidents(const GraphId& edge_id, graph_tile_ptr& tile);

protected:
  // (Tar) extract of tiles - the contents are empty if not being used
  struct tile_extract_t {
    tile_extract_t(const boost::property_tree::ptree& pt, bool traffic_readonly = true);
    // TODO: dont remove constness, and actually make graphtile read only?
    std::unordered_map<uint64_t, std::pair<char*, size_t>> tiles;
    std::unordered_map<uint64_t, std::pair<char*, size_t>> traffic_tiles;
    std::shared_ptr<midgard::tar> archive;
    std::shared_ptr<midgard::tar> traffic_archive;
    uint64_t checksum;
  };
  std::shared_ptr<const tile_extract_t> tile_extract_;
  static std::shared_ptr<const GraphReader::tile_extract_t>
  get_extract_instance(const boost::property_tree::ptree& pt);

  struct shortcut_recovery_t {
  protected:
    /**
     * Recovers the edges comprising a shortcut edge.
     * @param reader       The GraphReader for graph data access
     * @param  shortcutid  GraphId of the shortcut edge.
     * @return Returns the GraphIds of the directed edges this shortcut represents.
     */
    std::vector<valhalla::baldr::GraphId>
    recover_shortcut(valhalla::baldr::GraphReader& reader,
                     const valhalla::baldr::GraphId& shortcut_id) const {
      using namespace valhalla::baldr;
      // grab the shortcut edge
      auto tile = reader.GetGraphTile(shortcut_id);
      assert(tile);
      const DirectedEdge* shortcut = tile->directededge(shortcut_id);

      // bail if this isnt a shortcut
      if (!shortcut->is_shortcut()) {
        return {shortcut_id};
      }

      // loop over the edges leaving its begin node and find the superseded edge
      GraphId begin_node = reader.edge_startnode(shortcut_id);
      if (!begin_node)
        return {shortcut_id};

      // loop over the edges leaving its begin node and find the superseded edge
      std::vector<GraphId> edges;
      for (const DirectedEdge& de : tile->GetDirectedEdges(begin_node.id())) {
        if (shortcut->shortcut() & de.superseded()) {
          edges.push_back(tile->header()->graphid());
          edges.back().set_id(&de - tile->directededge(0));
          break;
        }
      }

      // bail if we couldnt find it
      if (edges.empty()) {
        LOG_TRACE("Unable to recover shortcut for edgeid " + std::to_string(shortcut_id) +
                  " | no superseded edge");
        return {shortcut_id};
      }

      // seed the edge walking with the first edge
      const DirectedEdge* current_edge = tile->directededge(edges.back());
      uint32_t accumulated_length = current_edge->length();

      // walk edges until we find the same ending node as the shortcut
      while (current_edge->endnode() != shortcut->endnode()) {
        // get the node at the end of the last edge we added
        const NodeInfo* node = reader.GetEndNode(current_edge, tile);
        if (!node)
          return {shortcut_id};
        auto node_index = node - tile->node(0);

        // check the edges leaving this node to see if we can find the one that is part of the
        // shortcut
        current_edge = nullptr;
        for (const DirectedEdge& edge : tile->GetDirectedEdges(node_index)) {
          // are they the same enough that its part of the shortcut
          // NOTE: this fails in about .05% of cases where there are two candidates and its not clear
          // which edge is the right one. looking at shortcut builder its not obvious how this is
          // possible as it seems to terminate a shortcut if more than one edge pair can be
          // contracted... NOTE: because we change the speed of the edge in graph enhancer we cant use
          // speed as a reliable determining factor
          if (begin_node != edge.endnode() && !edge.is_shortcut() &&
              edge.forwardaccess() == shortcut->forwardaccess() &&
              edge.reverseaccess() == shortcut->reverseaccess() && edge.sign() == shortcut->sign() &&
              edge.use() == shortcut->use() && edge.classification() == shortcut->classification() &&
              edge.roundabout() == shortcut->roundabout() && edge.link() == shortcut->link() &&
              edge.toll() == shortcut->toll() && edge.destonly() == shortcut->destonly() &&
              edge.unpaved() == shortcut->unpaved() && edge.surface() == shortcut->surface() &&
              edge.use() != Use::kConstruction /*&& edge.speed() == shortcut->speed()*/) {
            // we are going to keep this edge
            edges.emplace_back(tile->header()->graphid());
            edges.back().set_id(&edge - tile->directededge(0));
            // and keep expanding from the end of it
            current_edge = &edge;
            begin_node = tile->header()->graphid();
            begin_node.set_id(node_index);
            accumulated_length += edge.length();
            break;
          }
        }

        // if we didnt add an edge or we went over the length we failed
        if (current_edge == nullptr || accumulated_length > shortcut->length()) {
          LOG_TRACE("Unable to recover shortcut for edgeid " + std::to_string(shortcut_id) +
                    " | accumulated_length: " + std::to_string(accumulated_length) +
                    " | shortcut_length: " + std::to_string(shortcut->length()));
          return {shortcut_id};
        }
      }

      // we somehow got to the end via a shorter path
      if (accumulated_length < shortcut->length()) {
        LOG_TRACE("Unable to recover shortcut for edgeid (accumulated_length < shortcut->length()) " +
                  std::to_string(shortcut_id) +
                  " | accumulated_length: " + std::to_string(accumulated_length) +
                  " | shortcut_length: " + std::to_string(shortcut->length()));
        return {shortcut_id};
      }

      // these edges make up this shortcut
      return edges;
    }

    /**
     * Finds the shortcut that supersedes the given edge.
     * @param reader   The GraphReader for graph data access
     * @param id       GraphId of the edge.
     * @return Returns the GraphId of the shortcut that supersedes the edge
     *         or an invalid GraphId if the edge is not part of any shortcut.
     */
    valhalla::baldr::GraphId find_shortcut(valhalla::baldr::GraphReader& reader,
                                           const valhalla::baldr::GraphId& id) const {
      using namespace valhalla::baldr;
      // Lambda to get continuing edge at a node. Skips the specified edge Id
      // transition edges, shortcut edges, and transit connections. Returns
      // nullptr if more than one edge remains or no continuing edge is found.
      auto continuing_edge = [](const graph_tile_ptr& tile, const GraphId& edgeid,
                                const NodeInfo* nodeinfo) {
        uint32_t idx = nodeinfo->edge_index();
        const DirectedEdge* continuing_edge = static_cast<const DirectedEdge*>(nullptr);
        const DirectedEdge* directededge = tile->directededge(idx);
        for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, idx++) {
          if (idx == edgeid.id() || directededge->is_shortcut() ||
              directededge->use() == Use::kTransitConnection ||
              directededge->use() == Use::kEgressConnection ||
              directededge->use() == Use::kPlatformConnection) {
            continue;
          }
          if (continuing_edge != nullptr) {
            return static_cast<const DirectedEdge*>(nullptr);
          }
          continuing_edge = directededge;
        }
        return continuing_edge;
      };

      // No shortcuts on the local level or transit level.
      if (id.level() >= TileHierarchy::levels().back().level) {
        return {};
      }

      // If this edge is a shortcut return this edge Id
      graph_tile_ptr tile = reader.GetGraphTile(id);
      const DirectedEdge* directededge = tile->directededge(id);
      if (directededge->is_shortcut()) {
        return id;
      }

      // Walk backwards along the opposing directed edge until a shortcut
      // beginning is found or to get the continuing edge until a node that starts
      // the shortcut is found or there are 2 or more other regular edges at the
      // node.
      GraphId edgeid = id;
      const NodeInfo* node = nullptr;
      const DirectedEdge* cont_de = nullptr;
      while (true) {
        // Get the continuing directed edge. Initial case is to use the opposing
        // directed edge.
        cont_de =
            (node == nullptr) ? reader.GetOpposingEdge(id) : continuing_edge(tile, edgeid, node);
        if (cont_de == nullptr) {
          return {};
        }

        // Get the end node and end node tile
        GraphId endnode = cont_de->endnode();
        if (cont_de->leaves_tile()) {
          tile = reader.GetGraphTile(endnode.Tile_Base());
        }
        node = tile->node(endnode);

        // Get the opposing edge Id and its directed edge
        uint32_t idx = node->edge_index() + cont_de->opp_index();
        edgeid = {endnode.tileid(), endnode.level(), idx};
        directededge = tile->directededge(edgeid);
        if (directededge->superseded()) {
          // Get the shortcut edge Id that supersedes this edge
          uint32_t idx = node->edge_index() + (directededge->superseded() - 1);
          return GraphId(endnode.tileid(), endnode.level(), idx);
        }
      }
      return {};
    }

    // a place to cache the recovered shortcuts
    std::unordered_map<valhalla::baldr::GraphId, std::vector<valhalla::baldr::GraphId>> shortcuts;
    // a place to cache the shortcut membership
    std::unordered_map<valhalla::baldr::GraphId, valhalla::baldr::GraphId> superseded_edges;
    // a place to keep some stats about the recovery
    size_t unrecovered;
    size_t superseded;

    bool shortcut_to_edge_cache_enabled;
    bool edge_to_shortcut_cache_enabled;

  public:
    /**
     * Constructs a shortcut cache from a graphreaders tileset by recovering all shortcuts. If the
     * graphreader passed in is null nothing is cached and revoery will happen on the fly
     * @param reader
     */
    shortcut_recovery_t(valhalla::baldr::GraphReader* reader,
                        bool shortcut_to_edge_cache,
                        bool edge_to_shortcut_cache)
        : shortcut_to_edge_cache_enabled(shortcut_to_edge_cache),
          edge_to_shortcut_cache_enabled(edge_to_shortcut_cache) {
      // do nothing if the reader is no good or both directions are disabled
      if (!reader || (!shortcut_to_edge_cache && !edge_to_shortcut_cache)) {
        LOG_INFO("Shortcut recovery cache disabled");
        return;
      }
      if (shortcut_to_edge_cache)
        LOG_INFO("Shortcut to edge recovery cache enabled");
      if (edge_to_shortcut_cache)
        LOG_INFO("Edge to shortcut recovery cache enabled");

      // completely skip the levels that dont have shortcuts
      for (const auto& level : valhalla::baldr::TileHierarchy::levels()) {
        // we dont get shortcuts on level 2 and up
        if (level.level > 1)
          continue;
        // for each tile
        for (auto tile_id : reader->GetTileSet(level.level)) {
          // cull cache if we are over allocated
          if (reader->OverCommitted())
            reader->Trim();
          // this shouldnt fail but garbled files could cause it
          auto tile = reader->GetGraphTile(tile_id);
          assert(tile);
          // for each edge in the tile
          for (const auto& edge : tile->GetDirectedEdges()) {
            // skip non-shortcuts or the shortcut is one we wont use
            if (!edge.shortcut())
              continue;
            auto shortcut_id = tile->header()->graphid();
            shortcut_id.set_id(&edge - tile->directededge(0));
            // skip already found opposing edges
            if (shortcuts.find(shortcut_id) != shortcuts.end())
              continue;
            // recover the shortcut and make a copy for opposing direction
            auto recovered = recover_shortcut(*reader, shortcut_id);
            decltype(recovered) opp_recovered = recovered;
            std::reverse_copy(recovered.cbegin(), recovered.cend(), opp_recovered.begin());
            // save some stats
            bool failed = recovered.front() == shortcut_id;
            unrecovered += failed;
            superseded += failed ? 0 : recovered.size();
            // cache it even if it failed (no point in trying the same thing twice)
            if (edge_to_shortcut_cache) {
              for (auto recovered_edge : recovered) {
                superseded_edges.emplace(recovered_edge, shortcut_id);
              }
            }
            if (shortcut_to_edge_cache) {
              shortcuts.emplace(shortcut_id, std::move(recovered));
            }

            // its cheaper to get the opposing without crawling the graph
            auto opp_tile = tile;
            auto opp_id = reader->GetOpposingEdgeId(shortcut_id, opp_tile);
            if (!opp_id.Is_Valid())
              continue; // dont store edges which arent in our tileset

            for (auto& id : opp_recovered) {
              id = reader->GetOpposingEdgeId(id, opp_tile);
              if (!id.Is_Valid()) {
                opp_recovered = {opp_id};
                break;
              }
            }
            // stats
            failed = opp_recovered.front() == opp_id;
            unrecovered += failed;
            superseded += failed ? 0 : opp_recovered.size();
            // cache it even if it failed (no point in trying the same thing twice)
            if (edge_to_shortcut_cache) {
              for (auto recovered_edge : opp_recovered) {
                superseded_edges.emplace(recovered_edge, opp_id);
              }
            }
            if (shortcut_to_edge_cache) {
              shortcuts.emplace(opp_id, std::move(opp_recovered));
            }
          }
        }
      }

      LOG_INFO(std::to_string(shortcuts.size()) + " shortcuts recovered as " +
               std::to_string(superseded) + " superseded edges. " + std::to_string(unrecovered) +
               " shortcuts could not be recovered.");
    }

    /**
     * returns the list of graphids of the edges superceded by the provided shortcut. saddly because
     * we may have to recover the shortcut on the fly we cannot return const reference here
     *
     * @param shortcut_id   the shortcuts edge id
     * @return the list of superceded edges
     */
    std::vector<valhalla::baldr::GraphId> get(const valhalla::baldr::GraphId& shortcut_id,
                                              valhalla::baldr::GraphReader& reader) const {
      // in the case that we didnt fill the cache we fallback to recovering on the fly
      if (!shortcut_to_edge_cache_enabled) {
        return recover_shortcut(reader, shortcut_id);
      }

      auto itr = shortcuts.find(shortcut_id);
      if (itr != shortcuts.cend())
        return itr->second;

      return {};
    }

    /**
     * returns the graphid of the shortcut that supersedes the provided edge.
     *
     * @param shortcut_id   the shortcuts edge id
     * @return the list of superceded edges
     */
    valhalla::baldr::GraphId get_shortcut(const valhalla::baldr::GraphId& edge_id,
                                          valhalla::baldr::GraphReader& reader) const {
      // in the case that we didnt fill the cache we fallback to recovering on the fly
      if (!edge_to_shortcut_cache_enabled) {
        return find_shortcut(reader, edge_id);
      }

      auto itr = superseded_edges.find(edge_id);
      if (itr != superseded_edges.cend())
        return itr->second;

      return {};
    }
  };
  std::shared_ptr<const shortcut_recovery_t> shortcut_recovery_;

  // Information about where the tiles are kept
  const std::string tile_dir_;

  // Stuff for getting at remote tiles
  std::unique_ptr<tile_getter_t> tile_getter_;
  const size_t max_concurrent_users_;
  const std::string tile_url_;

  std::mutex _404s_lock;
  std::unordered_set<GraphId> _404s;

  std::unique_ptr<TileCache> cache_;

  bool enable_incidents_;
};

// Given the Location relation, return the full metadata
const valhalla::IncidentsTile::Metadata&
getIncidentMetadata(const std::shared_ptr<const valhalla::IncidentsTile>& tile,
                    const valhalla::IncidentsTile::Location& incident_location);
} // namespace baldr
} // namespace valhalla
