#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <boost/property_tree/ptree.hpp>

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
    // using max value to indicate invalid
    return offset < cache_indices_.size() ? cache_indices_[offset] : midgard::invalid<uint32_t>();
  }

  // The actual cached GraphTile objects
  std::vector<graph_tile_ptr> cache_;

  // Indices into the array of actual cached items
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
   * @param pt  Property tree listing the configuration for the cache configuration
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
   *          is missing). If successful the opp_tile will point to the
   *          tile containing the opp_edge
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
   *          is missing). If successful the opp_tile will point to the
   *          tile containing the opp_edge
   */
  GraphId
  GetOpposingEdgeId(const GraphId& edgeid, const DirectedEdge*& opp_edge, graph_tile_ptr& opp_tile) {
    GraphId opp_edgeid = GetOpposingEdgeId(edgeid, opp_tile);
    if (opp_edgeid)
      opp_edge = opp_tile->directededge(opp_edgeid);
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
   *          where the adjacent tile is missing). If successful the opp_tile
   *          will point to the tile containing the opp_edge
   */
  const DirectedEdge* GetOpposingEdge(const GraphId& edgeid, graph_tile_ptr& opp_tile) {
    GraphId oppedgeid = GetOpposingEdgeId(edgeid, opp_tile);
    return oppedgeid.Is_Valid() ? opp_tile->directededge(oppedgeid) : nullptr;
  }

  /**
   * Convenience method to get an opposing directed edge.
   * @param  edge    edge of the directed edge.
   * @param  tile    Reference to a pointer to a const tile.
   * @return  Returns the opposing directed edge or nullptr if the
   *          opposing edge does not exist (can occur with a regional extract
   *          where the adjacent tile is missing). If successful the opp_tile
   *             will point to the tile containing the opp_edge
   */
  const DirectedEdge* GetOpposingEdge(const DirectedEdge* edge, graph_tile_ptr& opp_tile) {
    if (GetGraphTile(edge->endnode(), opp_tile)) {
      const auto* node = opp_tile->node(edge->endnode());
      return opp_tile->directededge(node->edge_index() + edge->opp_index());
    }
    return nullptr;
  }

  /**
   * Convenience method to get an end node.
   * @param edge  the edge whose end node you want
   * @param  tile    Reference to a pointer to a const tile.
   * @return returns the end node of edge or nullptr if it couldn't. if successful the end_node_tile
   *                 will point to the tile containing the end_node of edge
   */
  const NodeInfo* GetEndNode(const DirectedEdge* edge, graph_tile_ptr& end_node_tile) {
    return GetGraphTile(edge->endnode(), end_node_tile) ? end_node_tile->node(edge->endnode())
                                                        : nullptr;
  }

  /**
   * Method to get the begin node of an edge by using its opposing edges end node
   * @param edge    the edge whose begin node you want
   * @param tile    reference to a pointer to a const tile containing the begin node
   * @return        returns GraphId of begin node of the edge (empty if couldn't find).
   *                if successful begin_node_tile will point to the tile containing the
   *                begin_node of edge
   */
  GraphId GetBeginNodeId(const DirectedEdge* edge, graph_tile_ptr& begin_node_tile) {
    // grab the end node maybe in an adjacent tile
    graph_tile_ptr maybe_other_tile = begin_node_tile;
    if (!GetGraphTile(edge->endnode(), maybe_other_tile))
      return {};
    const auto* node = maybe_other_tile->node(edge->endnode());
    // grab the opp edge also could be in this adjacent tile
    const auto* opp_edge = maybe_other_tile->directededge(node->edge_index() + edge->opp_index());
    // grab the end node of the opp_edge, it should be in the original tile
    GetGraphTile(opp_edge->endnode(), begin_node_tile); // no-op if original tile is already correct
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
   *          at a node, false if not. If successful the edge1_end_node_tile will point
   *          to the tile containing the end node of edge1
   */
  bool AreEdgesConnectedForward(const GraphId& edge1,
                                const GraphId& edge2,
                                graph_tile_ptr& edge1_end_node_tile);

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
   * @return Returns a pointer to the node information. If successful node_tile will
   *                 point to the tile containing nodeid
   */
  const NodeInfo* nodeinfo(const GraphId& nodeid, graph_tile_ptr& node_tile) {
    return GetGraphTile(nodeid, node_tile) ? node_tile->node(nodeid) : nullptr;
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
   * @return Returns a pointer to the directed edge. If successful edge_tile will point to the tile
   *                 which contains edgeid
   */
  const DirectedEdge* directededge(const GraphId& edgeid, graph_tile_ptr& edge_tile) {
    return GetGraphTile(edgeid, edge_tile) ? edge_tile->directededge(edgeid) : nullptr;
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
   *         is not available). If successful edge_tile will point to
   *         the one containing edgeid
   */
  std::pair<GraphId, GraphId> GetDirectedEdgeNodes(const GraphId& edgeid, graph_tile_ptr& edge_tile) {
    if (edge_tile && edge_tile->id().Tile_Base() == edgeid.Tile_Base()) {
      return GetDirectedEdgeNodes(edge_tile, edge_tile->directededge(edgeid));
    } else {
      edge_tile = GetGraphTile(edgeid);
      if (!edge_tile)
        return {};
      return GetDirectedEdgeNodes(edge_tile, edge_tile->directededge(edgeid));
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
   * @return  Returns the end node of the edge. If successful edge_tile will point to the one
   *          containing edgeid
   */
  GraphId edge_endnode(const GraphId& edgeid, graph_tile_ptr& edge_tile) {
    const DirectedEdge* de = directededge(edgeid, edge_tile);
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
   * @return  Returns the start node of the edge. If successful end_node_tile will point to the tile
   *                  containing the end_node of the input edge
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
   * @returns Returns the edgeinfo for the specified id. If successful edge_tile will point to the
   *                  tile containing edgeid
   */
  EdgeInfo edgeinfo(const GraphId& edgeid, graph_tile_ptr& edge_tile) {
    auto* edge = directededge(edgeid, edge_tile);
    if (edge == nullptr) {
      throw std::runtime_error("Cannot find edgeinfo for edge: " + std::to_string(edgeid));
    }
    return edge_tile->edgeinfo(edge);
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
   * Convenience method to get the timezone index from an edge. Preferably it returns
   * the start's node's timezone.
   * @param edge   GraphId of the edge to get the timezone index.
   * @param tile   Current tile. Can be changed to the tile of the edge's end node.
   * @return Returns the timezone index. A value of 0 indicates an invalid timezone.
   *         It's possible that the tile changes to the edge's end node's tile.
   */
  int GetTimezoneFromEdge(const baldr::GraphId& edge, graph_tile_ptr& tile);

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
   * @return IncidentResult. If successful edge_tile will point to the tile containing the edge_id
   */
  IncidentResult GetIncidents(const GraphId& edge_id, graph_tile_ptr& edge_tile);

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
