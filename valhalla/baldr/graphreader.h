#ifndef VALHALLA_BALDR_GRAPHREADER_H_
#define VALHALLA_BALDR_GRAPHREADER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <functional>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/tilehierarchy.h>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace baldr {

/**
 * Class that manages access to GraphTiles.
 */
class GraphReader {
public:
  virtual ~GraphReader() = default;

  using TileLoadInterrupt = std::function<void()>;
  virtual void SetInterrupt(const TileLoadInterrupt* interrupt) = 0;

  /**
   * Test if tile exists
   * @param  graphid  GraphId of the tile to test (tile id and level).
   */
  virtual bool DoesTileExist(const GraphId& graphid) const = 0;

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  virtual const GraphTile* GetGraphTile(const GraphId& graphid) = 0;

  /**
   * Get a pointer to a graph tile object given a GraphId. This method also
   * supplies the current graph tile - so if the same tile is requested in
   * succession it does not have to look up the tile in the cache.
   * @param graphid  the graphid of the tile
   * @param tile the tile pointer that may already contain a graphtile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const GraphId& graphid, const GraphTile*& tile) {
    if (!tile || tile->id() != graphid.Tile_Base()) {
      tile = GetGraphTile(graphid);
    }
    return tile;
  }

  /**
   * Get a pointer to a graph tile object given a PointLL and a Level
   * @param pointll  the lat,lng that the tile covers
   * @param level    the hierarchy level to use when getting the tile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const midgard::PointLL& pointll, const uint8_t level) {
    GraphId id = TileHierarchy::GetGraphId(pointll, level);
    return id.Is_Valid() ? GetGraphTile(id) : nullptr;
  }

  /**
   * Get a pointer to a graph tile object given a PointLL and using the highest
   * level in the hierarchy
   * @param pointll  the lat,lng that the tile covers
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const midgard::PointLL& pointll) {
    return GetGraphTile(pointll, TileHierarchy::levels().rbegin()->second.level);
  }

  /**
   * Clears the cache
   */
  virtual void Clear() = 0;

  /**
   * Tries to ensure the cache footprint below allowed maximum
   * In some cases may even remove the entire cache.
   */
  virtual void Trim() = 0;

  /**
   * Returns the maximum number of threads that can
   * use the reader concurrently without blocking
   */
  virtual size_t MaxConcurrentUsers() const = 0;

  /**
   * Lets you know if the cache is too large
   * @return true if the cache is over committed with respect to the limit
   */
  virtual bool OverCommitted() const = 0;

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
   * Convenience method to get an opposing directed edge.
   * @param  edge    edge of the directed edge.
   * @param  tile    Reference to a pointer to a const tile.
   * @return  Returns the opposing directed edge or nullptr if the
   *          opposing edge does not exist (can occur with a regional extract
   *          where the adjacent tile is missing)
   */
  const DirectedEdge* GetOpposingEdge(const DirectedEdge* edge, const GraphTile*& tile) {
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
  bool AreEdgesConnectedForward(const GraphId& edge1, const GraphId& edge2, const GraphTile*& tile);

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
  const NodeInfo* nodeinfo(const GraphId& nodeid, const GraphTile*& tile) {
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
  const DirectedEdge* directededge(const GraphId& edgeid, const GraphTile*& tile) {
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
  std::pair<GraphId, GraphId> GetDirectedEdgeNodes(const GraphTile* tile, const DirectedEdge* edge);

  /**
   * Get the end nodes of a directed edge.
   * @param  edgeid  Directed edge Id.
   * @param  tile    Current tile.
   * @return Returns a pair of GraphIds: the first is the start node
   *         and the second is the end node. An invalid start node
   *         can occur in regional extracts (where the end node tile
   *         is not available).
   */
  std::pair<GraphId, GraphId> GetDirectedEdgeNodes(const GraphId& edgeid, const GraphTile*& tile) {
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
   * @return  Returns the start node of the edge.
   */
  GraphId edge_startnode(const GraphId& edgeid) {
    const GraphTile* NO_TILE = nullptr;
    return edge_startnode(edgeid, NO_TILE);
  }

  /**
   * Get the edgeinfo of an edge
   * @param edgeid Edge Id (Graph Id)
   * @param tile   Current tile.
   * @returns Returns the edgeinfo for the specified id.
   */
  EdgeInfo edgeinfo(const GraphId& edgeid, const GraphTile*& tile) {
    auto* edge = directededge(edgeid, tile);
    if (edge == nullptr) {
      throw std::runtime_error("Cannot find edgeinfo for edge: " + std::to_string(edgeid));
    }
    return tile->edgeinfo(edge->edgeinfo_offset());
  }

  /**
   * Get the edgeinfo of an edge
   * @param edgeid Edge Id (Graph Id)
   * @returns Returns the edgeinfo for the specified id.
   */
  EdgeInfo edgeinfo(const GraphId& edgeid) {
    const GraphTile* NO_TILE = nullptr;
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
  virtual std::unordered_set<GraphId> GetTileSet() const = 0;

  /**
   * Gets back a set of available tiles on the specified level
   * @param  level  Level to get tile set.
   * @return  returns the list of available tiles on this level
   */
  virtual std::unordered_set<GraphId> GetTileSet(const uint8_t level) const = 0;

  /**
   * Given an input bounding box, the reader will query the tile set to find the minimum
   * bounding box which entirely encloses all the edges who have begin nodes in the input
   * bounding box. If there is no data enclosed in the region the bounding box will have
   * invalid coordinates.
   *
   * @param bb   the input bounding box which is used to find begin nodes of edges
   */
  midgard::AABB2<midgard::PointLL> GetMinimumBoundingBox(const midgard::AABB2<midgard::PointLL>& bb);
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_GRAPHREADER_H_
