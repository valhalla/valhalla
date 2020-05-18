#ifndef VALHALLA_BALDR_MERGE_H_
#define VALHALLA_BALDR_MERGE_H_

#include <cstdint>
#include <deque>
#include <utility>
#include <valhalla/baldr/edgetracker.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>

namespace valhalla {
namespace baldr {
namespace merge {

// a segment is a collection of a start node, end node and the edge between
// them. this is used when creating paths.
struct segment {
  GraphId m_start, m_edge, m_end;
  segment(GraphId start, GraphId edge, GraphId end);
  GraphId start() const {
    return m_start;
  }
  GraphId end() const {
    return m_end;
  }
  GraphId edge() const {
    return m_edge;
  }
};

// a path is a start node, end node and a collection of contiguous edges between
// them. it is created by concatenating segments.
struct path {
  GraphId m_start, m_end;
  std::deque<GraphId> m_edges;

  explicit path(segment s);
  explicit path(GraphId node_id);

  void push_back(segment s);
  void push_front(segment s);
};

namespace detail {

struct edge_collapser {
  edge_collapser(GraphReader& reader,
                 edge_tracker& tracker,
                 std::function<bool(const DirectedEdge*)> edge_merge_pred,
                 std::function<bool(const DirectedEdge*)> edge_allowed_pred,
                 std::function<void(const path&)> func);
  std::pair<GraphId, GraphId> nodes_reachable_from(GraphId node_id);
  GraphId next_node_id(GraphId last_node_id, GraphId node_id);
  GraphId edge_between(GraphId cur, GraphId next);
  void explore(GraphId node_id);

  // return true if a loop is detected
  bool explore(GraphId prev, GraphId cur, path& forward, path& reverse);

private:
  GraphReader& m_reader;
  edge_tracker& m_tracker;
  std::function<bool(const DirectedEdge*)> m_edge_merge_predicate;
  std::function<bool(const DirectedEdge*)> m_edge_allowed_predicate;
  std::function<void(const path&)> m_func;
};

} // namespace detail

/**
 * Read the graph and merge all compatible edges, calling the provided function
 * with each path that has been found. Each edge in the graph will be part of
 * exactly one path, but paths may contain multiple edges.
 *
 * @param tiles A range object over GraphId for the tiles to consider.
 * @param reader The graph to traverse.
 * @param edge_merge_pred   A predicate function which should return true for
 *                          any edge that can be collapsed. This will return
 *                          false if an edge is encountered such that merging
 *                          at a node should not be performed.
 * @param edge_allowed_pred A predicate function which should return true for
 *                          any edge that can be allowed on the path.
 * @param func The function to execute for each discovered path.
 */
template <typename TileSet>
void merge(TileSet& tiles,
           GraphReader& reader,
           std::function<bool(const DirectedEdge*)> edge_merge_pred,
           const std::function<bool(const DirectedEdge*)>& edge_allowed_pred,
           const std::function<void(const path&)>& func) {
  edge_tracker tracker = edge_tracker::create(tiles, reader);
  detail::edge_collapser e(reader, tracker, std::move(edge_merge_pred), edge_allowed_pred, func);

  // Iterate over tiles. Merge edges at nodes where the edges can be collapsed.
  for (GraphId tile_id : tiles) {
    const auto* tile = reader.GetGraphTile(tile_id);
    uint32_t node_count = tile->header()->nodecount();
    GraphId node_id(tile_id.tileid(), tile_id.level(), 0);
    for (uint32_t i = 0; i < node_count; ++i, ++node_id) {
      e.explore(node_id);
    }

    // Clear cache if over committed
    if (reader.OverCommitted()) {
      reader.Trim();
    }
  }

  // Iterate over tiles. Handle single edges that remain.
  for (GraphId tile_id : tiles) {
    const auto* tile = reader.GetGraphTile(tile_id);
    const auto num_edges = tile->header()->directededgecount();
    GraphId edge_id(tile_id.tileid(), tile_id.level(), 0);
    for (uint32_t i = 0; i < num_edges; ++i, ++edge_id) {
      if (!tracker.get(edge_id)) {
        // Store single edge paths if the edge is allowed.
        auto* edge = tile->directededge(edge_id);
        if (edge_allowed_pred(edge)) {
          // Store the single edge path if the start node is valid. It can be
          // invalid if the end node tile is null (as in a regional extract)
          auto end_nodes = reader.GetDirectedEdgeNodes(tile, edge);
          path p(segment(end_nodes.first, edge_id, end_nodes.second));
          if (p.m_start.Is_Valid()) {
            func(p);
          }
        }
      }
    }

    // Clear cache if over committed
    if (reader.OverCommitted()) {
      reader.Trim();
    }
  }
}

} // namespace merge
} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_MERGE_H_
