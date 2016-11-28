#ifndef VALHALLA_BALDR_MERGE_H_
#define VALHALLA_BALDR_MERGE_H_

#include <deque>
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
  GraphId start() const { return m_start; }
  GraphId end() const { return m_end; }
  GraphId edge() const { return m_edge; }
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

// a place we can mark what edges we've seen, even for the planet we should
// need ~ 100mb, as it allocates one bit per ID. there are currently around
// 453 million ways in OSM, and we might allocate two edge IDs per way,
// giving something like 108mb of bits needed.
struct bitset_t {
  typedef uint64_t value_type;
  static const size_t bits_per_value = sizeof(value_type) * CHAR_BIT;
  static const uint64_t u64_size = static_cast<uint64_t>(bits_per_value);
  static const uint64_t u64_one = static_cast<uint64_t>(1);

  bitset_t(size_t size);
  void set(const uint64_t id);
  bool get(const uint64_t id) const;

protected:
  std::vector<value_type> bits;

  // return ceil(n / q) = r, such that r * q >= n.
  static inline constexpr size_t div_round_up(size_t n, size_t q) {
    return (n + (q - 1)) / q;
  }

  // return the "end" id, one greater than the maximum id, supported by this
  // container.
  inline uint64_t end_id() const {
    return static_cast<uint64_t>(bits.size() * bits_per_value);
  }
};

// edge tracker wraps a bitset to provide compact storage of GraphIds.
//
// A TileSet is used to enumerate all relevant tiles, and the range of tile IDs
// is used to construct a compact range by concatenating all the existing
// compact ranges for each tile.
struct edge_tracker {
  typedef std::unordered_map<GraphId, uint64_t> edge_index_t;

  template <typename TileSet>
  static edge_tracker create(TileSet &tiles, GraphReader &reader);

  bool get(const GraphId &edge_id) const;
  void set(const GraphId &edge_id);

  edge_index_t m_edges_in_tiles;
  //this is how we know what i've touched and what we havent
  bitset_t m_edge_set;

private:
  edge_tracker(edge_index_t &&edges, size_t n)
    : m_edges_in_tiles(std::move(edges))
    , m_edge_set(n)
    {}
};

template <typename TileSet>
edge_tracker edge_tracker::create(TileSet &tiles, GraphReader &reader) {
  //keep the global number of edges encountered at the point we encounter each tile
  //this allows an edge to have a sequential global id and makes storing it very small
  uint64_t edge_count = 0;
  edge_tracker::edge_index_t edges_in_tiles;
  for (GraphId tile_id : tiles) {
    //TODO: just read the header, parsing the whole thing isnt worth it at this point
    edges_in_tiles.emplace(tile_id, edge_count);
    const auto* tile = reader.GetGraphTile(tile_id);
    edge_count += tile->header()->directededgecount();
    // clear the cache if it is overcommitted to avoid running out of memory.
    if (reader.OverCommitted()) {
      reader.Clear();
    }
  }

  return edge_tracker(std::move(edges_in_tiles), edge_count);
}

struct edge_collapser {
  edge_collapser(GraphReader &reader, edge_tracker &tracker, std::function<bool(const DirectedEdge *)> edge_pred, std::function<void(const path &)> func);
  std::pair<GraphId, GraphId> nodes_reachable_from(GraphId node_id);
  GraphId next_node_id(GraphId last_node_id, GraphId node_id);
  GraphId edge_between(GraphId cur, GraphId next);
  void explore(GraphId node_id);
  void explore(GraphId prev, GraphId cur, path &forward, path &reverse);

private:
  GraphReader &m_reader;
  edge_tracker &m_tracker;
  std::function<bool(const DirectedEdge *)> m_edge_predictate;
  std::function<void(const path &)> m_func;
};

path make_single_edge_path(GraphReader &reader, GraphId edge_id);

} // namespace detail

/**
 * Read the graph and merge all compatible edges, calling the provided function
 * with each path that has been found. Each edge in the graph will be part of
 * exactly one path, but paths may contain multiple edges.
 *
 * @param tiles A range object over GraphId for the tiles to consider.
 * @param reader The graph to traverse.
 * @param edge_pred A predicate function which should return true for any edge that can be collapsed.
 * @param func The function to execute for each discovered path.
 */
template <typename TileSet>
void merge(TileSet &tiles, GraphReader &reader, std::function<bool(const DirectedEdge *)> edge_pred, std::function<void(const path &)> func) {
  detail::edge_tracker tracker = detail::edge_tracker::create(tiles, reader);
  detail::edge_collapser e(reader, tracker, edge_pred, func);

  for (GraphId tile_id : tiles) {
    const auto *tile = reader.GetGraphTile(tile_id);
    uint32_t node_count = tile->header()->nodecount();
    for (uint32_t i = 0; i < node_count; ++i) {
      GraphId node_id(tile_id.tileid(), tile_id.level(), i);
      e.explore(node_id);
    }
  }

  for (GraphId tile_id : tiles) {
    const auto *tile = reader.GetGraphTile(tile_id);
    const auto num_edges = tile->header()->directededgecount();
    for (uint32_t i = 0; i < num_edges; ++i) {
      GraphId edge_id(tile_id.tileid(), tile_id.level(), i);
      if (!tracker.get(edge_id)) {
        auto p = detail::make_single_edge_path(reader, edge_id);
        func(p);
      }
    }
  }
}

}
}
}

#endif  // VALHALLA_BALDR_MERGE_H_
