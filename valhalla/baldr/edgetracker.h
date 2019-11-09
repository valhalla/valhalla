#ifndef VALHALLA_BALDR_EDGE_TRACKER_H_
#define VALHALLA_BALDR_EDGE_TRACKER_H_

#include <cstdint>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>

namespace valhalla {
namespace baldr {

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

  template <typename TileSet> static edge_tracker create(TileSet& tiles, GraphReader& reader);

  bool get(const GraphId& edge_id) const;
  void set(const GraphId& edge_id);

  edge_index_t m_edges_in_tiles;
  // this is how we know what i've touched and what we havent
  bitset_t m_edge_set;

private:
  edge_tracker(edge_index_t&& edges, size_t n) : m_edges_in_tiles(std::move(edges)), m_edge_set(n) {
  }
};

template <typename TileSet> edge_tracker edge_tracker::create(TileSet& tiles, GraphReader& reader) {
  // keep the global number of edges encountered at the point we encounter each tile
  // this allows an edge to have a sequential global id and makes storing it very small
  uint64_t edge_count = 0;
  edge_tracker::edge_index_t edges_in_tiles;
  for (GraphId tile_id : tiles) {
    // TODO: just read the header, parsing the whole thing isnt worth it at this point
    edges_in_tiles.emplace(tile_id, edge_count);
    const auto* tile = reader.GetGraphTile(tile_id);
    edge_count += tile->header()->directededgecount();
    // clear the cache if it is overcommitted to avoid running out of memory.
    if (reader.OverCommitted()) {
      reader.Trim();
    }
  }

  return edge_tracker(std::move(edges_in_tiles), edge_count);
}

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_EDGE_TRACKER_H_
