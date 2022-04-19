#include "baldr/edgetracker.h"
#include "baldr/graphreader.h"

namespace valhalla {
namespace baldr {

bitset_t::bitset_t(size_t size) : bits(div_round_up(size, bits_per_value)) {
}

void bitset_t::set(const uint64_t id) {
  if (id >= end_id()) {
    throw std::runtime_error("id out of bounds");
  }
  bits[id / u64_size] |= u64_one << (id % u64_size);
}

bool bitset_t::get(const uint64_t id) const {
  if (id >= end_id()) {
    throw std::runtime_error("id out of bounds");
  }
  return bits[id / u64_size] & (u64_one << (id % u64_size));
}

bool edge_tracker::get(const GraphId& edge_id) const {
  auto itr = m_edges_in_tiles.find(edge_id.Tile_Base());
  assert(itr != m_edges_in_tiles.end());
  return m_edge_set.get(edge_id.id() + itr->second);
}

void edge_tracker::set(const GraphId& edge_id) {
  auto itr = m_edges_in_tiles.find(edge_id.Tile_Base());
  assert(itr != m_edges_in_tiles.end());
  m_edge_set.set(edge_id.id() + itr->second);
}

} // namespace baldr
} // namespace valhalla
