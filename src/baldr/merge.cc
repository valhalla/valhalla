#include "baldr/merge.h"
#include "baldr/graphreader.h"

#include <boost/range/adaptor/map.hpp>

namespace bra = boost::adaptors;

namespace valhalla {
namespace baldr {
namespace merge {

namespace {

uint64_t count_tiles_in_levels(GraphReader &reader) {
  uint64_t tile_count = 0;
  for (auto level : reader.GetTileHierarchy().levels() | bra::map_values) {
    tile_count += level.tiles.ncolumns() * level.tiles.nrows();
  }
  return tile_count;
}

namespace iter {

// the "edges" struct provides a container wrapper for the edges leaving a node
// which is compatible with C++ range-based for loops and can make reading code
// a bit nicer. for example:
//
//     for (const auto &pair : iter::edges(reader, node_id)) {
//       ...
//     }
//
// the iterator can be dereferenced into a pair of const DirectedEdge * and
// GraphId. this allows iteration over both the edge and the ID of the edge,
// which isn't directly obtainable from the DirectedEdge structure.
struct edges {
  struct const_iterator {
    const DirectedEdge *ptr;
    GraphId id;

    const_iterator() : ptr(nullptr), id(0) {}
    const_iterator(const DirectedEdge *p, GraphId i) : ptr(p), id(i) {}

    const_iterator operator+(uint64_t i) const {
      return const_iterator(ptr + i, id + i);
    }

    const_iterator &operator++() {
      ++ptr;
      id++;
      return *this;
    }

    const_iterator operator++(int) {
      const_iterator ret = *this;
      ++(*this);
      return ret;
    }

    std::pair<const DirectedEdge *const, GraphId> operator*() const {
      return std::pair<const DirectedEdge *const, GraphId>(ptr, GraphId(id));
    }

    bool operator!=(const const_iterator &other) const {
      return (ptr != other.ptr) || (id != other.id);
    }
  };

  edges(GraphReader &reader, GraphId node_id) {
    auto *tile = reader.GetGraphTile(node_id);
    auto *node_info = tile->node(node_id);

    auto edge_idx = node_info->edge_index();
    m_begin = const_iterator(tile->directededge(edge_idx),
                             node_id.Tile_Base() + uint64_t(edge_idx));
    m_end = m_begin + node_info->edge_count();
  }

  const_iterator begin() const { return m_begin; }
  const_iterator end()   const { return m_end; }

private:
  const_iterator m_begin, m_end;
};

} // namespace iter

} // anonymous namespace

namespace detail {

bitset_t::bitset_t(size_t size) : bits(div_round_up(size, bits_per_value)) {}

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

bool edge_tracker::get(const GraphId &edge_id) const {
  auto itr = m_edges_in_tiles.find(edge_id.Tile_Base());
  assert(itr != m_edges_in_tiles.end());
  return m_edge_set.get(edge_id.id() + itr->second);
}

void edge_tracker::set(const GraphId &edge_id) {
  auto itr = m_edges_in_tiles.find(edge_id.Tile_Base());
  assert(itr != m_edges_in_tiles.end());
  m_edge_set.set(edge_id.id() + itr->second);
}

edge_collapser::edge_collapser(GraphReader &reader, edge_tracker &tracker, std::function<bool(const DirectedEdge *)> edge_pred, std::function<void(const path &)> func)
  : m_reader(reader)
  , m_tracker(tracker)
  , m_edge_predictate(edge_pred)
  , m_func(func)
{}

// returns the pair of nodes reachable from the given @node_id where they
// are the only two nodes reachable by non-shortcut edges, and none of the
// edges of @node_id cross into a different level.
std::pair<GraphId, GraphId> edge_collapser::nodes_reachable_from(GraphId node_id) {
  static const std::pair<GraphId, GraphId> none;
  GraphId first, second;

  for (const auto &edge : iter::edges(m_reader, node_id)) {
    // nodes which connect to ferries, transit or to a different level
    // shouldn't be collapsed.
    if (!m_edge_predictate(edge.first)) {
      return none;
    }

    // shortcut edges should be ignored
    if (edge.first->shortcut()) {
      continue;
    }

    if (first) {
      if (second) {
        // can't add a third, that means this node is a true junction.
        return none;

      } else {
        second = edge.first->endnode();
      }
    } else {
      first = edge.first->endnode();
    }
  }

  if (first && second) {
    return std::make_pair(first, second);
  } else {
    return none;
  }
}

GraphId edge_collapser::next_node_id(GraphId last_node_id, GraphId node_id) {
  //
  //        -->--     -->--
  //   \   /  e4 \   /  e1 \   /
  //   -(p)       (c)       (n)-
  //   /   \ e3  /   \ e2  /   \
  //        --<--     --<--
  //
  // given p (last_node_id) and c (node_id), return n if there is such a node.
  auto nodes = nodes_reachable_from(node_id);
  if (!nodes.first || !nodes.second) {
    return GraphId();
  }
  assert(nodes.first == last_node_id || nodes.second == last_node_id);
  if (nodes.first == last_node_id) {
    return nodes.second;
  } else {
    return nodes.first;
  }
}

GraphId edge_collapser::edge_between(GraphId cur, GraphId next) {
  GraphId edge_id;
  for (const auto &edge : iter::edges(m_reader, cur)) {
    if (edge.first->endnode() == next) {
      edge_id = edge.second;
      break;
    }
  }
  assert(bool(edge_id));
  return edge_id;
}

// explore starts walking the graph from a single node, building a forward and
// reverse path of edges as long as the nodes found haven't been explored
// before and have exactly two out-edges.
//
// the user-defined function is called for each path found.
void edge_collapser::explore(GraphId node_id) {
  auto nodes = nodes_reachable_from(node_id);
  if (!nodes.first || !nodes.second) {
    return;
  }

  // if either edge has been marked, then don't explore down either of them.
  if (m_tracker.get(edge_between(node_id, nodes.first)) ||
      m_tracker.get(edge_between(node_id, nodes.second))) {
    return;
  }

  path forward(node_id), reverse(node_id);

  explore(node_id, nodes.first,  forward, reverse);
  explore(node_id, nodes.second, reverse, forward);

  m_func(forward);
  m_func(reverse);
}

// walk in a single direction, using the "direction" given by two nodes to
// select which edge is considered to be "forward".
void edge_collapser::explore(GraphId prev, GraphId cur, path &forward, path &reverse) {
  const auto original_node_id = prev;

  GraphId maybe_next;
  do {
    auto e1 = edge_between(prev, cur);
    forward.push_back(segment(prev, e1, cur));
    m_tracker.set(e1);
    auto e2 = edge_between(cur, prev);
    reverse.push_front(segment(cur, e2, prev));
    m_tracker.set(e2);

    maybe_next = next_node_id(prev, cur);
    if (maybe_next) {
      prev = cur;
      cur = maybe_next;
      if (cur == original_node_id) {
        // circular!
        break;
      }
    }
  } while (maybe_next);
}

// utility function to make a path out of a single edge. this is called once all
// the collapsible paths have been found and single edges are all that's left.
path make_single_edge_path(GraphReader &reader, GraphId edge_id) {
  auto *edge = reader.GetGraphTile(edge_id)->directededge(edge_id);
  auto node_id = edge->endnode();
  auto opp_edge_idx = edge->opp_index();
  auto edge_idx = reader.GetGraphTile(node_id)->node(node_id)->edge_index() + opp_edge_idx;
  GraphId opp_edge_id(node_id.tileid(), node_id.level(), edge_idx);
  auto *opp_edge = reader.GetGraphTile(opp_edge_id)->directededge(opp_edge_id);
  auto start_node_id = opp_edge->endnode();

  path p(segment(start_node_id, edge_id, node_id));
  return p;
}

} // namespace detail

segment::segment(GraphId start, GraphId edge, GraphId end)
  : m_start(start)
  , m_edge(edge)
  , m_end(end)
{}

path::path(segment s)
  : m_start(s.start())
  , m_end(s.end()) {
  m_edges.push_back(s.edge());
}

path::path(GraphId node_id)
  : m_start(node_id)
  , m_end(node_id) {
}

void path::push_back(segment s) {
  assert(s.start() == m_end);
  m_end = s.end();
  m_edges.push_back(s.edge());
}

void path::push_front(segment s) {
  assert(s.end() == m_start);
  m_start = s.start();
  m_edges.push_front(s.edge());
}

}
}
}
