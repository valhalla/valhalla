#include "baldr/merge.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"

#include <boost/range/adaptor/map.hpp>
#include <utility>

namespace bra = boost::adaptors;

namespace valhalla {
namespace baldr {
namespace merge {

namespace {
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
    const DirectedEdge* ptr;
    GraphId id;

    const_iterator() : ptr(nullptr), id(0) {
    }
    const_iterator(const DirectedEdge* p, GraphId i) : ptr(p), id(i) {
    }

    const_iterator operator+(uint64_t i) const {
      return const_iterator(ptr + i, id + i);
    }

    const_iterator& operator++() {
      ++ptr;
      ++id;
      return *this;
    }

    const_iterator operator++(int) {
      const_iterator ret = *this;
      ++(*this);
      return ret;
    }

    std::pair<const DirectedEdge* const, GraphId> operator*() const {
      return std::pair<const DirectedEdge* const, GraphId>(ptr, GraphId(id));
    }

    bool operator!=(const const_iterator& other) const {
      return (ptr != other.ptr) || (id != other.id);
    }
  };

  edges(const graph_tile_ptr& tile, GraphId node_id) {
    auto* node_info = tile->node(node_id);
    auto edge_idx = node_info->edge_index();
    m_begin = const_iterator(tile->directededge(edge_idx), node_id.Tile_Base() + uint64_t(edge_idx));
    m_end = m_begin + node_info->edge_count();
  }

  const_iterator begin() const {
    return m_begin;
  }
  const_iterator end() const {
    return m_end;
  }

private:
  const_iterator m_begin, m_end;
};

} // namespace iter

} // anonymous namespace

namespace detail {

edge_collapser::edge_collapser(GraphReader& reader,
                               edge_tracker& tracker,
                               std::function<bool(const DirectedEdge*)> edge_merge_pred,
                               std::function<bool(const DirectedEdge*)> edge_allowed_pred,
                               std::function<void(const path&)> func)
    : m_reader(reader), m_tracker(tracker), m_edge_merge_predicate(std::move(edge_merge_pred)),
      m_edge_allowed_predicate(std::move(edge_allowed_pred)), m_func(std::move(func)) {
}

// returns the pair of nodes reachable from the given @node_id where they
// are the only two nodes reachable based on the predicate methods.
std::pair<GraphId, GraphId> edge_collapser::nodes_reachable_from(GraphId node_id) {
  static const std::pair<GraphId, GraphId> none;
  GraphId first, second;

  // Get the tile of the node, return none if the tile is not found
  auto tile = m_reader.GetGraphTile(node_id);
  if (tile == nullptr) {
    return none;
  }
  for (const auto& edge : iter::edges(tile, node_id)) {
    // Check if this edge excludes merging at this node
    if (!m_edge_merge_predicate(edge.first)) {
      return none;
    }

    // Skip this edge if it is not allowed on the path
    if (!m_edge_allowed_predicate(edge.first)) {
      continue;
    }

    // Exclude merging at a node where loops occur (non-unique end node)
    if (edge.first->endnode() == first || edge.first->endnode() == second) {
      return none;
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
  /*
   *        -->--     -->--
   *   \   /  e4 \   /  e1 \   /
   *   -(p)       (c)       (n)-
   *   /   \ e3  /   \ e2  /   \
   *        --<--     --<--
   *
   * given p (last_node_id) and c (node_id), return n if there is such a node.
   */
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

// Get the edge Id between 2 nodes. Will return an invalid edge Id if there
// is no "allowed" edge between the 2 nodes.
GraphId edge_collapser::edge_between(GraphId cur, GraphId next) {
  // Get the tile of the current node, return none if the tile is not found
  auto tile = m_reader.GetGraphTile(cur);
  if (tile == nullptr) {
    return {};
  }
  for (const auto& edge : iter::edges(tile, cur)) {
    // Return the edge Id if endnode matches and the edge is allowed
    if (edge.first->endnode() == next && m_edge_allowed_predicate(edge.first)) {
      return edge.second;
    }
  }
  return {};
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

  // if either edge has been marked or is invalid (was not allowed in the
  // edge between method) then don't explore down either of them.
  GraphId e1 = edge_between(node_id, nodes.first);
  if (!e1.Is_Valid() || m_tracker.get(e1)) {
    return;
  }
  GraphId e2 = edge_between(node_id, nodes.second);
  if (!e2.Is_Valid() || m_tracker.get(e2)) {
    return;
  }

  // Construct forward path and reverse path. If a loop is detected on the
  // first explore method do not call the second as it will created overlapping
  // edges.
  path forward(node_id), reverse(node_id);
  bool loop = explore(node_id, nodes.first, forward, reverse);
  if (!loop) {
    explore(node_id, nodes.second, reverse, forward);
  }
  m_func(forward);
  m_func(reverse);
}

// walk in a single direction, using the "direction" given by two nodes to
// select which edge is considered to be "forward". Returns true if a loop
// is found, otherwise returns false.
bool edge_collapser::explore(GraphId prev, GraphId cur, path& forward, path& reverse) {
  const auto original_node_id = prev;

  GraphId maybe_next;
  do {
    auto e1 = edge_between(prev, cur);
    if (!e1.Is_Valid()) {
      return false;
    }
    forward.push_back(segment(prev, e1, cur));
    m_tracker.set(e1);
    auto e2 = edge_between(cur, prev);
    if (!e2.Is_Valid()) {
      return false;
    }
    reverse.push_front(segment(cur, e2, prev));
    m_tracker.set(e2);

    maybe_next = next_node_id(prev, cur);
    if (maybe_next) {
      prev = cur;
      cur = maybe_next;
      if (cur == original_node_id) {
        // Loop is detected - add last edge and return true to indicate a loop
        auto e1 = edge_between(prev, cur);
        if (!e1.Is_Valid()) {
          return false;
        }
        forward.push_back(segment(prev, e1, cur));
        m_tracker.set(e1);
        auto e2 = edge_between(cur, prev);
        if (!e2.Is_Valid()) {
          return false;
        }
        reverse.push_front(segment(cur, e2, prev));
        m_tracker.set(e2);
        return true;
      }
    }
  } while (maybe_next);
  return false;
}

} // namespace detail

segment::segment(GraphId start, GraphId edge, GraphId end)
    : m_start(start), m_edge(edge), m_end(end) {
}

path::path(segment s) : m_start(s.start()), m_end(s.end()) {
  m_edges.push_back(s.edge());
}

path::path(GraphId node_id) : m_start(node_id), m_end(node_id) {
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

} // namespace merge
} // namespace baldr
} // namespace valhalla
