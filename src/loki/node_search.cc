#include "loki/node_search.h"
#include "valhalla/midgard/tiles.h"

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;

namespace {

struct tile_cache {
  tile_cache(vb::GraphReader &reader)
    : m_reader(reader), m_last_tile_id(), m_tile(nullptr),
      m_nodes(nullptr), m_edges(nullptr) {}

  inline tile_cache &operator()(vb::GraphId id) {
    const auto id_base = id.Tile_Base();
    if (m_last_tile_id != id_base) {
      lookup(id_base);
    }
    return *this;
  }

  inline const vb::GraphTile *tile() {
    return m_tile;
  }

  inline const vb::NodeInfo &node(vb::GraphId id) {
    return m_nodes[id.id()];
  }

  inline const vb::DirectedEdge &edge(vb::GraphId id) {
    return m_edges[id.id()];
  }

private:
  void lookup(vb::GraphId id_base) {
    m_last_tile_id = id_base;
    m_tile = m_reader.GetGraphTile(id_base);

    if (m_tile != nullptr) {
      m_nodes = m_tile->node(0);
      m_edges = m_tile->directededge(0);
    } else {
      m_nodes = nullptr;
      m_edges = nullptr;
    }
  }

  vb::GraphReader &m_reader;
  vb::GraphId m_last_tile_id;
  const vb::GraphTile *m_tile;
  const vb::NodeInfo *m_nodes;
  const vb::DirectedEdge *m_edges;
};

} // anonymous namespace

namespace valhalla {
namespace loki {

std::vector<baldr::GraphId>
nodes_in_bbox(const midgard::AABB2<midgard::PointLL> &bbox, baldr::GraphReader& reader) {
  std::vector<baldr::GraphId> nodes;

  auto hierarchy = reader.GetTileHierarchy();
  auto tiles = hierarchy.levels().rbegin()->second.tiles;

  auto intersections = tiles.Intersect(bbox);

  // we cache the last tile lookup, since the nodes and tweeners arrays are in
  // order then this guarantees the smallest number of times we have to look up
  // a new tile from the reader.
  tile_cache cache(reader);

  // keep a set of (hopefully rare) "tweeners", which are edges which cross
  // from one tile into another. to find their end-node, we need to load up
  // a different tile, which is best done in a second loop to avoid too many
  // tile lookups.
  std::vector<vb::GraphId> tweeners;

  // keep a set of (again, hopefully rare) combinations of node and index for
  // when we encounter an edge whose opposite is in a different tile. the
  // opposite is used to get the start node of the current edge.
  //
  // TODO: could be really naughty here and stuff the opposite index into the
  // extra unused field of the GraphId... but that might cause real headaches
  // down the road.
  std::vector<std::pair<vb::GraphId, uint32_t> > opposite_edges;

  for (const auto &entry : intersections) {
    // TODO: level fixed at 2 for intersections?
    vb::GraphId tile_id(entry.first, 2, 0);
    auto *tile = reader.GetGraphTile(tile_id);
    // tile might not exist - the Tiles::Intersect routine returns all tiles
    // which might intersect, regardless of whether any of them exist.
    if (tile == nullptr) {
      continue;
    }

    // fetch the beginning of the edges and nodes arrays as raw pointers to
    // avoid the slowness of bounds checks in the inner loop. we assume that the
    // tile is valid and only references edges and nodes which exist.
    auto *tile_edges = tile->directededge(0);
    auto *tile_nodes = tile->node(0);

    for (auto bin_id : entry.second) {
      for (auto edge_id : tile->GetBin(bin_id)) {
        //assert(edge_id.fields.spare == 0);
        if (edge_id.Tile_Base() == tile_id) {
          // we want _both_ nodes attached to this edge. the end node is easy
          // because the ID is available from the edge itself.
          const auto &edge = tile_edges[edge_id.id()];
          const auto endnode = edge.endnode();
          nodes.push_back(endnode);

          // however, the start node - even though we know it must be in the
          // same tile as the edge, is only available as the end node of the
          // opposite edge, which might be in a different tile!
          const auto opp_index = edge.opp_index();
          if (endnode.Tile_Base() == tile_id) {
            const auto &node = tile_nodes[endnode.id()];
            assert(opp_index < node.edge_count());
            const auto startnode = tile_edges[node.edge_index() + opp_index].endnode();
            nodes.push_back(startnode);
          }
          // save the (node, index) pair for later.
          else {
            opposite_edges.emplace_back(endnode, opp_index);
          }
        }
        // ignore edges which lead to different levels
        // TODO: is this right?
        else if (edge_id.level() == tile_id.level()) {
          tweeners.emplace_back(edge_id);
        }
      }
    }
  }

  // sort the tweeners, so that they're in tile order
  std::sort(tweeners.begin(), tweeners.end());
  for (auto edge_id : tweeners) {
    const auto &edge = cache(edge_id).edge(edge_id);
    const auto endnode = edge.endnode();
    nodes.push_back(endnode);

    const auto opp_index = edge.opp_index();
    if (endnode.Tile_Base() == edge_id.Tile_Base()) {
      const auto &node = cache(endnode).node(endnode);
      assert(opp_index < node.edge_count());
      vb::GraphId opp_edge_id(edge_id.tileid(), edge_id.level(), node.edge_index() + opp_index);
      const auto startnode = cache(opp_edge_id).edge(opp_edge_id).endnode();
      nodes.push_back(startnode);
    }
    // save the (node, index) pair for later.
    else {
      opposite_edges.emplace_back(endnode, opp_index);
    }
  }

  // don't need tweeners any more
  tweeners.clear();

  // TODO: sort and loop over opposite_edges

  // erase the duplicates
  std::sort(nodes.begin(), nodes.end());
  auto uniq_end = std::unique(nodes.begin(), nodes.end());

  // the nodes at this point are just candidates, we need to filter through them
  // to figure out which ones are actually within the bounding box.
  auto rm_end = std::remove_if(
    nodes.begin(), uniq_end, [&](vb::GraphId node_id) -> bool {
      auto coord = cache(node_id).node(node_id).latlng();
      return !bbox.Contains(coord);
    });

  // remove any of the space at the end of the nodes array which was removed by
  // either the unique or bounding box checks.
  nodes.erase(rm_end, nodes.end());

  return nodes;
}

}
}
