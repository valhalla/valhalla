#include "loki/node_search.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/tiles.h"
#include <cmath>

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;

namespace {

struct tile_cache {
  tile_cache(vb::GraphReader& reader)
      : m_reader(reader), m_last_tile_id(), m_tile(nullptr), m_nodes(nullptr), m_edges(nullptr) {
  }

  inline tile_cache& operator()(vb::GraphId id) {
    const auto id_base = id.Tile_Base();
    if (m_last_tile_id != id_base) {
      lookup(id_base);
    }
    return *this;
  }

  inline const vb::GraphTile* tile() {
    return m_tile;
  }

  inline const vb::NodeInfo& node(vb::GraphId id) {
    return m_nodes[id.id()];
  }

  inline vm::PointLL node_ll(const vb::GraphId id) {
    return m_tile->get_node_ll(id);
  }

  inline const vb::DirectedEdge& edge(vb::GraphId id) {
    return m_edges[id.id()];
  }

  inline bool exists() const {
    return m_tile != nullptr;
  }

  inline vb::GraphId tile_id() const {
    return m_last_tile_id;
  }

private:
  void lookup(vb::GraphId id_base) {
    m_last_tile_id = id_base;
    m_tile = m_reader.GetGraphTile(id_base);

    if (m_tile != nullptr) {
      // fetch the beginning of the edges and nodes arrays as raw pointers to
      // avoid the slowness of bounds checks in the inner loop. we assume that the
      // tile is valid and only references edges and nodes which exist.
      //
      // note that the tile might not have any edges or nodes, and might have bins
      // that refer only to edges in other tiles.
      const auto tile_edge_count = m_tile->header()->directededgecount();
      const auto tile_node_count = m_tile->header()->nodecount();
      m_edges = tile_edge_count > 0 ? m_tile->directededge(0) : nullptr;
      m_nodes = tile_node_count > 0 ? m_tile->node(0) : nullptr;

    } else {
      m_nodes = nullptr;
      m_edges = nullptr;
    }
  }

  vb::GraphReader& m_reader;
  vb::GraphId m_last_tile_id;
  const vb::GraphTile* m_tile;
  const vb::NodeInfo* m_nodes;
  const vb::DirectedEdge* m_edges;
};

// find the bin which contains pt and return its bounding box
vm::AABB2<vm::PointLL> bin_bbox(const vm::Tiles<vm::PointLL>& tiles, const vm::PointLL& pt) {

  const auto tile_id = tiles.TileId(pt);
  const auto tile_box = tiles.TileBounds(tile_id);
  const int32_t nsubdivisions = tiles.nsubdivisions();
  const float row_delta = nsubdivisions / tile_box.Width();
  const float col_delta = nsubdivisions / tile_box.Height();

  int32_t sub_row = (pt.x() - tile_box.minx()) * row_delta;
  int32_t sub_col = (pt.y() - tile_box.miny()) * col_delta;

  return vm::AABB2<vm::PointLL>(tile_box.minx() + sub_row * row_delta,
                                tile_box.miny() + sub_col * col_delta,
                                tile_box.minx() + (sub_row + 1) * row_delta,
                                tile_box.miny() + (sub_col + 1) * col_delta);
}

// return a set of bounding boxes which have been expanded as necessary to make
// sure that none of its boundaries are coincident with the divisions between
// tiles or the subdivisions between bins within the tiles.
//
// multiple bounding boxes can be returned if the input box is close to the
// anti-meridian.
std::vector<vm::AABB2<vm::PointLL>>
expand_bbox_across_boundaries(const vm::AABB2<vm::PointLL>& bbox,
                              const vm::Tiles<vm::PointLL>& tiles) {

  // use a tolerance to ensure that, for all tiles, the lat/lng used for the
  // bounding box is is distinct from the one used for the tile / bin
  // boundary.
  static constexpr float tolerance = std::numeric_limits<float>::epsilon() * 180.0f;

  auto minx = bbox.minx();
  auto miny = bbox.miny();
  auto maxx = bbox.maxx();
  auto maxy = bbox.maxy();

  auto ll_bin_pt = bin_bbox(tiles, bbox.minpt()).minpt();
  if (std::abs(ll_bin_pt.lng() - minx) < tolerance) {
    minx = ll_bin_pt.lng() - tolerance;
  }
  if (std::abs(ll_bin_pt.lat() - miny) < tolerance) {
    miny = ll_bin_pt.lat() - tolerance;
  }

  auto ur_bin_pt = bin_bbox(tiles, bbox.maxpt()).maxpt();
  if (std::abs(ur_bin_pt.lng() - maxx) < tolerance) {
    maxx = ur_bin_pt.lng() + tolerance;
  }
  if (std::abs(ur_bin_pt.lat() - maxy) < tolerance) {
    maxy = ur_bin_pt.lat() + tolerance;
  }

  // cap latitude extents to +/-90, as there is nothing on the "other side" of
  // these limits to include.
  if (miny < -90.0f) {
    miny = -90.0f;
  }
  if (maxy > 90.0f) {
    maxy = 90.0f;
  }

  std::vector<vm::AABB2<vm::PointLL>> boxes;

  // if the longitude extents wrap across +/-180, then either merge them if
  // they overlap, or create a second bounding box if they do not.
  if (minx + 360.0f < maxx) {
    minx = -180.0f;
    maxx = 180.0f;

  } else if (minx < -180.0f) {
    // given that minx + 360 >= maxx && minx < -180, this implies that maxx
    // < 180.0f, and that the boxes do not overlap.
    boxes.push_back(vm::AABB2<vm::PointLL>(minx + 360.0f, miny, 180.0f, maxy));
    minx = -180.0f;

  } else if (maxx > 180.0f) {
    // given the above two cases, this means that minx > maxx - 360, so the
    // boxes do not overlap.
    boxes.push_back(vm::AABB2<vm::PointLL>(-180.0f, miny, maxx - 360.0f, maxy));
    maxx = 180.0f;
  }

  boxes.push_back(vm::AABB2<vm::PointLL>(minx, miny, maxx, maxy));

  return boxes;
}

// for each bounding box in boxes, calculate the intersection and merge all the
// results together, then return it.
std::unordered_map<int32_t, std::unordered_set<uint16_t>>
merge_intersections(const std::vector<vm::AABB2<vm::PointLL>>& boxes,
                    const vm::Tiles<vm::PointLL>& tiles) {

  std::unordered_map<int32_t, std::unordered_set<uint16_t>> result;

  for (const auto& box : boxes) {
    auto intersection = tiles.Intersect(box);

    for (const auto& entry : intersection) {
      result[entry.first].insert(entry.second.begin(), entry.second.end());
    }
  }

  return result;
}

struct filtered_nodes {
  explicit filtered_nodes(const vm::AABB2<vm::PointLL>& b, std::vector<vb::GraphId>& nodes)
      : m_box(b), m_nodes(nodes) {
  }

  inline void push_back(vb::GraphId id, const vm::PointLL& ll) {
    if (m_box.Contains(ll)) {
      m_nodes.push_back(id);
    }
  }

private:
  vm::AABB2<vm::PointLL> m_box;
  std::vector<vb::GraphId>& m_nodes;
};

// functor to sort GraphId objects by level, tile then id within the tile.
struct sort_by_tile {
  sort_by_tile() {
  }

  inline bool operator()(vb::GraphId a, vb::GraphId b) const {
    return ((a.level() < b.level()) ||
            ((a.level() == b.level()) &&
             ((a.tileid() < b.tileid()) || ((a.tileid() == b.tileid()) && (a.id() < b.id())))));
  }
};

// functor to sort pairs of (GraphId, uint32_t) objects by the functor above
struct sort_pair_by_tile {
  typedef std::pair<vb::GraphId, uint32_t> value_type;
  static const sort_by_tile sort_tile;

  inline bool operator()(const value_type& a, const value_type& b) {
    return (sort_tile(a.first, b.first) || ((a.first == b.first) && (a.second < b.second)));
  }
};

const sort_by_tile sort_pair_by_tile::sort_tile;

struct node_collector {
  node_collector(tile_cache& cache, filtered_nodes& nodes) : m_cache(cache), m_nodes(nodes) {
  }

  void add_edge(vb::GraphId edge_id) {
    const auto tile_id = m_cache.tile_id();

    if (edge_id.Tile_Base() == tile_id) {
      // we want _both_ nodes attached to this edge. the end node is easy
      // because the ID is available from the edge itself.
      const auto& edge = m_cache.edge(edge_id);
      const auto endnode_id = edge.endnode();

      // however, the start node - even though we know it must be in the
      // same tile as the edge, is only available as the end node of the
      // opposite edge, which might be in a different tile!
      const auto opp_index = edge.opp_index();
      add_node_with_opposite(endnode_id, opp_index);
    }
    // edges which lead to different levels or tiles are tweeners which we
    // gather and process in a second sweep. note that up/down transfer
    // edges and transit edges are not present in the bins, so we don't
    // need to screen them out.
    else {
      m_tweeners.emplace_back(edge_id);
    }
  }

  void add_node_with_opposite(vb::GraphId node_id, uint32_t opp_index) {
    const auto tile_id = m_cache.tile_id();

    if (node_id.Tile_Base() == tile_id) {
      const auto& node = m_cache.node(node_id);

      // node is in this tile, so add it to the collection
      m_nodes.push_back(node_id, m_cache.node_ll(node_id));

      if (opp_index < node.edge_count()) {
        // assert(opp_index < node.edge_count());
        // add the node at the other end of the opposite - which would be the
        // start node of the original edge.
        auto opp_id = tile_id + uint64_t(node.edge_index() + opp_index);
        add_node(m_cache.edge(opp_id).endnode());
      } else {
        LOG_ERROR("Opposing index >= node edge count!");
      }
    }
    // the node is not in this tile, so we cannot look up its position or edge
    // offset yet. save the (node, index) pair for later.
    else {
      m_opposite_edges.emplace_back(node_id, opp_index);
    }
  }

  void add_node(vb::GraphId node_id) {
    const auto tile_id = m_cache.tile_id();

    if (node_id.Tile_Base() == tile_id) {
      const auto& node = m_cache.node(node_id);

      // node is in this tile, so add it to the collection
      m_nodes.push_back(node_id, m_cache.node_ll(node_id));
    }
    // node is not in this tile, so save it for later
    else {
      m_backfill_nodes.push_back(node_id);
    }
  }

  void finish_tweeners() {
    std::sort(m_tweeners.begin(), m_tweeners.end(), sort_by_tile());
    for (auto tweener : m_tweeners) {
      if (m_cache(tweener).exists()) {
        add_edge(tweener);
      }
    }
    m_tweeners.clear();
  }

  void finish_opposite_edges() {
    std::sort(m_opposite_edges.begin(), m_opposite_edges.end(), sort_pair_by_tile());
    for (const auto& entry : m_opposite_edges) {
      if (m_cache(entry.first).exists()) {
        add_node_with_opposite(entry.first, entry.second);
      }
    }
    m_opposite_edges.clear();
  }

  void finish_backfill_nodes() {
    std::sort(m_backfill_nodes.begin(), m_backfill_nodes.end(), sort_by_tile());
    for (auto node_id : m_backfill_nodes) {
      if (m_cache(node_id).exists()) {
        add_node(node_id);
      }
    }
    m_backfill_nodes.clear();
  }

  void finish() {
    finish_tweeners();
    finish_opposite_edges();
    finish_backfill_nodes();
  }

private:
  // object which caches access to tiles. therefore it's much more efficient to
  // access objects grouped by (level, tile).
  tile_cache& m_cache;

  // a bounding-box filtered list of nodes. we append nodes to this, and when
  // the algorithm finishes, this list can be made unique.
  filtered_nodes& m_nodes;

  // keep a set of (hopefully rare) "tweeners", which are edges which cross
  // from one tile into another. to find their end-node, we need to load up
  // a different tile, which is best done in a second loop to avoid too many
  // tile lookups.
  std::vector<vb::GraphId> m_tweeners;

  // keep a set of (again, hopefully rare) combinations of node and index for
  // when we encounter an edge whose opposite is in a different tile. the
  // opposite is used to get the start node of the current edge.
  //
  // TODO: could be really naughty here and stuff the opposite index into the
  // extra unused field of the GraphId... but that might cause real headaches
  // down the road.
  std::vector<std::pair<vb::GraphId, uint32_t>> m_opposite_edges;

  // keep a set of (rarest of them all) nodes which are in a different tile from
  // their opposite edge, which is in a different tile from the original node.
  std::vector<vb::GraphId> m_backfill_nodes;
};

} // anonymous namespace

namespace valhalla {
namespace loki {

std::vector<baldr::GraphId> nodes_in_bbox(const vm::AABB2<vm::PointLL>& bbox,
                                          baldr::GraphReader& reader) {
  std::vector<vb::GraphId> nodes;

  auto tiles = vb::TileHierarchy::levels().rbegin()->second.tiles;
  const uint8_t bin_level = vb::TileHierarchy::levels().rbegin()->second.level;

  // if the bbox only touches the edge of the tile or bin, then we need to
  // include neighbouring bins as well, in case both the edge and its opposite
  // were tie-broken into a bin which doesn't intersect the original bbox.
  auto expanded_bboxes = expand_bbox_across_boundaries(bbox, tiles);
  auto intersections = merge_intersections(expanded_bboxes, tiles);

  // we cache the last tile lookup, since the nodes and tweeners arrays are in
  // order then this guarantees the smallest number of times we have to look up
  // a new tile from the reader.
  tile_cache cache(reader);

  // wrap the nodes in a filter so that only nodes contained within the bounding
  // box are appended to the vector. there might be duplicates, so we have to
  // sort and uniq the vector later.
  filtered_nodes filtered(bbox, nodes);

  // a wrapper process which aims to order the lookups against tiles into a
  // number of sequential passes through the set of tiles.
  node_collector collector(cache, filtered);

  for (const auto& entry : intersections) {
    vb::GraphId tile_id(entry.first, bin_level, 0);
    // tile might not exist - the Tiles::Intersect routine returns all tiles
    // which might intersect, regardless of whether any of them exist.
    auto& tile = cache(tile_id);
    if (!tile.exists()) {
      continue;
    }

    for (auto bin_id : entry.second) {
      for (auto edge_id : tile.tile()->GetBin(bin_id)) {
        collector.add_edge(edge_id);
      }
    }
  }

  // finish the collector by going over any stored edges or nodes which weren't
  // accessible in the current tile at the time they were found.
  collector.finish();

  // erase the duplicates
  std::sort(nodes.begin(), nodes.end());
  auto uniq_end = std::unique(nodes.begin(), nodes.end());

  // remove any of the space at the end of the nodes array which is unused since
  // duplicates were removed.
  nodes.erase(uniq_end, nodes.end());

  return nodes;
}

} // namespace loki
} // namespace valhalla
