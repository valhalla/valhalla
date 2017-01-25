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

// find the bin which contains pt and return its bounding box
vm::AABB2<vm::PointLL> bin_bbox(
  const vm::Tiles<vm::PointLL> &tiles,
  const vm::PointLL &pt) {

  const auto tile_id = tiles.TileId(pt);
  const auto tile_box = tiles.TileBounds(tile_id);
  const int32_t nsubdivisions = tiles.nsubdivisions();
  const float row_delta = nsubdivisions / tile_box.Width();
  const float col_delta = nsubdivisions / tile_box.Height();

  int32_t sub_row = (pt.x() - tile_box.minx()) * row_delta;
  int32_t sub_col = (pt.y() - tile_box.miny()) * col_delta;

  return vm::AABB2<vm::PointLL>(
    tile_box.minx() + sub_row * row_delta,
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
std::vector<vm::AABB2<vm::PointLL> > expand_bbox_across_boundaries(
  const vm::AABB2<vm::PointLL> &bbox,
  const vm::Tiles<vm::PointLL> &tiles) {

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

  std::vector<vm::AABB2<vm::PointLL> > boxes;

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
std::unordered_map<int32_t, std::unordered_set<uint16_t> >
merge_intersections(
  const std::vector<vm::AABB2<vm::PointLL> > &boxes,
  const vm::Tiles<vm::PointLL> &tiles) {

  std::unordered_map<int32_t, std::unordered_set<uint16_t> > result;

  for (const auto &box : boxes) {
    auto intersection = tiles.Intersect(box);

    for (const auto &entry : intersection) {
      result[entry.first].insert(entry.second.begin(), entry.second.end());
    }
  }

  return result;
}

} // anonymous namespace

namespace valhalla {
namespace loki {

std::vector<baldr::GraphId>
nodes_in_bbox(const vm::AABB2<vm::PointLL> &bbox, baldr::GraphReader& reader) {
  std::vector<baldr::GraphId> nodes;

  auto hierarchy = reader.GetTileHierarchy();
  auto tiles = hierarchy.levels().rbegin()->second.tiles;
  const uint8_t bin_level = hierarchy.levels().rbegin()->second.level;

  // if the bbox only touches the edge of the tile or bin, then we need to
  // include neighbouring bins as well, in case both the edge and its opposite
  // were tie-broken into a bin which doesn't intersect the original bbox.
  auto expanded_bboxes = expand_bbox_across_boundaries(bbox, tiles);
  auto intersections = merge_intersections(expanded_bboxes, tiles);

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
    vb::GraphId tile_id(entry.first, bin_level, 0);
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

  // sort and loop over opposite_edges, adding them to nodes. this is to pick up
  // on any nodes which only have edges in the bin which leave the tile. whether
  // this edge is in the tile or in the neighbouring one.
  std::sort(opposite_edges.begin(), opposite_edges.end());
  for (const auto &entry : opposite_edges) {
    vb::GraphId node_id = entry.first;
    uint32_t opp_index = entry.second;

    const auto &node = cache(node_id).node(node_id);
    assert(opp_index < node.edge_count());
    vb::GraphId opp_edge_id = node_id + uint64_t(opp_index);
    const auto startnode = cache(opp_edge_id).edge(opp_edge_id).endnode();
    nodes.push_back(startnode);
  }

  // don't need opposite_edges any more
  opposite_edges.clear();

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
