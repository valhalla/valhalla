#include "loki/polygon_search.h"
#include "loki/node_search.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "proto/options.pb.h"

namespace bg = boost::geometry;
namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;

namespace {} // namespace

namespace valhalla {
namespace loki {

std::set<vb::GraphId> edges_in_rings(const multi_ring_t& rings, baldr::GraphReader& reader) {

  // Get the lowest level and tiles
  const auto tiles = vb::TileHierarchy::levels().back().tiles;
  const auto bin_level = vb::TileHierarchy::levels().back().level;

  // output container
  std::set<vb::GraphId> edges;

  for (auto ring : rings) {
    // first pull out all bins which intersect the ring (no within/contains!)
    auto line_intersected = tiles.Intersect(ring);

    // get the tile ids in a vector and sort
    // this might help to find tiles & bins which are ENTIRELY inside the ring
    std::vector<int32_t> tile_keys;
    tile_keys.reserve(line_intersected.size());
    std::for_each(line_intersected.cbegin(), line_intersected.cend(),
                  [&tile_keys](const std::pair<int32_t, std::unordered_set<unsigned short>>& e) {
                    tile_keys.push_back(e.first);
                  });
    std::sort(tile_keys.begin(), tile_keys.end());

    // std::cout << "No of intersected tiles: " << std::to_string(line_intersected.size()) <<
    // std::endl;
    for (auto& tile_id : tile_keys) {
      // int32_t row, col;
      // std::tie(row, col) = tiles.GetRowColumn(tile_id);

      // Problem: only intersects edges which are part of the tile
      auto tile = reader.GetGraphTile({static_cast<uint32_t>(tile_id), bin_level, 0});
      if (!tile) {
        continue;
      }
      for (auto bin_id : line_intersected[tile_id]) {
        for (auto edge_id : tile->GetBin(bin_id)) {
          // weed out duplicates early on
          // TODO: find a more efficient way to do this..
          if (edges.find(edge_id) != edges.end()) {
            continue;
          }
          // TODO: figure out how to best get the edges from other tiles
          // why the hell is this still failing the assert in tile->directededge()?!?!
          if (edge_id.Tile_Base() != tile->header()->graphid().Tile_Base()) {
            continue;
          }
          auto opp_edge_id = reader.GetOpposingEdgeId(edge_id, tile);
          const auto edge = tile->directededge(edge_id);
          if (!edge) {
            continue;
          }
          auto edge_info = tile->edgeinfo(edge->edgeinfo_offset());
          auto edge_shape = edge_info.shape();
          bool intersects = bg::intersects(ring, edge_shape);
          if (intersects) {
            edges.emplace(edge_id);
            edges.emplace(opp_edge_id);
          }
        }
      }
    }
  }

  return edges;
}

multi_ring_t PBFToRings(const google::protobuf::RepeatedPtrField<Options::AvoidPolygon>& rings_pbf) {
  multi_ring_t rings;
  for (const auto& ring_pbf : rings_pbf) {
    ring_bg_t new_ring;
    for (const auto& coord : ring_pbf.coords()) {
      new_ring.push_back({coord.ll().lng(), coord.ll().lat()});
    }
    // corrects geometry and handedness as expected
    bg::correct(new_ring);
    rings.push_back(new_ring);
  }
  return rings;
}

/*
double GetAvoidArea(const multi_ring_t& rings) {
  double area;
  for (const auto& ring : rings) {
    area += bg::area(ring, bg::strategy::area::geographic<>());
  }
  return area;
}
*/
float GetRingLength(const multi_ring_t& rings) {
  float length = 0;
  for (const auto& ring : rings) {
    length += bg::length(ring, Haversine());
  }
  return length;
}
} // namespace loki
} // namespace valhalla
