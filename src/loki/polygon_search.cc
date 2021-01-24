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

std::set<vb::GraphId> edges_in_rings(const std::vector<ring_bg_t>& rings,
                                     baldr::GraphReader& reader,
                                     const std::shared_ptr<sif::DynamicCost>& costing) {

  // Get the lowest level and tiles
  const auto tiles = vb::TileHierarchy::levels().back().tiles;
  const auto bin_level = vb::TileHierarchy::levels().back().level;

  std::set<vb::GraphId> result_ids;

  for (const auto& ring : rings) {
    // first pull out all bins which intersect the ring (no within/contains!)
    // TODO: fill this container for all rings before proceeding, makes sure no duplicate bins are
    // iterated
    auto line_intersected = tiles.Intersect(ring);

    // get the tile ids in a vector and sort
    // TODO: find tiles & bins which are ENTIRELY inside the ring, should be pure arithmetics
    std::vector<int32_t> tile_keys;
    tile_keys.reserve(line_intersected.size());
    std::for_each(line_intersected.cbegin(), line_intersected.cend(),
                  [&tile_keys](const std::pair<int32_t, std::unordered_set<unsigned short>>& e) {
                    tile_keys.push_back(e.first);
                  });
    std::sort(tile_keys.begin(), tile_keys.end());
    for (auto& tile_id : tile_keys) {
      // int32_t row, col;
      // std::tie(row, col) = tiles.GetRowColumn(tile_id);
      auto tile = reader.GetGraphTile({static_cast<uint32_t>(tile_id), bin_level, 0});
      if (!tile) {
        continue;
      }
      for (auto bin_id : line_intersected[tile_id]) {
        for (auto edge_id : tile->GetBin(bin_id)) {
          // weed out duplicates early on
          // TODO: don't iterate over the same bin twice, much more efficient
          if (result_ids.find(edge_id) != result_ids.end()) {
            continue;
          }
          if (edge_id.Tile_Base() != tile->header()->graphid().Tile_Base() &&
              !reader.GetGraphTile(edge_id, tile)) {
            continue;
          }
          const auto edge = tile->directededge(edge_id);
          // grab the opposing
          auto opp_tile = tile;
          const baldr::DirectedEdge* opp_edge = nullptr;
          baldr::GraphId opp_id;

          // bail if we wouldnt be allowed on this edge anyway (or its opposing)
          if (!costing->Allowed(edge, tile) &&
              (!(opp_id = reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile)).Is_Valid() ||
               !costing->Allowed(opp_edge, opp_tile))) {
            continue;
          }
          auto edge_info = tile->edgeinfo(edge->edgeinfo_offset());
          auto name = edge_info.GetNames()[0];
          bool intersects = bg::intersects(ring, edge_info.shape());
          if (intersects) {
            result_ids.emplace(edge_id);
            result_ids.emplace(
                opp_id.Is_Valid() ? opp_id : reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile));
          }
        }
      }
    }
  }

  return result_ids;
}

ring_bg_t PBFToRing(const Options::Polygon& ring_pbf) {
  ring_bg_t new_ring;
  for (const auto& coord : ring_pbf.coords()) {
    new_ring.push_back({coord.ll().lng(), coord.ll().lat()});
  }
  // corrects geometry and handedness as expected by bg
  bg::correct(new_ring);
  return new_ring;
}

double GetRingLength(const ring_bg_t& ring) {
  line_bg_t line{ring.begin(), ring.end()};
  auto length = bg::length(line, Haversine());
  return length;
}
} // namespace loki
} // namespace valhalla
