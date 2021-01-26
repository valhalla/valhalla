#include <numeric>

#include "loki/node_search.h"
#include "loki/polygon_search.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "proto/options.pb.h"

namespace bg = boost::geometry;
namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;

namespace valhalla {
namespace loki {

std::unordered_set<vb::GraphId> edges_in_rings(const std::vector<ring_bg_t>& rings,
                                               baldr::GraphReader& reader,
                                               const std::shared_ptr<sif::DynamicCost>& costing) {
  // Get the lowest level and tiles
  const auto tiles = vb::TileHierarchy::levels().back().tiles;
  const auto bin_level = vb::TileHierarchy::levels().back().level;

  // keep track which tile's bins intersect which rings
  bins_collector bins_intersected;
  std::unordered_set<vb::GraphId> avoid_edge_ids;

  // first pull out all *unique* bins which intersect the rings
  for (size_t ring_idx = 0; ring_idx < rings.size(); ring_idx++) {
    auto ring = rings[ring_idx];
    auto line_intersected = tiles.Intersect(ring);
    for (const auto& tb : line_intersected) {
      for (const auto& b : tb.second) {
        bins_intersected[tb.first][b].push_back(ring_idx);
      }
    }
  }

  std::vector<int32_t> tile_keys(bins_intersected.size());
  std::for_each(bins_intersected.begin(), bins_intersected.end(),
                [&tile_keys](
                    const std::pair<int32_t, std::unordered_map<unsigned short, std::vector<size_t>>>&
                        e) { tile_keys.push_back(e.first); });
  std::sort(tile_keys.begin(), tile_keys.end());
  for (const auto& tile_id : tile_keys) {
    auto& tile_bins = bins_intersected[tile_id];
    auto tile = reader.GetGraphTile({static_cast<uint32_t>(tile_id), bin_level, 0});
    if (!tile) {
      continue;
    }
    for (const auto& bin : tile_bins) {
      // tile will be mutated most likely in the loop
      if (tile->id().tileid() != tile_id) {
        tile = reader.GetGraphTile({static_cast<uint32_t>(tile_id), bin_level, 0});
      }
      for (const auto& edge_id : tile->GetBin(bin.first)) {
        // weed out duplicates early on
        // TODO: is this really worth it?
        if (avoid_edge_ids.find(edge_id) != avoid_edge_ids.end()) {
          continue;
        }
        // TODO: optimize the tile switching by enqueuing edges
        // from other levels & tiles and process them after this big loop
        if (edge_id.Tile_Base() != tile->header()->graphid().Tile_Base() &&
            !reader.GetGraphTile(edge_id, tile)) {
          continue;
        }
        const auto edge = tile->directededge(edge_id);
        auto opp_tile = tile;
        const baldr::DirectedEdge* opp_edge = nullptr;
        baldr::GraphId opp_id;

        // bail if we wouldnt be allowed on this edge anyway (or its opposing)
        if (!costing->Allowed(edge, tile) &&
            (!(opp_id = reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile)).Is_Valid() ||
             !costing->Allowed(opp_edge, opp_tile))) {
          continue;
        }

        // TODO: some logic to set percent_along for origin/destination edges
        // careful: polygon can intersect a single edge multiple times
        // and needs to be applied to bins which are entirely inside a ring as well
        auto edge_info = tile->edgeinfo(edge->edgeinfo_offset());
        auto name = edge_info.GetNames()[0];
        bool intersects = false;
        for (const auto& ring_loc : bin.second) {
          intersects = bg::intersects(rings[ring_loc], edge_info.shape());
          if (intersects) {
            break;
          }
        }
        // TODO: insert PBF avoid edge right here instead of this?
        if (intersects) {
          avoid_edge_ids.emplace(edge_id);
          avoid_edge_ids.emplace(
              opp_id.Is_Valid() ? opp_id : reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile));
        }
      }
    }
  }

  return avoid_edge_ids;
}

ring_bg_t PBFToRing(const Options::Polygon& ring_pbf) {
  ring_bg_t new_ring;
  for (const auto& coord : ring_pbf.coords()) {
    new_ring.push_back({coord.ll().lng(), coord.ll().lat()});
  }
  // corrects geometry and handedness as expected by bg for rings
  bg::correct(new_ring);
  return new_ring;
}

double GetRingLength(const ring_bg_t& ring) {
  // bg doesn't (yet) support length of ring geoms
  line_bg_t line{ring.begin(), ring.end()};
  auto length = bg::length(line, Haversine());
  return length;
}
} // namespace loki
} // namespace valhalla
