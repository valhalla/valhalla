#include <numeric>

#include "loki/node_search.h"
#include "loki/polygon_search.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "proto/options.pb.h"

namespace bg = boost::geometry;
namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;

namespace {
namespace vl = valhalla::loki;

// check specific rings for intersection with a shape
template <typename geom>
bool intersects_rings(const geom& shape,
                      const std::vector<vl::ring_bg_t>& rings,
                      const std::vector<size_t>& ring_ids) {
  for (const auto& ring_loc : ring_ids) {
    if (bg::intersects(rings[ring_loc], shape)) {
      return true;
    };
  }
  return false;
}
template <typename geom>
bool intersects_rings(const geom& shape, const std::vector<vl::ring_bg_t>& rings) {
  std::vector<size_t> ring_ids(rings.size());
  std::iota(ring_ids.begin(), ring_ids.end(), 0);
  return intersects_rings(shape, rings, ring_ids);
}

void collect_bins_within(std::unordered_map<int32_t, vl::bins_collector>& bins_intersected,
                         const std::vector<vl::ring_bg_t>& rings,
                         const vm::Tiles<vm::PointLL>& tiles) {
  // collect all rows and their columns, both sorted
  std::map<int32_t, std::set<int32_t>> rowcols;
  const auto bin_center_offset = 0.5 * tiles.SubdivisionSize();
  // first collect all bins within intersected tiles
  for (const auto& tb : bins_intersected) {
    int32_t row, col;
    std::tie(row, col) = tiles.GetRowColumn(tb.first);
    rowcols[row].insert(col);
    auto tile_base_x = tiles.TileBounds().minx() + (col * tiles.TileSize());
    auto tile_base_y = tiles.TileBounds().miny() + (row * tiles.TileSize());
    for (auto bin_id = 0; bin_id <= vb::kBinCount; bin_id++) {
      // skip if it was intersected
      if (tb.second.find(bin_id) != tb.second.end()) {
        continue;
      }
      auto bin_center_x =
          tile_base_x + (tiles.SubdivisionSize() * static_cast<float>(bin_id % vb::kBinsDim));
      auto bin_center_y =
          tile_base_y + (tiles.SubdivisionSize() * static_cast<float>(bin_id / vb::kBinsDim));
      const auto bin_center =
          vm::PointLL(bin_center_x + bin_center_offset, bin_center_y + bin_center_offset);
      if (intersects_rings(bin_center, rings)) {
        bins_intersected[tb.first][bin_id].within = true;
      }
    }
  }
  // next loop through all rows and cols and collect the bins of the tiles within
  // for (const auto& rowcol : rowcols) {
  //
  //}
}

} // namespace

namespace valhalla {
namespace loki {

std::set<vb::GraphId> edges_in_rings(const std::vector<ring_bg_t>& rings,
                                     baldr::GraphReader& reader,
                                     const std::shared_ptr<sif::DynamicCost>& costing) {
  // Get the lowest level and tiles
  const auto tiles = vb::TileHierarchy::levels().back().tiles;
  const auto bin_level = vb::TileHierarchy::levels().back().level;

  // set up some containers to keep track
  std::unordered_map<int32_t, bins_collector> bins_intersected;
  std::set<vb::GraphId> result_ids;

  // first pull out all *unique* bins which intersect the rings
  for (size_t ring_idx = 0; ring_idx < rings.size(); ring_idx++) {
    auto ring = rings[ring_idx];
    auto line_intersected = tiles.Intersect(ring);
    for (auto& tb : line_intersected) {
      for (auto& b : tb.second) {
        bins_intersected[tb.first][b].ring_ids.push_back(ring_idx);
      }
    }
  }
  // then get the bins that are entirely contained
  // collect_bins_within(bins_intersected, rings, tiles);
  std::vector<int32_t> tile_keys;
  tile_keys.reserve(bins_intersected.size());
  std::for_each(bins_intersected.cbegin(), bins_intersected.cend(),
                [&tile_keys](const std::pair<int32_t, bins_collector>& e) {
                  tile_keys.push_back(e.first);
                });
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
        if (result_ids.find(edge_id) != result_ids.end()) {
          continue;
        }
        // TODO: optimize the tile changing all the time by enqueuing edges
        // from other tiles and process them after the big loop
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
        // no need to check edges in bins which are entirely inside a ring
        if (bin.second.within) {
          result_ids.emplace(edge_id);
          result_ids.emplace(opp_id);
          continue;
        }

        // down here only valid edges arrive which potentially intersect a ring
        // TODO: some logic to set percent_along for origin/destination edges
        // careful: polygon can intersect a single edge multiple times
        // and needs to be applied to bins which are entirely inside a ring as well
        auto edge_info = tile->edgeinfo(edge->edgeinfo_offset());
        bool intersects = false;
        for (const auto& ring_loc : bin.second.ring_ids) {
          intersects = bg::intersects(rings[ring_loc], edge_info.shape());
        }
        if (intersects) {
          result_ids.emplace(edge_id);
          result_ids.emplace(
              opp_id.Is_Valid() ? opp_id : reader.GetOpposingEdgeId(edge_id, opp_edge, opp_tile));
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
