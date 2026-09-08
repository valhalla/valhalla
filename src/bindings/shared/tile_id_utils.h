#ifndef VALHALLA_BINDINGS_TILE_ID_UTILS_H
#define VALHALLA_BINDINGS_TILE_ID_UTILS_H

#include "baldr/graphid.h"
#include "baldr/tilehierarchy.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

#include <algorithm>
#include <ranges>
#include <stdexcept>
#include <string>
#include <vector>

namespace valhalla::bindings {

namespace vb = valhalla::baldr;
namespace vm = valhalla::midgard;

// road levels + transit
inline const uint32_t MAX_LEVEL = vb::TileHierarchy::levels().back().level + 1;

inline void check_level(const uint32_t level) {
  if (level >= MAX_LEVEL) {
    throw std::invalid_argument("We only support " + std::to_string(MAX_LEVEL) +
                                " hierarchy levels.");
  }
}

inline void check_coord(const double minx, const double miny, const double maxx, const double maxy) {
  if (minx < -180. || maxx > 180. || miny < -90. || maxy > 90.) {
    throw std::invalid_argument("Invalid coordinate, remember it's (lon, lat)");
  }
}

inline std::vector<uint32_t> default_levels() {
  return {0, 1, 2};
}

/**
 * Given a polygon ring (as PointLL coords) and hierarchy levels, return all tile GraphIds
 * whose tiles are inside or intersect the ring.
 *
 * @param ring  The polygon ring coordinates (lon, lat). Will be closed and oriented automatically.
 * @param levels  Tile hierarchy levels to query. Empty means all road levels (0, 1, 2).
 * @return  Vector of tile-base GraphIds.
 */
inline std::vector<vb::GraphId> get_tile_ids_from_ring(std::vector<vm::PointLL> ring,
                                                       std::vector<uint32_t> levels) {
  if (ring.size() < 3) {
    throw std::invalid_argument("Ring must have at least 3 coordinates");
  }

  if (levels.empty()) {
    levels = default_levels();
  }

  // close open rings
  if (ring.front().lng() != ring.back().lng() || ring.front().lat() != ring.back().lat()) {
    ring.push_back(ring.front());
  }
  // reverse if counter-clockwise
  if (vm::polygon_area(ring) > 0) {
    std::reverse(ring.begin(), ring.end());
  }

  // point-in-polygon test using ray casting (to the right from pt)
  auto point_in_ring = [&ring](const vm::PointLL& test_pt) {
    bool inside = false;
    // first connect the last with the first point, then walk through the segments of the ring
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
      const auto& segment_start = ring[j];
      const auto& segment_end = ring[i];

      // does this segment cross the ray's latitude?
      bool crossing_lat =
          (segment_end.lat() > test_pt.lat()) != (segment_start.lat() > test_pt.lat());
      if (!crossing_lat) {
        continue;
      }

      // longitude where the segment crosses the ray's latitude
      double crossing_lng = segment_start.lng() + (segment_end.lng() - segment_start.lng()) *
                                                      (test_pt.lat() - segment_start.lat()) /
                                                      (segment_end.lat() - segment_start.lat());
      // does the crossing point fall to the right of the point, flip inside state if so
      if (test_pt.lng() < crossing_lng) {
        inside = !inside;
      }
    }
    // if the number of ray-polygon_edges crossings is odd, the point is inside the ring
    return inside;
  };

  std::vector<vb::GraphId> result;
  for (const auto level : levels) {
    check_level(level);
    const auto& tiles = vb::TileHierarchy::get_tiling(static_cast<uint8_t>(level));

    // find tiles intersecting (crossing) the ring
    auto intersected = tiles.Intersect(ring);
    std::vector<int32_t> boundary_tiles;
    boundary_tiles.reserve(intersected.size());
    result.reserve(result.size() + intersected.size());
    for (const auto& [tile_id, bins] : intersected) {
      boundary_tiles.push_back(tile_id);
      result.push_back(vb::GraphId{static_cast<uint32_t>(tile_id), level, 0});
    }

    // sort the tiles according to their index, so we can easily traverse them
    // row by row
    std::sort(boundary_tiles.begin(), boundary_tiles.end());

    // start at the lower-left tile & row and walk the grid to collect inner tiles
    auto curr_tile = boundary_tiles.begin();
    auto curr_row = *curr_tile / tiles.ncolumns();
    curr_tile++;
    for (; curr_tile != boundary_tiles.end(); ++curr_tile) {
      // last tile must a boundary tile, breaking also makes sure we have a next tile to look at
      if (*curr_tile == boundary_tiles.back()) {
        break;
      }

      const auto next_tile = *(curr_tile + 1);

      // we're about to move to the next row, reset and do it
      if (const auto next_row = next_tile / tiles.ncolumns(); next_row > curr_row) {
        curr_row = next_row;
        continue;
      }

      const auto col_distance = (next_tile % tiles.ncolumns()) - (*curr_tile % tiles.ncolumns());
      // the next tile is also a boundary tile
      if (col_distance == 1) {
        continue;
      }
      // skip gaps that are outside the polygon (handles concave shapes like U/W)
      if (!point_in_ring(tiles.Center(*curr_tile + col_distance / 2))) {
        continue;
      }
      // in the same row, walk through the columns and add inner tiles
      result.reserve(result.size() + col_distance - 1);
      for (auto const add_col : std::views::iota(1, col_distance)) {
        result.emplace_back(*curr_tile + add_col, level, 0);
      }
    }
  }

  return result;
}

} // namespace valhalla::bindings

#endif // VALHALLA_BINDINGS_TILE_ID_UTILS_H
