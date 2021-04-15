#include "midgard/point_tile_index.h"

namespace valhalla {

namespace midgard {

// A "special" value that means "this point is deleted".
const PointLL PointTileIndex::deleted_point = {1000.0, 1000.0};

PointTileIndex::PointTileIndex(double tile_width_degrees) {
  assert(tile_width_degrees <= 180.0);
  assert(tile_width_degrees > 0.0);
  int level = 1;
  num_x_subdivisions = 1;
  double degrees_at_level = 180.0;
  constexpr int max_level = 32;
  while ((degrees_at_level > tile_width_degrees) && (level <= max_level)) {
    level++;
    degrees_at_level /= 2.0;
    num_x_subdivisions = num_x_subdivisions << 1;
  }

  // y's space is half that of x (because the full latitude range is
  // 180 deg vs longitude which is 360).
  num_y_subdivisions = num_x_subdivisions >> 1;
}

template <class container_t> void PointTileIndex::tile(const container_t& polyline) {
  this->points.reserve(polyline.size());
  tiled_space.reserve(polyline.size());
  size_t index = 0;
  for (auto iter = polyline.begin(); iter != polyline.end(); iter++, index++) {
    this->points.emplace_back(*iter);
    TileId pid = get_tile_id(*iter);
    auto map_iter = tiled_space.find(pid);
    if (map_iter == tiled_space.end())
      tiled_space.insert(std::pair<TileId, std::unordered_set<size_t>>{pid, {index}});
    else
      map_iter->second.insert(index);
  }
}

void PointTileIndex::get_points_near(const PointLL& pt, std::unordered_set<size_t>& points) {
  TileId pid = get_tile_id(pt);
  // TODO: handle under/overflow
  for (uint32_t x = pid.x - 1; x < pid.x + 1; x++) {
    for (uint32_t y = pid.y - 1; y < pid.y + 1; y++) {
      auto iter = tiled_space.find(TileId{x, y});
      if (iter != tiled_space.end()) {
        points.insert(iter->second.begin(), iter->second.end());
      }
    }
  }
}

void PointTileIndex::get_points_near_segment(const LineSegment2<PointLL>& seg,
                                             std::unordered_set<size_t>& points) {
  TileId pida = get_tile_id(seg.a());
  TileId pidb = get_tile_id(seg.b());
  uint32_t minx = std::min(pida.x, pidb.x);
  uint32_t maxx = std::max(pida.x, pidb.x);
  uint32_t miny = std::min(pida.y, pidb.y);
  uint32_t maxy = std::max(pida.y, pidb.y);
  // TODO: handle under/overflow
  // TODO: follow line instead of just using a box
  for (uint32_t x = minx - 1; x < maxx + 1; x++) {
    for (uint32_t y = miny - 1; y < maxy + 1; y++) {
      TileId tid{x, y};
      auto iter = tiled_space.find(tid);
      if (iter != tiled_space.end()) {
        points.insert(iter->second.begin(), iter->second.end());
      }
    }
  }
}

void PointTileIndex::remove_point(const size_t& idx) {
  // delete this entry from its tile
  auto iter = tiled_space.find(get_tile_id(points[idx]));
  if (iter != tiled_space.end()) {
    std::unordered_set<size_t>& pixel_points = iter->second;
    pixel_points.erase(idx);
  }

  // don't actually delete from the vector, just mark as deleted
  points[idx] = PointTileIndex::deleted_point;
}

// Explicit instantiation
template void PointTileIndex::tile(const std::vector<PointXY<float>>&);
template void PointTileIndex::tile(const std::vector<PointXY<double>>&);
template void PointTileIndex::tile(const std::list<PointXY<float>>&);
template void PointTileIndex::tile(const std::list<PointXY<double>>&);
template void PointTileIndex::tile(const std::vector<GeoPoint<float>>&);
template void PointTileIndex::tile(const std::vector<GeoPoint<double>>&);
template void PointTileIndex::tile(const std::list<GeoPoint<float>>&);
template void PointTileIndex::tile(const std::list<GeoPoint<double>>&);


} // namespace midgard
} // namespace valhalla
