#include "midgard/point_tile_index.h"

#include <list>

namespace valhalla {

namespace midgard {

// A "special" value that means "this point is deleted". An invalid
// lat/lon.
const PointLL PointTileIndex::kDeletedPoint = {1000.0, 1000.0};

template <class container_t>
PointTileIndex::PointTileIndex(double tile_width_degrees, const container_t& polyline) {
  if (polyline.size() == 0)
    return;
  if (tile_width_degrees <= 0.0)
    return;

  // Determine the extents of the points we want to index. This determines
  // the size of our Tile space.
  double min_lat = 1000.0, max_lat = -1000.0;
  double min_lng = 1000.0, max_lng = -1000.0;
  for (auto iter = polyline.begin(); iter != polyline.end(); iter++) {
    const PointLL& p = *iter;
    if (p.lat() < min_lat)
      min_lat = p.lat();
    if (p.lat() > max_lat)
      max_lat = p.lat();
    if (p.lng() < min_lng)
      min_lng = p.lng();
    if (p.lng() > max_lng)
      max_lng = p.lng();
  }

  // We need a tile buffer around our tiled-space on every side, hence
  // the extra 2. This is because our spatial search will look query
  // every tile around the given tile we are searching and this prevents
  // us from wrapping at the tiled-space boundaries.
  constexpr int tile_buffer = 2;
  min_lat -= tile_buffer * tile_width_degrees;
  min_lng -= tile_buffer * tile_width_degrees;
  max_lat += 2 * tile_buffer * tile_width_degrees;
  max_lng += 2 * tile_buffer * tile_width_degrees;

  double deltax = max_lng - min_lng;
  double deltay = max_lat - min_lat;

  PointLL min_pt = {min_lng, min_lat};

  int32_t num_x_divs = std::ceil(deltax / tile_width_degrees);
  int32_t num_y_divs = std::ceil(deltay / tile_width_degrees);

  // A square shape
  int32_t num_divs = 2 * tile_buffer + std::max(num_y_divs, num_x_divs);

  // Ok full confession, I know how the TileId's are generated inside the Tile
  // class and they have an upper limit of int32_t. I'm going to cap how many
  // divisions can be made to avoid hitting that upper limit. Yes I know the
  // fix should be made inside the Tile class but that's a separate project.
  static const int32_t max_divs = std::floor(std::sqrt(std::numeric_limits<int32_t>::max()));
  num_divs = std::min(max_divs, num_divs);

  tiles = std::make_unique<Tiles<PointLL>>(min_pt, tile_width_degrees, num_divs, num_divs);

  this->points.reserve(polyline.size());
  tiled_space.reserve(polyline.size());
  size_t index = 0;
  for (auto iter = polyline.begin(); iter != polyline.end(); iter++, index++) {
    const PointLL& p = *iter;
    this->points.emplace_back(p);
    int32_t tid = tiles->TileId(p);
    tiled_space[tid].insert(index);
  }
}

std::unordered_set<size_t> PointTileIndex::get_points_near(const PointLL& pt) {
  return get_points_near_segment(LineSegment2<PointLL>(pt, pt));
}

std::unordered_set<size_t> PointTileIndex::get_points_near_segment(const LineSegment2<PointLL>& seg) {
  const PointLL& a = seg.a();
  const PointLL& b = seg.b();

  // We need to "stretch" our search area by one tile in all directions.
  // Stretch our min-point in the SW direction.
  double minx = std::min(a.x(), b.x());
  double miny = std::min(a.y(), b.y());
  PointLL minpt = {minx, miny};
  int32_t mintid = tiles->LeftNeighbor(tiles->BottomNeighbor(tiles->TileId(minpt)));
  AABB2<PointLL> mintidbox = tiles->TileBounds(mintid);

  // Stretch our max-point in the NE direction.
  double maxx = std::max(a.x(), b.x());
  double maxy = std::max(a.y(), b.y());
  PointLL maxpt = {maxx, maxy};
  int32_t maxtid = tiles->RightNeighbor(tiles->TopNeighbor(tiles->TileId(maxpt)));
  AABB2<PointLL> maxtidbox = tiles->TileBounds(maxtid);

  // Box from min-pt to max-pt. Determine the tiles covered by thebox.
  AABB2<PointLL> thebox = mintidbox;
  thebox.Expand(maxtidbox);
  std::vector<int32_t> tiles_covered = tiles->TileList(thebox);

  // Gather up all points in the tiles_covered.
  std::unordered_set<size_t> near_pts;
  for (int32_t tid : tiles_covered) {
    auto iter = tiled_space.find(tid);
    if (iter != tiled_space.end()) {
      near_pts.insert(iter->second.begin(), iter->second.end());
    }
  }

  return near_pts;
}

void PointTileIndex::remove_point(size_t idx) {
  // delete this entry from its tile
  auto iter = tiled_space.find(tiles->TileId(points[idx]));
  if (iter != tiled_space.end()) {
    std::unordered_set<size_t>& tile_points = iter->second;
    tile_points.erase(idx);
  }

  // don't actually delete from the vector, just mark as deleted
  points[idx] = PointTileIndex::kDeletedPoint;
}

// Explicit instantiation
template PointTileIndex::PointTileIndex(double, const std::vector<PointXY<float>>&);
template PointTileIndex::PointTileIndex(double, const std::vector<PointXY<double>>&);
template PointTileIndex::PointTileIndex(double, const std::list<PointXY<float>>&);
template PointTileIndex::PointTileIndex(double, const std::list<PointXY<double>>&);
template PointTileIndex::PointTileIndex(double, const std::vector<GeoPoint<float>>&);
template PointTileIndex::PointTileIndex(double, const std::vector<GeoPoint<double>>&);
template PointTileIndex::PointTileIndex(double, const std::list<GeoPoint<float>>&);
template PointTileIndex::PointTileIndex(double, const std::list<GeoPoint<double>>&);

} // namespace midgard
} // namespace valhalla
