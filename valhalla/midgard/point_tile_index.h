#pragma once

#include <memory>
#include <unordered_set>

#include "midgard/linesegment2.h"
#include "midgard/pointll.h"
#include "midgard/tiles.h"

namespace valhalla {

namespace midgard {

/**
 * Provided a search width (tile_width_degrees) and a bunch of points,
 * this class will bin the points into a gridded/tiled space at that width.
 *
 * Once all the points are binned we can quickly solve problems like "what
 * points are near me" - in effectively O(1) time (the speed of a hash
 * lookup).
 *
 * Of important note is the fact that to determine the "points within an
 * exact distance of a given point" you must 1) use the get_points_near*()
 * methods to round-up nearby points, then 2) compute the distance those
 * nearby points using a more accurate distance computation of your choosing.
 */
class PointTileIndex {
  // This guy can tell us which tile a points belongs to.
  std::unique_ptr<Tiles<PointLL>> tiles;

  // key: TileId (uint32_t)
  // value: unordered_set of point indices that live in this tile
  std::unordered_map<uint32_t, std::unordered_set<size_t>> tiled_space;

public:
  // The given tile_width_degrees determines how the PointTileIndex will subdivide
  // space. All "near" queries will be based on this distance. The given polyline
  // points will be binned/indexed into our tiled space.
  template <class container_t> PointTileIndex(double tile_width_degrees, const container_t& polyline);

  // Get all the points roughly within the "tile_width_degrees" of the given pt.
  // Some of the returned points could be as far as 2*tile_width_degrees from
  // your point. The caller can decide what type of distance calculation
  // to use to determine exact distances.
  std::unordered_set<size_t> get_points_near(const PointLL& pt);

  // Get all the points roughly within the "tile_width_degrees" of the given
  // segment. Some of the returned points could be as far as 2*tile_width_degrees
  // from your segment. The caller can decide what type of distance calculation
  // to use to determine exact distances.
  std::unordered_set<size_t> get_points_near_segment(const LineSegment2<PointLL>& seg);

  // Removes a point from our tiled space given its index.
  void remove_point(size_t idx);

  // Remove a range of points by index from the tiled space. This includes
  // the point from start_index (inclusive) to end_index (exclusive).
  void remove_points(const size_t& start_index, const size_t& end_index) {
    for (size_t i = start_index; i < end_index; i++) {
      remove_point(i);
    }
  }

  // need random access to every point
  std::vector<PointLL> points;

  // when points are deleted from our polyline they become this "special" value
  static const PointLL kDeletedPoint;
};

} // namespace midgard
} // namespace valhalla