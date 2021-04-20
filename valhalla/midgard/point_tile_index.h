#pragma once

#include <list>
#include <unordered_set>

#include "midgard/linesegment2.h"
#include "midgard/pointll.h"

namespace valhalla {

namespace midgard {

/**
 * Indexes a given container_t of PointLL's into a tiled space.
 * (So yea, I'm using the word "tile" which is a heavily overloaded term.
 * Maybe "pixel" is a better term? Or "grid"? Or "discretized space"?)
 *
 * Basically, I've taken the lat/lon ranges for the earth and subdivided them
 * into smaller and smaller rectangles until we reach a rectangle size that
 * only just big enough to meet the given tile_width_degrees.
 *
 * Then, in a single pass in the ::tile() method, we compute each PointLL's
 * "TileId" - which is just a Cartesian coordinate into the tiled space.
 * We then place each PointLL into its appropriate TileId bin.
 *
 * Once all the points are binned we can quickly solve problems like "what
 * points are near me" - in effectively O(1) time (the speed of a hash
 * lookup).
 *
 * Of important note is the fact that to determine the "points within an
 * exact distance of a given point" you must round up all points in all
 * surrounding tiles. You can subsequently compute the distance bewteen your
 * point and each "nearby" point using a more accurate distance computation
 * of your choosing.
 */
class PointTileIndex {
  // A coordinate into our tile/grid space. Using uint32_t means we could
  // theoretically subdivide 360 degrees of latitude into 2^32 bins. Very
  // roughly, that's about 1e-8 degrees or 1 cm.
  struct TileId {
    uint32_t x, y;
    friend bool operator==(const TileId& L, const TileId& R) {
      return L.x == R.x && L.y == R.y;
    }
  };

  // prev/next "iterators" to ensure we stay on the tiled-space when
  // we might otherwise underflow/overflow.
  uint32_t prevx(uint32_t x) {
    return (x > 0) ? x - 1 : num_x_subdivisions - 1;
  }
  uint32_t nextx(uint32_t x) {
    return (x < num_x_subdivisions - 1) ? x + 1 : 0;
  }
  uint32_t prevy(uint32_t y) {
    return (y > 0) ? y - 1 : num_y_subdivisions - 1;
  }
  uint32_t nexty(uint32_t y) {
    return (y < num_y_subdivisions - 1) ? y + 1 : 0;
  }

  struct TileId_hash_functor {
    size_t operator()(const TileId& tid) const {
#if 0
      // In my tests this wasn't as fast as the simple implementation below.
      size_t seed = 0;
      valhalla::midgard::hash_combine(seed, tid.x);
      valhalla::midgard::hash_combine(seed, tid.y);
      return seed;
#endif
      // I'll concede this could be an awful way to hash in mock
      // scenarios. However, for real world data, its darn simple/fast.
      return tid.x + tid.y;
    }
  };

  // key: TileId
  // value: unordered_set of point indices that live in this tile
  std::unordered_map<TileId, std::unordered_set<size_t>, TileId_hash_functor> tiled_space;

  // How many sections the world's longitude (x) and latitude (y) are divided
  // to satisfy the given tile_width_degrees.
  uint32_t num_x_subdivisions = {0};
  uint32_t num_y_subdivisions = {0};

  static constexpr double max_x_range = 360.0; // -180 to 180
  static constexpr double max_y_range = 180.0; // -90 to 90

  inline TileId get_tile_id(const PointLL& pt) {
    TileId tid;
    tid.x = std::lround(num_x_subdivisions * ((pt.lng() + max_x_range / 2.0) / max_x_range));
    tid.y = std::lround(num_y_subdivisions * ((pt.lat() + max_y_range / 2.0) / max_y_range));
    return tid;
  }

public:
  // when points are deleted from our polyline they become this "special" value
  static const PointLL deleted_point;

  // need random access to every point
  std::vector<PointLL> points;

  // The given tile_width_degrees determines how the PointTileIndex will subdivide
  // space. All "near" queries will be based on this distance.
  PointTileIndex(double tile_width_degrees);

  // Walks every point in the polyline and bins each into the index.
  template <class container_t> void tile(const container_t& polyline);

  // Get all the points roughly within the "tile_width_degrees" of the given pt.
  // Some of the returned points could be as far as 2*tile_width_degrees from
  // your point. The caller can decide what type of distance calculation
  // to use to determine exact distances.
  void get_points_near(const PointLL& pt, std::unordered_set<size_t>& points);

  // Get all the points roughly within the "tile_width_degrees" of the given
  // segment. Some of the returned points could be as far as 2*tile_width_degrees
  // from your segment. The caller can decide what type of distance calculation
  // to use to determine exact distances.
  void get_points_near_segment(const LineSegment2<PointLL>& seg, std::unordered_set<size_t>& points);

  // Removes a point from our tiled space given its index.
  void remove_point(const size_t& idx);

  // Remove a range of points by index from the tiled space. This includes
  // the point from sidx (inclusive) to eidx (exclusive).
  void remove_points(const size_t& sidx, const size_t& eidx) {
    for (size_t i = sidx; i < eidx; i++) {
      remove_point(i);
    }
  }
};

} // namespace midgard
} // namespace valhalla