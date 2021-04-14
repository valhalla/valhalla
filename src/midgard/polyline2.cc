#include "midgard/polyline2.h"
#include "midgard/distanceapproximator.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

#include <iostream>
#include <list>
#include <unistd.h>

namespace valhalla {
namespace midgard {

/**
 * Finds the length of the polyline by accumulating the length of all
 * segments.
 * @return    Returns the length of the polyline.
 */
template <typename coord_t> typename coord_t::value_type Polyline2<coord_t>::Length() const {
  typename coord_t::value_type length = 0;
  if (pts_.size() < 2) {
    return length;
  }
  for (auto p = std::next(pts_.cbegin()); p != pts_.cend(); ++p) {
    length += std::prev(p)->Distance(*p);
  }
  return length;
}

/**
 * Compute the length of the specified polyline.
 * @param   pts  Polyline vertices.
 * @return  Returns the length of the polyline.
 */
template <typename coord_t>
template <class container_t>
typename coord_t::value_type Polyline2<coord_t>::Length(const container_t& pts) {
  typename coord_t::value_type length = 0;
  if (pts.size() < 2) {
    return length;
  }
  for (auto p = std::next(pts.cbegin()); p != pts.cend(); ++p) {
    length += std::prev(p)->Distance(*p);
  }
  return length;
}

template <typename coord_t> std::list<coord_t> Polyline2<coord_t>::GetSelfIntersections() {
  std::list<coord_t> intersections;
  std::vector<coord_t>& points = pts_;
  for (size_t i = 1; i < points.size() - 2; i++) {
    const coord_t& ia(points[i - 1]);
    const coord_t& ib(points[i]);
    for (size_t j = i + 2; j < points.size() - 1; j++) {
      const coord_t& ja(points[j - 1]);
      const coord_t& jb(points[j]);
      LineSegment2<coord_t> segmenti(ia, ib);
      LineSegment2<coord_t> segmentj(ja, jb);
      coord_t intersection_point;
      if (segmenti.Intersect(segmentj, intersection_point)) {
        intersections.emplace_back(std::move(intersection_point));
      }
    }
  }

  return intersections;
}

struct TileId {
  unsigned int x, y;
  friend bool operator==(const TileId& L, const TileId& R) {
    return L.x == R.x && L.y == R.y;
  }
};

struct TileId_hash_functor {
  size_t operator()(const TileId& tid) const {
#if 0
    // In my tests this wasn't quite as fast as the simple impl below.
    size_t seed = 0;
    valhalla::midgard::hash_combine(seed, tid.x);
    valhalla::midgard::hash_combine(seed, tid.y);
    return seed;
#endif
    // I'll concede this could be an awful way to hash in mock
    // scenarios. However, for real world data, its darn simple/fast.
    return tid.x;
  }
};

// Use the barycentric technique to test if the point p
// is inside the triangle formed by (a, b, c).
template <typename coord_t>
bool triangle_contains(const coord_t& a, const coord_t& b, const coord_t& c, const coord_t& p) {
  double v0x = c.x() - a.x();
  double v0y = c.y() - a.y();
  double v1x = b.x() - a.x();
  double v1y = b.y() - a.y();
  double v2x = p.x() - a.x();
  double v2y = p.y() - a.y();

  double dot00 = v0x * v0x + v0y * v0y;
  double dot01 = v0x * v1x + v0y * v1y;
  double dot02 = v0x * v2x + v0y * v2y;
  double dot11 = v1x * v1x + v1y * v1y;
  double dot12 = v1x * v2x + v1y * v2y;

  double denom = dot00 * dot11 - dot01 * dot01;

  // Triangle with very small area, e.g., nearly a line.
  if (std::fabs(denom) < 1e-20)
    return false;

  double u = (dot11 * dot02 - dot01 * dot12) / denom;
  double v = (dot00 * dot12 - dot01 * dot02) / denom;

  // Check if point is in triangle
  return (u >= 0) && (v >= 0) && (u + v < 1);
}

/**
 * Indexes a given container_t of PointLL's into a tiled space.
 * (So yea, I'm using the word "tile" which is a heavily overloaded term.
 * Maybe "pixel" is a better term? Or "discretized space"?)
 *
 * Basically, I've taken the lat/lon ranges for the earth and subdivided them
 * into smaller and smaller rectangles until the rectangle is only just big
 * enough to meet the given tile_width_degrees.
 *
 * Then, in a single pass in the ::tile() method (O(n)), we compute each
 * PointLL's TileId - which is just a Cartesian coordinate into the tiled space.
 * We then place each PointLL into its appropriate TileId bin.
 *
 * Once all the points are binned we can quickly solve problems like "what
 * points are near me" - in effectively O(1) time (the speed of a hash
 * lookup).
 *
 * Of important note is the fact that to determine the "points nearest a
 * given point" you have to round up all points in all surrounding tiles.
 * You can subsequently compute the distance bewteen your point and each
 * "nearby" point using a more accurate distance computation.
 */
class PointTiler {
  // key: TileId
  // value: unordered_set of point indices that live in this tile
  std::unordered_map<TileId, std::unordered_set<size_t>, TileId_hash_functor> tiled_space;

  // How many sections the world's longitude (x) and latitude (y) are divided
  // to satisfy the given tile_width_degrees.
  unsigned int num_x_subdivisions;
  unsigned int num_y_subdivisions;

  inline TileId get_tile_id(const PointLL& pt) {
    TileId tid;
    tid.x = std::lround(num_x_subdivisions * ((pt.lng() + 180.0) / 360.0));
    tid.y = std::lround(num_y_subdivisions * ((pt.lat() + 90.0) / 180.0));
    return tid;
  }

public:
  // when points are deleted from our polyline they become this "special" value
  static const PointLL deleted_point;

  // need random access to every point
  std::vector<PointLL> polyline;

  PointTiler(double tile_width_degrees);

  template <class container_t> void tile(container_t& polyline);

  void get_points_near(const PointLL& pt, std::unordered_set<size_t>& points);

  void get_points_in_and_around_triangle(const PointLL& pta,
                                         const PointLL& ptb,
                                         const PointLL& ptc,
                                         std::unordered_set<size_t>& points);

  void get_points_along(const PointLL& pta,
                        const PointLL& ptb,
                        std::unordered_set<size_t>& points);

#if 0
  void get_points_along_line(const PointLL& pta,
                             const PointLL& ptb,
                             std::unordered_set<size_t>& near_pts);
#endif

  void remove_point(const size_t& idx);

  // Removes points from the tile-index. This includes the points from sidx up to
  // eidx but not including eidx.
  void remove_points(const size_t& sidx, const size_t& eidx) {
    for (size_t i = sidx; i < eidx; i++) {
      remove_point(i);
    }
  }
};

// A "special" value that means "this point is deleted".
const PointLL PointTiler::deleted_point = {1000.0, 1000.0};

PointTiler::PointTiler(double tile_width_degrees) {
  assert(tile_width_degrees <= 180.0);
  assert(tile_width_degrees > 0.0);
  int level = 1;
  num_x_subdivisions = 1;
  double degrees_at_level = 180.0;
  constexpr double max_level = 32;
  while ((degrees_at_level > tile_width_degrees) && (level <= max_level)) {
    level++;
    degrees_at_level /= 2.0;
    num_x_subdivisions = num_x_subdivisions << 1;
  }

  // y's space is half that of x
  num_y_subdivisions = num_x_subdivisions >> 1;
}

template <class container_t> void PointTiler::tile(container_t& polyline) {
  this->polyline.reserve(polyline.size());
  tiled_space.reserve(polyline.size());
  size_t index = 0;
  for (auto iter = polyline.begin(); iter != polyline.end(); iter++, index++) {
    this->polyline.emplace_back(*iter);
    TileId pid = get_tile_id(*iter);
    auto map_iter = tiled_space.find(pid);
    if (map_iter == tiled_space.end())
      tiled_space.insert(std::pair<TileId, std::unordered_set<size_t>>{pid, {index}});
    else
      map_iter->second.insert(index);
  }
}

void PointTiler::get_points_near(const PointLL& pt, std::unordered_set<size_t>& points) {
  TileId pid = get_tile_id(pt);
  // TODO: handle under/overflow
  for (unsigned int x = pid.x - 1; x < pid.x + 1; x++) {
    for (unsigned int y = pid.y - 1; y < pid.y + 1; y++) {
      auto iter = tiled_space.find(TileId{x, y});
      if (iter != tiled_space.end()) {
        points.insert(iter->second.begin(), iter->second.end());
      }
    }
  }
}

void PointTiler::get_points_along(const PointLL& pta,
                                  const PointLL& ptb,
                                  std::unordered_set<size_t>& points) {
  TileId pida = get_tile_id(pta);
  TileId pidb = get_tile_id(ptb);
  unsigned int minx = std::min(pida.x, pidb.x);
  unsigned int maxx = std::max(pida.x, pidb.x);
  unsigned int miny = std::min(pida.y, pidb.y);
  unsigned int maxy = std::max(pida.y, pidb.y);
  // TODO: handle under/overflow
  // TODO: follow line instead of just using a box
  for (unsigned int x = minx - 1; x < maxx + 1; x++) {
    for (unsigned int y = miny - 1; y < maxy + 1; y++) {
      TileId tid{x, y};
      auto iter = tiled_space.find(tid);
      if (iter != tiled_space.end()) {
        points.insert(iter->second.begin(), iter->second.end());
      }
    }
  }
}

void PointTiler::remove_point(const size_t& idx) {
  // delete this entry from its tile
  auto iter = tiled_space.find(get_tile_id(polyline[idx]));
  if (iter != tiled_space.end()) {
    std::unordered_set<size_t>& pixel_points = iter->second;
    pixel_points.erase(idx);
  }

  // don't actually delete from the vector, just mark as deleted
  polyline[idx] = PointTiler::deleted_point;
}

template <class container_t>
void peucker_avoid_self_intersections(PointTiler& point_tile_index,
                                      const double& epsilon_sq,
                                      const std::unordered_set<size_t>& exclusions,
                                      size_t sidx,
                                      size_t eidx) {

  while ((exclusions.find(sidx) != exclusions.end()) && (sidx < eidx)) {
    sidx++;
  }
  while ((exclusions.find(eidx) != exclusions.end()) && (eidx > sidx)) {
    eidx--;
  }
  if (sidx >= eidx)
    return;

  const PointLL& start = point_tile_index.polyline[sidx];
  const PointLL& end = point_tile_index.polyline[eidx];

  double dmax = std::numeric_limits<double>::lowest();
  LineSegment2<PointLL> line_segment{start, end};

  // hfidx represents the index of the highest freq detail (the dividing point)
  size_t hfidx = sidx;

  // find the point furthest from the line-segment formed by {start, end}
  PointLL tmp;
  for (size_t idx = sidx + 1; idx < eidx; idx++) {
    // special points we dont want to generalize no matter what take precedence
    if (exclusions.find(idx) != exclusions.end()) {
      dmax = epsilon_sq;
      hfidx = idx;
      break;
    }

    const PointLL& c = point_tile_index.polyline[idx];

    // test if this is the highest frequency detail so far
    auto d = line_segment.DistanceSquared(c, tmp);
    if (d > dmax) {
      dmax = d;
      hfidx = idx;
    }
  }

  // if (dmax < epsilon_sq) then we have a relatively straight line between (start,end).
  // However, it can occur that if we were to blindly simplify this polyline we might create
  // a self-intersection. To test for that possibility, picture N triangles formed by
  // start, end and every polyline point between. Then test if any points reside in each
  // triangle. It is reasonable that some of the points along (start,end) will be inside
  // the triangle, ignore those. If any points remain, we cannot simplify this line or we'd
  // otherwise create a self-intersection.
  if (dmax < epsilon_sq) {

    //
    std::unordered_set<size_t> triangle_points;
    point_tile_index.get_points_between(start, end, triangle_points);

    bool can_simplify = true;
    for (size_t cidx = sidx + 1; (cidx < eidx) && can_simplify; cidx++) {
      const PointLL& c = point_tile_index.polyline[cidx];

      // Use our tiled search space to determine which points are in-and-around the
      // triangle formed by (start, c, end).
//      triangle_points.clear();
//      point_tile_index.get_points_in_and_around_triangle(start, c, end, triangle_points);

      // Using a tiled space to determine the points in-and-around the triangle is
      // coarse and will contain points both inside and outside the triangle. Now
      // use an exactly geometry check to determine which points are exactly inside
      // the triangle (start, c, end).
      for (size_t point_idx : triangle_points) {
        if (!triangle_contains(start, c, end, point_tile_index.polyline[point_idx])) {
          triangle_points.erase(point_idx);
        }
      }

      // Disregard points that are along the polyline [start,end].
      for (size_t i = sidx; i <= eidx; i++) {
        triangle_points.erase(i);
      }

      // If no points remain within the triangle, we can simplify this line
      can_simplify = triangle_points.empty();
    }

    if (can_simplify) {
      // Simplify the polyline by removing all points between sidx and eidx
      // from the point-tile-index (but don't remove sidx or eidx).
      point_tile_index.remove_points(sidx + 1, eidx);
    } else {
      // Simplifying this polyline would result in a self-intersection, so
      // we cannot. Force recursion around hfidx.
      dmax = epsilon_sq;
    }
  }

  // there are some high frequency details between start and end
  // so we need to look for flatter sections between them
  if (dmax >= epsilon_sq) {
    // we recurse from right to left for two reasons:
    // 1. we want to preserve iterator validity in the vector version
    // 2. its the only way to preserve the indices in the keep set
    if (eidx - hfidx > 1)
      peucker_avoid_self_intersections<container_t>(point_tile_index, epsilon_sq, exclusions, hfidx,
                                                    eidx);
    if (hfidx - sidx > 1)
      peucker_avoid_self_intersections<container_t>(point_tile_index, epsilon_sq, exclusions, sidx,
                                                    hfidx);
  }
}

template <class container_t>
void Generalize_orig(container_t& polyline,
                     double epsilon,
                     const std::unordered_set<size_t>& exclusions) {
  // any epsilon this low will have no effect on the input nor will any super short input
  if (epsilon <= 0 || polyline.size() < 3)
    return;

  // the recursive bit
  epsilon *= epsilon;
  std::function<void(typename container_t::iterator, size_t, typename container_t::iterator, size_t)>
      peucker;
  peucker = [&peucker, &polyline, epsilon, &exclusions](typename container_t::iterator start,
                                                        size_t s, typename container_t::iterator end,
                                                        size_t e) {
    // find the point furthest from the line
    double dmax = std::numeric_limits<double>::lowest();
    typename container_t::iterator itr;
    LineSegment2<PointLL> l{*start, *end};
    size_t j = e - 1, k;
    PointLL tmp;
    for (auto i = std::prev(end); i != start; --i, --j) {
      // special points we dont want to generalize no matter what take precidence
      if (exclusions.find(j) != exclusions.end()) {
        itr = i;
        dmax = epsilon;
        k = j;
        break;
      }

      // if this is the highest frequency detail so far
      auto d = l.DistanceSquared(*i, tmp);
      if (d > dmax) {
        itr = i;
        dmax = d;
        k = j;
      }
    }

    // there are some high frequency details between start and end
    // so we need to look for flatter sections between them
    if (dmax >= epsilon) {
      // we recurse from right to left for two reasons:
      // 1. we want to preserve iterator validity in the vector version
      // 2. its the only way to preserve the indices in the keep set
      if (e - k > 1)
        peucker(itr, k, end, e);
      if (k - s > 1)
        peucker(start, s, itr, k);
    } // nothing sticks out between start and end so simplify everything between away
    else
      polyline.erase(std::next(start), end);
  };

  // recurse!
  peucker(polyline.begin(), 0, std::prev(polyline.end()), polyline.size() - 1);
}

long generalize_time;

/**
 * Generalize the given list of points
 *
 * @param polyline    the list of points
 * @param epsilon     the tolerance used in removing points
 * @param exclusions  list of indices of points not to generalize
 */
template <typename coord_t>
template <class container_t>
void Polyline2<coord_t>::Generalize(container_t& polyline,
                                    typename coord_t::value_type epsilon,
                                    const std::unordered_set<size_t>& exclusions) {
  // any epsilon this low will have no effect on the input nor will any super short input
  if (epsilon <= 0 || polyline.size() < 3)
    return;

  auto start_time = std::chrono::high_resolution_clock::now();

  // Create a tiled-space spatial-index which allows us to conduct a peucker style
  // line simplification that avoids creating self-intersections within the polyline/polygon.
  const PointLL& first_point = *polyline.begin();
  double meters_per_deg = DistanceApproximator<PointLL>::MetersPerLngDegree(first_point.lat());
  double epsilon_in_deg = epsilon / meters_per_deg;
  PointTiler point_tile_index(epsilon_in_deg);
  point_tile_index.tile<container_t>(polyline);

  peucker_avoid_self_intersections<container_t>(point_tile_index, epsilon * epsilon, exclusions, 0,
                                                polyline.size() - 1);

  // copy the simplified polyline into 'polyline'
  polyline.clear();
  for (const auto& pt : point_tile_index.polyline) {
    if (pt != PointTiler::deleted_point) {
      polyline.push_back(pt);
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  generalize_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
}

// Explicit instantiation
template class Polyline2<PointXY<float>>;
template class Polyline2<PointXY<double>>;
template class Polyline2<GeoPoint<float>>;
template class Polyline2<GeoPoint<double>>;

template float Polyline2<PointXY<float>>::Length(const std::vector<PointXY<float>>&);
template double Polyline2<PointXY<double>>::Length(const std::vector<PointXY<double>>&);
template float Polyline2<PointXY<float>>::Length(const std::list<PointXY<float>>&);
template double Polyline2<PointXY<double>>::Length(const std::list<PointXY<double>>&);
template float Polyline2<GeoPoint<float>>::Length(const std::vector<GeoPoint<float>>&);
template double Polyline2<GeoPoint<double>>::Length(const std::vector<GeoPoint<double>>&);
template float Polyline2<GeoPoint<float>>::Length(const std::list<GeoPoint<float>>&);
template double Polyline2<GeoPoint<double>>::Length(const std::list<GeoPoint<double>>&);

template void Polyline2<PointXY<float>>::Generalize(std::vector<PointXY<float>>&,
                                                    float,
                                                    const std::unordered_set<size_t>&);
template void Polyline2<PointXY<double>>::Generalize(std::vector<PointXY<double>>&,
                                                     double,
                                                     const std::unordered_set<size_t>&);
template void Polyline2<PointXY<float>>::Generalize(std::list<PointXY<float>>&,
                                                    float,
                                                    const std::unordered_set<size_t>&);
template void Polyline2<PointXY<double>>::Generalize(std::list<PointXY<double>>&,
                                                     double,
                                                     const std::unordered_set<size_t>&);
template void Polyline2<GeoPoint<float>>::Generalize(std::vector<GeoPoint<float>>&,
                                                     float,
                                                     const std::unordered_set<size_t>&);
template void Polyline2<GeoPoint<double>>::Generalize(std::vector<GeoPoint<double>>&,
                                                      double,
                                                      const std::unordered_set<size_t>&);
template void Polyline2<GeoPoint<float>>::Generalize(std::list<GeoPoint<float>>&,
                                                     float,
                                                     const std::unordered_set<size_t>&);
template void Polyline2<GeoPoint<double>>::Generalize(std::list<GeoPoint<double>>&,
                                                      double,
                                                      const std::unordered_set<size_t>&);

} // namespace midgard
} // namespace valhalla
