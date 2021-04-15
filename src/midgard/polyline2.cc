#include "midgard/polyline2.h"
#include "midgard/distanceapproximator.h"
#include "midgard/point2.h"
#include "midgard/point_tile_index.h"
#include "midgard/util.h"

#include <list>

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

/**
 * Uses a brute-force O(n^2) technique to determine if there are any
 * self-intersections.
 * TODO: Use the TilePointIndex to make this faster.
 * @return  T/F if there are self-intersections.
 */
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

/**
 * A Douglas-Peucker line simplification algorithm that will not generate
 * self-intersections.
 */
void peucker_avoid_self_intersections(PointTileIndex& point_tile_index,
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

  const PointLL& start = point_tile_index.points[sidx];
  const PointLL& end = point_tile_index.points[eidx];

  double dmax = std::numeric_limits<double>::lowest();
  LineSegment2<PointLL> line_segment{start, end};

  // hfidx is the index of the highest freq detail (the dividing point)
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

    const PointLL& c = point_tile_index.points[idx];

    // test if this is the highest frequency detail so far
    auto d = line_segment.DistanceSquared(c, tmp);
    if (d > dmax) {
      dmax = d;
      hfidx = idx;
    }
  }

  // If (dmax < epsilon_sq) then we have a relatively straight line between (start,end).
  // A standard Douglas-Peucker algorithm would immediately decimate all the points
  // between (start,end). In this modified version, we use our tiled-point-space to
  // determine if decimating the line would result in a self-intersection.
  //
  // We use our tiled space to determine the points along the "epsilon buffer zone" of
  // the line (start,end). Because our tiled-point-space is coarse, our
  // "get_points_near_segment" query will contain points both of interest and not.
  // Consider this amazing ascii art example:
  //
  //                i             k
  //                 \           /
  //                  \         /
  //                   \       /
  //                    \     /
  //   s - - - - - - - - - - - - - - - - - - - - - - - - - - - - e
  //     `  .             \ /                             `
  //            `  .       j                 .
  //                  `  c        `
  //
  // s=start, e=end. c is a point along the polyline between s & e. We are considering
  // getting rid of c because it is within epsilon of (a,b).
  //
  // All the points shown in this hypothetical example are returned from the call to
  // "get_points_near_segment".
  //
  // As you can see, a completely separate portion of our polygon (i, j, k) would
  // self-intersect if we simplified. To detect this, we perform a triangle
  // containment test of point j using the triangle (s, c, e), see that its contained,
  // and decide not to simplify. While this example only has one point c, between
  // (a,b), there is typically more than one. The logic below will create a triangle
  // using every point c between start and end - and perform containment tests for all
  // "nearby" points for every (start,c,end) triangle. We can stop as soon as we find
  // an unexpected point inside our triangle.
  if (dmax < epsilon_sq) {
    // This returns the points in the "epsilon buffer zone" along the line (start, end).
    std::unordered_set<size_t> line_buffer_points;
    point_tile_index.get_points_near_segment(LineSegment2<PointLL>(start, end), line_buffer_points);

    bool can_simplify = true;
    for (size_t cidx = sidx + 1; (cidx < eidx) && can_simplify; cidx++) {
      const PointLL& c = point_tile_index.points[cidx];

      for (size_t point_idx : line_buffer_points) {
        if (!triangle_contains(start, c, end, point_tile_index.points[point_idx])) {
          line_buffer_points.erase(point_idx);
        }
      }

      // Disregard points that are along the polyline [start,end].
      for (size_t i = sidx; i <= eidx; i++) {
        line_buffer_points.erase(i);
      }

      // If no points remain within the triangle, we can simplify this line
      can_simplify = line_buffer_points.empty();

      // the moment we realize we cannot simplify we can stop
      if (!can_simplify)
        break;
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

  // if (dmax >= epsilon_sq) there are some high frequency details between start
  // and end so we need to look for flatter sections between them.
  if (dmax >= epsilon_sq) {
    // we recurse from right to left for two reasons:
    // 1. we want to preserve iterator validity in the vector version
    // 2. its the only way to preserve the indices in the keep set
    if (eidx - hfidx > 1)
      peucker_avoid_self_intersections(point_tile_index, epsilon_sq, exclusions, hfidx,
                                                    eidx);
    if (hfidx - sidx > 1)
      peucker_avoid_self_intersections(point_tile_index, epsilon_sq, exclusions, sidx,
                                                    hfidx);
  }
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

  // Create a tile-space-index which allows us to conduct a peucker style line
  // simplification that avoids creating self-intersections within the polyline/polygon.
  const PointLL& first_point = *polyline.begin();
  double meters_per_deg = DistanceApproximator<PointLL>::MetersPerLngDegree(first_point.lat());
  double epsilon_in_deg = epsilon / meters_per_deg;
  PointTileIndex point_tile_index(epsilon_in_deg);
  point_tile_index.tile<container_t>(polyline);

  peucker_avoid_self_intersections(point_tile_index, epsilon * epsilon, exclusions, 0,
                                                polyline.size() - 1);

  // copy the simplified polyline into 'polyline'
  polyline.clear();
  for (const auto& pt : point_tile_index.points) {
    if (pt != PointTileIndex::deleted_point) {
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
