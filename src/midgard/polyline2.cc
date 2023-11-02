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
 * TODO: Make this faster (not O(n^2)).
 * @return  T/F if there are self-intersections.
 */
template <typename coord_t> std::vector<coord_t> Polyline2<coord_t>::GetSelfIntersections() {
  std::vector<coord_t> intersections;
  coord_t intersection_point;
  std::vector<coord_t>& points = pts_;
  for (size_t i = 1; i < points.size() - 2; i++) {
    const coord_t& ia(points[i - 1]);
    const coord_t& ib(points[i]);
    for (size_t j = i + 2; j < points.size() - 1; j++) {
      const coord_t& ja(points[j - 1]);
      const coord_t& jb(points[j]);
      LineSegment2<coord_t> segmenti(ia, ib);
      LineSegment2<coord_t> segmentj(ja, jb);
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
 *
 * Notice this is not generically templated for coord_t. This is because
 * this routine employs the PointTileIndex, which indexes space using
 * lats/lons. Hence, this routine only works with PointLL's (aka
 * GeoPoint<double>'s).
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
  // and decide not to simplify. While this example only has one point c between
  // (a,b), there is typically more than one. The logic below will create a triangle
  // using every point c between start and end and perform containment tests for all
  // "nearby" points for every (start,c,end) triangle. We can stop as soon as we find
  // an unexpected point inside our triangle.
  if (dmax < epsilon_sq) {
    // This returns the points in the "epsilon buffer zone" along the line (start, end).
    std::unordered_set<size_t> line_buffer_points =
        point_tile_index.get_points_near_segment(LineSegment2<PointLL>(start, end));

    // We only care about checking for triangle containment for points that are not
    // along the polyline [start,end] - so we can remove those straightaway.
    for (size_t i = sidx; i <= eidx; i++) {
      line_buffer_points.erase(i);
    }

    bool can_simplify = true;
    for (size_t cidx = sidx + 1; (cidx < eidx) && can_simplify; cidx++) {
      const PointLL& c = point_tile_index.points[cidx];
      for (size_t point_idx : line_buffer_points) {
        const PointLL& p = point_tile_index.points[point_idx];
        if (triangle_contains(start, c, end, p)) {
          can_simplify = false;
          break;
        }
      }

      // the moment we realize we cannot simplify we can stop
      if (!can_simplify) {
        break;
      }
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
      peucker_avoid_self_intersections(point_tile_index, epsilon_sq, exclusions, hfidx, eidx);
    if (hfidx - sidx > 1)
      peucker_avoid_self_intersections(point_tile_index, epsilon_sq, exclusions, sidx, hfidx);
  }
}

template <class coord_t, class container_t>
void DouglastPeuckerAvoidSelfIntersection(container_t& polyline,
                                          typename coord_t::value_type epsilon_m,
                                          const std::unordered_set<size_t>& exclusions) {
  // Create a tile-space-index which allows us to perform a Douglas-Peucker style line
  // simplification that avoids creating self-intersections within the polyline/polygon.
  const PointLL& first_point = *polyline.begin();
  double meters_per_deg = DistanceApproximator<PointLL>::MetersPerLngDegree(first_point.lat());
  double epsilon_deg = epsilon_m / meters_per_deg;
  PointTileIndex point_tile_index(epsilon_deg, polyline);

  peucker_avoid_self_intersections(point_tile_index, epsilon_m * epsilon_m, exclusions, 0,
                                   polyline.size() - 1);

  // copy the simplified 'points' into 'polyline'
  polyline.clear();
  for (const auto& pt : point_tile_index.points) {
    if (pt != PointTileIndex::kDeletedPoint) {
      polyline.push_back(pt);
    }
  }
}

template <class coord_t, class container_t>
void DouglasPeucker(container_t& polyline,
                    typename coord_t::value_type epsilon,
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
    typename coord_t::value_type dmax = std::numeric_limits<typename coord_t::value_type>::lowest();
    typename container_t::iterator itr;
    LineSegment2<coord_t> l{*start, *end};
    size_t j = e - 1, k = 0;
    coord_t tmp;
    for (auto i = std::prev(end); i != start; --i, --j) {
      // special points we dont want to generalize no matter what take precedence
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

/**
 * Generalize the given list of points
 *
 * @param polyline    the list of points
 * @param epsilon     the tolerance (in meters) used in removing points
 * @param exclusions  list of indices of points not to generalize
 * @param avoid_self_intersection  avoid simplifications that cause self-intersection
 */
template <typename coord_t>
template <class container_t>
void Polyline2<coord_t>::Generalize(container_t& polyline,
                                    typename coord_t::value_type epsilon_m,
                                    const std::unordered_set<size_t>& exclusions,
                                    bool avoid_self_intersection) {
  // any epsilon this low will have no effect on the input nor will any super short input
  if (epsilon_m <= 0 || polyline.size() < 3)
    return;

  if (avoid_self_intersection)
    DouglastPeuckerAvoidSelfIntersection<coord_t>(polyline, epsilon_m, exclusions);
  else
    DouglasPeucker<coord_t>(polyline, epsilon_m, exclusions);
}

template <typename coord_t>
template <typename container_t>
typename container_t::value_type::first_type
Polyline2<coord_t>::HausdorffDistance(const container_t& l1, const container_t& l2) {
  typename container_t::value_type::first_type hausdorff = 0;

  // which point of l1 is furthest away from l2
  for (const auto& p : l1) {
    auto closest = p.ClosestPoint(l2);
    auto min_distance = p.Distance(std::get<0>(closest));
    if (min_distance > hausdorff)
      hausdorff = min_distance;
  }

  // which point of l2 is furthest away from l1
  for (const auto& p : l2) {
    auto closest = p.ClosestPoint(l1);
    auto min_distance = p.Distance(std::get<0>(closest));
    if (min_distance > hausdorff)
      hausdorff = min_distance;
  }

  return hausdorff;
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
                                                    const std::unordered_set<size_t>&,
                                                    bool);
template void Polyline2<PointXY<double>>::Generalize(std::vector<PointXY<double>>&,
                                                     double,
                                                     const std::unordered_set<size_t>&,
                                                     bool);
template void Polyline2<PointXY<float>>::Generalize(std::list<PointXY<float>>&,
                                                    float,
                                                    const std::unordered_set<size_t>&,
                                                    bool);
template void Polyline2<PointXY<double>>::Generalize(std::list<PointXY<double>>&,
                                                     double,
                                                     const std::unordered_set<size_t>&,
                                                     bool);
template void Polyline2<GeoPoint<float>>::Generalize(std::vector<GeoPoint<float>>&,
                                                     float,
                                                     const std::unordered_set<size_t>&,
                                                     bool);
template void Polyline2<GeoPoint<double>>::Generalize(std::vector<GeoPoint<double>>&,
                                                      double,
                                                      const std::unordered_set<size_t>&,
                                                      bool);
template void Polyline2<GeoPoint<float>>::Generalize(std::list<GeoPoint<float>>&,
                                                     float,
                                                     const std::unordered_set<size_t>&,
                                                     bool);
template void Polyline2<GeoPoint<double>>::Generalize(std::list<GeoPoint<double>>&,
                                                      double,
                                                      const std::unordered_set<size_t>&,
                                                      bool);

template double Polyline2<GeoPoint<double>>::HausdorffDistance(const std::vector<GeoPoint<double>>&,
                                                               const std::vector<GeoPoint<double>>&);

} // namespace midgard
} // namespace valhalla
