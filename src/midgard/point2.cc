#include "midgard/point2.h"
#include "midgard/util.h"
#include "midgard/vector2.h"

#include <list>

namespace valhalla {
namespace midgard {

/**
 * Finds the closest point to the supplied polyline as well as the distance
 * to that point and the index of the segment where the closest point lies.
 * @param  pts     List of points on the polyline.
 * @return  tuple of <Closest point along the polyline,
 *                    Returns the distance of the closest point,
 *                    Index of the segment of the polyline which contains the closest point
 *                   >
 */
template <typename PrecisionT>
std::tuple<PointXY<PrecisionT>, PrecisionT, int>
PointXY<PrecisionT>::ClosestPoint(const std::vector<PointXY<PrecisionT>>& pts) const {
  PointXY<PrecisionT> closest;
  value_type mindist = std::numeric_limits<value_type>::max();

  // If there are no points we are done
  if (pts.size() == 0) {
    return std::make_tuple(std::move(closest), std::move(mindist), 0);
  }
  // If there is one point we are done
  if (pts.size() == 1) {
    return std::make_tuple(pts.front(), std::sqrt(DistanceSquared(pts.front())), 0);
  }

  // Iterate through the pts
  bool beyond_end = true;     // Need to test past the end point?
  int idx = 0;                // Index of closest segment so far
  VectorXY<PrecisionT> v1;    // Segment vector (v1)
  VectorXY<PrecisionT> v2;    // Vector from origin to target (v2)
  PointXY<PrecisionT> projpt; // Projected point along v1
  value_type dot;             // Dot product of v1 and v2
  value_type comp;            // Component of v2 along v1
  value_type dist;            // Squared distance from target to closest point on line

  for (size_t index = 0; index < pts.size() - 1; ++index) {
    // Get the current segment
    const PointXY<PrecisionT>& p0 = pts[index];
    const PointXY<PrecisionT>& p1 = pts[index + 1];

    // Construct vector v1 - represents the segment.  Skip 0 length segments
    // that are not at the end of the line.
    v1.Set(p0, p1);
    if (v1.x() == 0.0f && v1.y() == 0.0f && index < pts.size() - 2) {
      continue;
    }

    // Vector v2 from the segment origin to the target point
    v2.Set(p0, *this);

    // Find the dot product of v1 and v2.  If less than 0 the segment
    // origin is the closest point.  Find the distance and continue
    // to the next segment.
    dot = v1.Dot(v2);
    if (dot <= 0.0f) {
      beyond_end = false;
      dist = DistanceSquared(p0);
      if (dist < mindist) {
        mindist = dist;
        closest = p0;
        idx = index;
      }
      continue;
    }

    // Closest point is either beyond the end of the segment or at a point
    // along the segment. Find the component of v2 along v1
    comp = dot / v1.Dot(v1);

    // If component >= 1.0 the segment end is the closest point. A future
    // polyline segment will be closer.  If last segment we need to check
    // distance to the endpoint.  Set flag so this happens.
    if (comp >= 1.0f) {
      beyond_end = true;
    } else {
      // Closest point is along the segment.  The closest point is found
      // by adding the projection of v2 onto v1 to the origin point.
      // The squared distance from this point to the target is then found.
      beyond_end = false;
      projpt = p0 + v1 * comp;
      dist = DistanceSquared(projpt);
      if (dist < mindist) {
        mindist = dist;
        closest = projpt;
        idx = index;
      }
    }
  }

  // Test the end point if flag is set - it may be the closest point
  if (beyond_end) {
    dist = DistanceSquared(pts.back());
    if (dist < mindist) {
      mindist = dist;
      closest = pts.back();
      idx = static_cast<int>(pts.size() - 2);
    }
  }
  return std::make_tuple(std::move(closest), std::move(std::sqrt(mindist)), std::move(idx));
}

/**
 * Tests whether this point is within a polygon.
 * @param  poly  List of vertices that form a polygon. Assumes
 *               the following:
 *                  Only the first and last vertices may be duplicated.
 * @return  Returns true if the point is within the polygon, false if not.
 */
template <typename PrecisionT>
template <typename container_t>
bool PointXY<PrecisionT>::WithinPolygon(const container_t& poly) const {
  auto p1 = poly.front() == poly.back() ? poly.begin() : std::prev(poly.end());
  auto p2 = poly.front() == poly.back() ? std::next(p1) : poly.begin();
  // for each edge
  size_t winding_number = 0;
  for (; p2 != poly.end(); p1 = p2, ++p2) {
    // going upward
    if (p1->second <= second) {
      // crosses if its in between on the y and to the left
      winding_number += p2->second > second && IsLeft(*p1, *p2) > 0;
    } // going downward maybe
    else {
      // crosses if its in between or on and to the right
      winding_number -= p2->second <= second && IsLeft(*p1, *p2) < 0;
    }
  }

  // If it was a full ring we are done otherwise check the last segment
  return winding_number != 0;
}

// explicit instantiations
template class VectorXY<float>;
template class PointXY<float>;
template class VectorXY<double>;
template class PointXY<double>;
template bool PointXY<float>::WithinPolygon(const std::vector<PointXY<float>>&) const;
template bool PointXY<float>::WithinPolygon(const std::list<PointXY<float>>&) const;
template bool PointXY<double>::WithinPolygon(const std::vector<PointXY<double>>&) const;
template bool PointXY<double>::WithinPolygon(const std::list<PointXY<double>>&) const;
} // namespace midgard
} // namespace valhalla

namespace std {
template <typename PrecisionT>
size_t hash<valhalla::midgard::PointXY<PrecisionT>>::operator()(
    const valhalla::midgard::PointXY<PrecisionT>& p) const {
  size_t seed = 0;
  valhalla::midgard::hash_combine(seed, p.first);
  valhalla::midgard::hash_combine(seed, p.second);
  return seed;
}

template size_t
hash<valhalla::midgard::PointXY<float>>::operator()(const valhalla::midgard::PointXY<float>&) const;
template size_t
hash<valhalla::midgard::PointXY<double>>::operator()(const valhalla::midgard::PointXY<double>&) const;
} // namespace std
