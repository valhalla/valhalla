#include "valhalla/midgard/point2.h"

#include <cmath>
#include <limits>
#include <list>

#include "midgard/util.h"
#include "midgard/vector2.h"

namespace valhalla {
namespace midgard {

bool Point2::ApproximatelyEqual(const Point2& p, float e) const {
  return equal<first_type>(first, p.first, e) && equal<second_type>(second, p.second, e);
}

float Point2::DistanceSquared(const Point2& p) const {
  return sqr(first - p.first) + sqr(second - p.second);
}

float Point2::Distance(const Point2& p) const {
  return sqrtf(sqr(first - p.first) + sqr(second - p.second));
}

Point2 Point2::AffineCombination(const float a0, const float a1, const Point2& p1) const {
  return Point2(a0 * first + a1 * p1.first, a0 * second + a1 * p1.second);
}

Point2 Point2::MidPoint(const Point2& p1) const {
  return Point2(0.5f * (first + p1.first), 0.5f * (second + p1.second));
}

Point2 Point2::operator+(const Vector2& v) const {
  return Point2(first + v.x(), second + v.y());
}

Point2 Point2::operator-(const Vector2& v) const {
  return Point2(first - v.x(), second - v.y());
}

Vector2 Point2::operator-(const Point2& p) const {
  return Vector2(first - p.first, second - p.second);
}

std::tuple<Point2, float, int> Point2::ClosestPoint(const std::vector<Point2>& pts) const {
  Point2 closest;
  float mindist = std::numeric_limits<float>::max();

  // If there are no points we are done
  if (pts.size() == 0) {
    return std::make_tuple(std::move(closest), std::move(mindist), 0);
  }
  // If there is one point we are done
  if (pts.size() == 1) {
    return std::make_tuple(pts.front(), std::sqrt(DistanceSquared(pts.front())), 0);
  }

  // Iterate through the pts
  bool beyond_end = true; // Need to test past the end point?
  int idx;                // Index of closest segment so far
  Vector2 v1;             // Segment vector (v1)
  Vector2 v2;             // Vector from origin to target (v2)
  Point2 projpt;          // Projected point along v1
  float dot;              // Dot product of v1 and v2
  float comp;             // Component of v2 along v1
  float dist;             // Squared distance from target to closest point on line

  for (size_t index = 0; index < pts.size() - 1; ++index) {
    // Get the current segment
    const Point2& p0 = pts[index];
    const Point2& p1 = pts[index + 1];

    // Construct vector v1 - represents the segment.  Skip 0 length segments
    v1.Set(p0, p1);
    if (v1.x() == 0.0f && v1.y() == 0.0f) {
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

// Tests whether this point is within a polygon. Iterate through the
// edges - to be inside the point must be to the same side of each edge.
template <class container_t> bool Point2::WithinPolygon(const container_t& poly) const {
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

bool Point2::IsSpherical() {
  return false;
}

// Explicit instantiations
template bool Point2::WithinPolygon(const std::vector<Point2>&) const;
template bool Point2::WithinPolygon(const std::list<Point2>&) const;

} // namespace midgard
} // namespace valhalla
