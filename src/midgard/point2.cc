#include "valhalla/midgard/point2.h"

#include <iostream>
#include <limits>

#include "valhalla/midgard/util.h"
#include "valhalla/midgard/vector2.h"

namespace valhalla {
namespace midgard {

Point2::~Point2() {
}

float Point2::x() const {
  return first;
}

float Point2::y() const {
  return second;
}

void Point2::set_x(const float x) {
  first = x;
}

void Point2::set_y(const float y) {
  second = y;
}

void Point2::Set(const float x, const float y) {
  first = x;
  second = y;
}

bool Point2::operator ==(const Point2& p) const {
  return (first == p.x() && second == p.y());
}

bool Point2::operator!=(const Point2& p) const {
  return (first != p.x() || second != p.y());
}

float Point2::DistanceSquared(const Point2& p) const {
  return sqr(first - p.x()) + sqr(second - p.y());
}

float Point2::Distance(const Point2& p) const {
  return sqrtf(sqr(first - p.x()) + sqr(second - p.y()));
}

Point2 Point2::AffineCombination(const float a0, const float a1,
                                 const Point2& p1) const {
  return Point2(a0 * first + a1 * p1.x(), a0 * second + a1 * p1.y());
}

Point2 Point2::MidPoint(const Point2& p1) const {
  return Point2(0.5f * first + 0.5f * p1.x(), 0.5f * second + 0.5f * p1.y());
}

Point2 Point2::operator +(const Vector2& v) const {
  return Point2(first + v.x(), second + v.y());
}

Point2 Point2::operator -(const Vector2& v) const {
  return Point2(first - v.x(), second - v.y());
}

Vector2 Point2::operator -(const Point2& p) const {
  return Vector2(first - p.x(), second - p.y());
}

float Point2::ClosestPoint(const std::vector<Point2>& pts, Point2& closest,
                           int& idx) const {

  float mindist = std::numeric_limits<float>::max();
  // If there are no points we are done
  if(pts.size() == 0)
    return mindist;
  // If there is one point we are done
  if(pts.size() == 1) {
    mindist = DistanceSquared(pts.front());
    closest = pts.front();
    idx = 0;
    return mindist;
  }

  // Iterate through the pts
  bool beyond_end = true;   // Need to test past the end point?
  Vector2 v1;               // Segment vector (v1)
  Vector2 v2;               // Vector from origin to target (v2)
  Point2 projpt;            // Projected point along v1
  float dot;                // Dot product of v1 and v2
  float comp;               // Component of v2 along v1
  float dist;   // Squared distance from target to closest point on line

  for (size_t index = 0; index < pts.size() - 1; ++index) {
    // Get the current segment
    const Point2& p0 = pts[index];
    const Point2& p1 = pts[index + 1];

    // Construct vector v1 - represents the segment.  Skip 0 length segments
    v1.Set(p0, p1);
    if (v1.x() == 0.0f && v1.y() == 0.0f)
      continue;

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
    if (comp >= 1.0f)
      beyond_end = true;
    else {
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
  return mindist;
}

}
}
