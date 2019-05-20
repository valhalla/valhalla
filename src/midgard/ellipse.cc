#include <cmath>

#include "valhalla/midgard/ellipse.h"

namespace valhalla {
namespace midgard {

// Default constructor.
template <class coord_t> Ellipse<coord_t>::Ellipse() : s(0), c(0) {
  center_.Set(0.0f, 0.0f);
  a = b = 0.0f;
  k1_ = k2_ = k3_ = 0.0f;
}

// Constructor given bounding rectangle and a rotation.
template <class coord_t>
Ellipse<coord_t>::Ellipse(const coord_t& p1, const coord_t& p2, float angle) {
  // Set the center and get sin and cos of the angle
  center_.Set(p1.x() + p2.x() * 0.5f, p1.y() + p2.y() * 0.5f);
  float angleRad = angle * kRadPerDeg;
  c = cosf(angleRad);
  s = sinf(angleRad);

  // Find the half lengths of the semi-major and semi-minor axes
  float dx = std::fabs(p2.x() - p1.x()) * 0.5;
  float dy = std::fabs(p2.y() - p1.y()) * 0.5;
  if (dx >= dy) {
    a = dx;
    b = dy;
  } else {
    a = dy;
    b = dx;
  }

  // Find k1_, k2_, k3_ - define when a point x,y is on the ellipse
  k1_ = sqr(c / a) + sqr(s / b);
  k2_ = 2 * s * c * ((1 / sqr(a)) - (1 / sqr(b)));
  k3_ = sqr(s / a) + sqr(c / b);
}

// Determines if a line segment intersects the ellipse and if so
// finds the point(s) of intersection.
template <class coord_t>
uint32_t
Ellipse<coord_t>::Intersect(const LineSegment2<coord_t>& seg, coord_t& pt0, coord_t& pt1) const {
  // Solution is found by parameterizing the line segment and
  // substituting those values into the ellipse equation resulting
  // in a quadratic equation.
  float x1 = center_.x();
  float y1 = center_.y();
  float u1 = seg.a().x();
  float v1 = seg.a().y();
  float u2 = seg.b().x();
  float v2 = seg.b().y();
  float dx = u2 - u1;
  float dy = v2 - v1;
  float q0 = k1_ * sqr(u1 - x1) + k2_ * (u1 - x1) * (v1 - y1) + k3_ * sqr(v1 - y1) - 1;
  float q1 = (2 * k1_ * dx * (u1 - x1)) + (k2_ * dx * (v1 - y1)) + (k2_ * dy * (u1 - x1)) +
             (2 * k3_ * dy * (v1 - y1));
  float q2 = (k1_ * sqr(dx)) + (k2_ * dx * dy) + (k3_ * sqr(dy));

  // Compare q1^2 to 4*q0*q2 to see how quadratic solves
  float d = sqr(q1) - (4 * q0 * q2);
  if (d < 0) {
    // Complex roots. Line containing the segment does
    // not intersect the ellipse
    return 0;
  }

  if (d == 0) {
    // One real-valued root - line is tangent to the ellipse
    float t = -q1 / (2 * q2);
    if (0 <= t && t <= 1) {
      // Intersection occurs along line segment
      pt0.Set(u1 + t * dx, v1 + t * dy);
      return 1;
    } else {
      return 0;
    }
  } else {
    // 2 distinct real-valued roots. Solve for the roots and see if
    // they fall along the line segment
    uint32_t n = 0;
    float q = std::sqrt(d);
    float t = (-q1 - q) / (2 * q2);
    if (0 <= t && t <= 1) {
      // Intersection occurs along line segment
      pt0.Set(u1 + t * dx, v1 + t * dy);
      n++;
    }

    // 2nd root
    t = (-q1 + q) / (2 * q2);
    if (0 <= t && t <= 1) {
      if (n == 0) {
        pt0.Set(u1 + t * dx, v1 + t * dy);
        n++;
      } else {
        pt1.Set(u1 + t * dx, v1 + t * dy);
        n++;
      }
    }
    return n;
  }
}

// Does the specified axis-aligned bounding box (rectangle) intersect
// this ellipse.
template <class coord_t>
IntersectCase Ellipse<coord_t>::DoesIntersect(const AABB2<coord_t>& r) const {
  // Test if all 4 corners of the rectangle are inside the ellipse
  coord_t ul(r.minx(), r.maxy());
  coord_t ur(r.maxx(), r.maxy());
  coord_t ll(r.minx(), r.miny());
  coord_t lr(r.maxx(), r.miny());
  if (Contains(ul) && Contains(ur) && Contains(ll) && Contains(lr)) {
    return kContains;
  }

  // Test if any of the rectangle edges intersect
  coord_t pt0, pt1;
  LineSegment2<coord_t> bottom(ll, lr);
  if (Intersect(bottom, pt0, pt1) > 0) {
    return kIntersects;
  }

  LineSegment2<coord_t> top(ul, ur);
  if (Intersect(top, pt0, pt1) > 0) {
    return kIntersects;
  }

  LineSegment2<coord_t> left(ll, ul);
  if (Intersect(left, pt0, pt1) > 0) {
    return kIntersects;
  }

  LineSegment2<coord_t> right(lr, ur);
  if (Intersect(right, pt0, pt1) > 0) {
    return kIntersects;
  }

  // Ellipse does not intersect any edge. The case for containing the
  // rectangle was considered above so if the ellipse center is inside the
  // AABB then the AABB is fully inside and if ellipse center is outside
  // then the AABB is fully outside.
  return (r.Contains(center_)) ? kWithin : kOutside;
}

// Tests if a point is inside the ellipse.
template <class coord_t> bool Ellipse<coord_t>::Contains(const coord_t& pt) const {
  // Plug in equation for ellipse, If evaluates to <= 0 then the
  // point is in or on the ellipse.
  float dx = pt.x() - center_.x();
  float dy = pt.y() - center_.y();
  return ((k1_ * sqr(dx)) + (k2_ * dx * dy) + (k3_ * sqr(dy)) - 1) < kEpsilon;
}

// Explicit instantiation
template class Ellipse<Point2>;
template class Ellipse<PointLL>;

} // namespace midgard
} // namespace valhalla
