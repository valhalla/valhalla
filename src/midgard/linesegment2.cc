#include "valhalla/midgard/linesegment2.h"
#include <cmath>

namespace valhalla {
namespace midgard {

// Finds the distance squared of a specified point from the line segment
// and the closest point on the segment to the specified point.
template <class coord_t>
float LineSegment2<coord_t>::DistanceSquared(const coord_t& p, coord_t& closest) const {
  // Construct vector v (ab) and w (ap)
  Vector2 v(a_, b_);
  Vector2 w(a_, p);

  // Numerator of the component of w onto v. If <= 0 then a
  // is the closest point. By separating into the numerator
  // and denominator of the component we avoid a division unless
  // it is necessary.
  float n = w.Dot(v);
  if (n <= 0.0f) {
    closest = a_;
  } else {
    // Get the denominator of the component.  If the component >= 1
    // (d <= n) then point b is the closest point
    float d = v.Dot(v);
    if (d <= n) {
      closest = b_;
    } else {
      // Closest point is along the segment - the projection of w onto v.
      closest = a_ + v * (n / d);
    }
  }
  return closest.DistanceSquared(p);
}

// Determines if the current segment intersects the specified segment.
// If an intersect occurs the intersection is computed.  Note: the
// case where the lines overlap is not considered.
template <class coord_t>
bool LineSegment2<coord_t>::Intersect(const LineSegment2<coord_t>& segment,
                                      coord_t& intersect) const {
  // Construct vectors
  Vector2 b = b_ - a_;
  Vector2 d = segment.b() - segment.a();

  // Set 2D perpendicular vector to d
  Vector2 dp = d.GetPerpendicular();

  // Check if denominator will be 0 (lines are parallel)
  float dtb = dp.Dot(b);
  if (dtb == 0.0f) {
    return false;
  }

  // Solve for the parameter t
  Vector2 c = segment.a() - a_;
  float t = dp.Dot(c) / dtb;
  if (t < 0.0f || t > 1.0f) {
    return false;
  }

  // Solve for the parameter u
  Vector2 bp = b.GetPerpendicular();
  float u = bp.Dot(c) / dtb;
  if (u < 0.0f || u > 1.0f) {
    return false;
  }

  // An intersect occurs.  Set the intersect point and return true
  intersect = a_ + b * t;
  return true;
}

// Determines if the line segment intersects specified convex polygon.
template <class coord_t>
bool LineSegment2<coord_t>::Intersect(const std::vector<coord_t>& poly) const {
  // Initialize the candidate interval
  float t_out = 1.0f;
  float t_in = 0.0f;

  // Iterate through each edge of the polygon
  Vector2 c = b_ - a_;
  auto pt1 = poly.end() - 1;
  auto pt2 = poly.begin();
  for (; pt2 != poly.end(); pt1 = pt2, pt2++) {
    // Set an outward facing normal (polygon is assumed to be CCW)
    Vector2 n(pt2->y() - pt1->y(), pt1->x() - pt2->x());

    // Dot product of the normal to this polygon edge with the ray
    float n_dot_c = n.Dot(c);
    float num = n.Dot((*pt1 - a_));

    // Check for parallel line
    if (std::abs(n_dot_c) < kEpsilon) {
      // No intersection if segment origin is outside this edge
      if (num < 0) {
        return false;
      } else {
        // Skip this edge
        continue;
      }
    }

    // Get intersection and update candidate interval
    float t = num / n_dot_c;
    if (n_dot_c > 0) {
      // Ray is exiting polygon
      if (t < t_out) {
        t_out = t;
      }
    } else {
      // Ray is entering polygon
      if (t > t_in) {
        t_in = t;
      }
    }

    // Early out
    if (t_in > t_out) {
      return false;
    }
  }
  return true;
}

// Clips the line segment to a specified convex polygon.
template <class coord_t>
bool LineSegment2<coord_t>::ClipToPolygon(const std::vector<coord_t>& poly,
                                          LineSegment2<coord_t>& clip_segment) const {
  // Initialize the candidate interval
  float t_out = 1.0f;
  float t_in = 0.0f;

  // Iterate through each edge of the polygon
  Vector2 c = b_ - a_;
  auto pt1 = poly.end() - 1;
  auto pt2 = poly.begin();
  for (; pt2 != poly.end(); pt1 = pt2, pt2++) {
    // Set an outward facing normal (polygon is assumed to be CCW)
    Vector2 n(pt2->y() - pt1->y(), pt1->x() - pt2->x());

    // Dot product of the normal to this polygon edge with the ray
    float n_dot_c = n.Dot(c);
    float num = n.Dot((*pt1 - a_));

    // Check for parallel line
    if (std::abs(n_dot_c) < kEpsilon) {
      // No intersection if segment origin is outside this edge
      if (num < 0) {
        return false;
      } else {
        // Skip this edge
        continue;
      }
    }

    // Get intersection and update candidate interval
    float t = num / n_dot_c;
    if (n_dot_c > 0) {
      // Ray is exiting polygon
      if (t < t_out) {
        t_out = t;
      }
    } else {
      // Ray is entering polygon
      if (t > t_in) {
        t_in = t;
      }
    }

    // Early out
    if (t_in > t_out) {
      return false;
    }
  }

  // If candidate interval is not empty then set the clip segment
  clip_segment = {a_ + c * t_in, a_ + c * t_out};
  return true;
}

// Explicit instantiation
template class LineSegment2<Point2>;
template class LineSegment2<PointLL>;

} // namespace midgard
} // namespace valhalla
