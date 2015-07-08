#include "valhalla/midgard/linesegment2.h"
#include <cmath>

namespace valhalla {
namespace midgard {

// Default constructor.
LineSegment2::LineSegment2() {
  a_.Set(0.0f, 0.0f);
  b_.Set(0.0f, 0.0f);
}

// Constructor given 2 points.
LineSegment2::LineSegment2(const Point2& p1, const Point2& p2) {
  a_ = p1;
  b_ = p2;
}

// Get the first point of the segment.
Point2 LineSegment2::a() const {
  return a_;
}

// Get the second point of the segment.
Point2 LineSegment2::b() const {
  return b_;
}

// Finds the distance squared of a specified point from the line segment
// and the closest point on the segment to the specified point.
float LineSegment2::DistanceSquared(const Point2& p, Point2& closest) const {
  // Construct vector v (ab) and w (ap)
  Vector2 v(a_, b_);
  Vector2 w(a_, p);

  // Numerator of the component of w onto v. If <= 0 then a
  // is the closest point. By separat_ing into the numerator
  // and denominator of the component we avoid a division unless
  // it is necessary.
  float n = w.Dot(v);
  if (n <= 0.0f) {
    closest = a_;
  }
  else {
    // Get the denominator of the component.  If the component >= 1
    // (d <= n) then point b is the closest point
    float d = v.Dot(v);
    if (d <= n) {
      closest = b_;
    }
    else {
      // Closest point is along the segment - the projection of w onto v.
      closest = a_ + v * (n / d);
    }
  }
  return closest.DistanceSquared(p);
}

// Finds the distance of a specified point from the line segment
// and the closest point on the segment to the specified point.
float LineSegment2::Distance(const Point2& p, Point2& closest) const {
  return sqrtf(DistanceSquared(p, closest));
}

// Determines if the current segment intersects the specified segment.
// If an intersect occurs the intersectPt is determined.  Note: the
// case where the lines overlap is not considered.
bool LineSegment2::Intersect(const LineSegment2& segment,
             Point2& intersect) const {
  // Construct vectors
  Vector2 b = b_ - a_;
  Vector2 d = segment.b() - segment.a();

  // Set 2D perpendicular vector to d
  Vector2 dp = d.GetPerpendicular();

  // Check if denominator will be 0 (lines are parallel)
  float dtb = dp.Dot(b);
  if (dtb == 0.0f)
    return false;

  // Solve for the parameter t
  Vector2 c = segment.a() - a_;
  float t = dp.Dot(c) / dtb;
  if (t < 0.0f || t > 1.0f)
    return false;

  // Solve for the parameter u
  Vector2 bp = b.GetPerpendicular();
  float u = bp.Dot(c) / dtb;
  if (u < 0.0f || u > 1.0f)
    return false;

  // An intersect occurs.  Set the intersect point and return true
  intersect = a_ + b * t;
  return true;
}

// Determines if the line segment intersects specified convex polygon.
bool LineSegment2::Intersect(const std::vector<Point2>& poly) const {
  // Initialize the candidate interval
  float t_out = 1.0f;
  float t_in  = 0.0f;

  // Iterate through each edge of the polygon
  Vector2 c = b_ - a_;
  auto pt1 = poly.end() - 1;
  auto pt2 = poly.begin();
  for ( ; pt2 != poly.end(); pt1 = pt2, pt2++) {
    // Set an outward facing normal (polygon is assumed to be CCW)
    Vector2 n(pt2->y() - pt1->y(), pt1->x() - pt2->x());

    // Dot product of the normal to this polygon edge with the ray
    float n_dot_c = n.Dot(c);
    float num = n.Dot((*pt1 - a_));

    // Check for parallel line
    if (std::abs(n_dot_c) < kEpsilon) {
      // No intersection if segment origin is outside wrt to this edge
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
bool LineSegment2::ClipToPolygon(const std::vector<Point2>& poly,
                   LineSegment2& clip_segment) const {
  // Initialize the candidate interval
  float t_out = 1.0f;
  float t_in  = 0.0f;

  // Iterate through each edge of the polygon
  Vector2 c = b_ - a_;
  auto pt1 = poly.end() - 1;
  auto pt2 = poly.begin();
  for ( ; pt2 != poly.end(); pt1 = pt2, pt2++) {
    // Set an outward facing normal (polygon is assumed to be CCW)
    Vector2 n(pt2->y() - pt1->y(), pt1->x() - pt2->x());

    // Dot product of the normal to this polygon edge with the ray
    float n_dot_c = n.Dot(c);
    float num = n.Dot((*pt1 - a_));

    // Check for parallel line
    if (std::abs(n_dot_c) < kEpsilon) {
      // No intersection if segment origin is outside wrt to this edge
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
  clip_segment = { a_ + c * t_in, a_ + c * t_out };
  return true;
}

// Tests if a point is to left, right, or on the line segment.
float LineSegment2::IsLeft(const Point2& p) const {
  return (b_.x() - a_.x()) * (p.y() - a_.y()) -
         (p.x() - a_.x()) * (b_.y() - a_.y());
}

}
}
