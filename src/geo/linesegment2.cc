#include "geo/linesegment2.h"

namespace valhalla{
namespace geo{

  LineSegment2::LineSegment2() {
    a_.Set(0.0f, 0.0f);
    b_.Set(0.0f, 0.0f);
  }

  LineSegment2::LineSegment2(const Point2& p1, const Point2& p2) {
    a_ = p1;
    b_ = p2;
  }

  Point2 LineSegment2::a() const {
    return a_;
  }

  Point2 LineSegment2::b() const {
    return b_;
  }

  float LineSegment2::DistanceSquared(const Point2& p, Point2& closest) const {
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

  float LineSegment2::Distance(const Point2& p, Point2& closest) const {
    return sqrtf(DistanceSquared(p, closest));
  }

  bool LineSegment2::Intersect(const LineSegment2& segment, Point2& intersect) {
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

  float LineSegment2::IsLeft(const Point2& p) {
    return (b_.x() - a_.x()) * (p.y() - a_.y()) -
           (p.x() - a_.x()) * (b_.y() - a_.y());
  }

}
}
