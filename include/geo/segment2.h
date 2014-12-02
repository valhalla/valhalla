#ifndef __segment2_h__
#define __segment2_h__

#include <math.h>
#include <vector>

namespace valhalla{
namespace geo{

/**
 * Line segment in 2D
 * @author  David W. Nesbitt
 */
class LineSegment2 {
 public:
  /**
   * Default constructor.
   */
  LineSegment2() {
    a_.Set(0.0f, 0.0f);
    b_.Set(0.0f, 0.0f);
  }

  /**
   * Constructor given 2 points.
   * @param   p1    First point of the segment.
   * @param   p2    Second point of the segment.
   */
  LineSegment2(const Point2& p1, const Point2& p2) {
    a_ = p1;
    b_ = p2;
  }

  /**
   * Get the first point of the segment.
   * @return  Returns first point of the segment.
   */
  Point2 a() const {
    return a_;
  }

  /**
   * Get the second point of the segment.
   * @return  Returns second point of the segment.
   */
  Point2 b() const {
    return b_;
  }

/**
   * Finds the distance squared of a specified point from the line segment
   * and the closest point on the segement to the specified point.
   * @param   p        Test point.
   * @param   closest  (Return) Closest point on the segment to c.
   * @return  Returns the distance squared from pt to the closest point on
   *          the segment.
  */
  float DistanceSquared(const Point2& p, Point2& closest) const {
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

  /**
   * Finds the distance of a specified point from the line segment
   * and the closest point on the segement to the specified point.
   * @param   p        Test point.
   * @param   closest  (Return) Closest point on the segment to c.
   * @return  Returns the distance from pt to the closest point on
   *          the segment.
  */
  float Distance(const Point2& p, Point2& closest) const {
    return sqrtf(DistanceSquared(p, closest));
  }

  /**
   * Determines if the current segment intersects the specified segment.
   * If an intersect occurs the intersectPt is determined.  Note: the
   * case where the lines overlap is not considered.
   * @param   segment      Segment to determine intersection with.
   * @param   intersect    (OUT) Intersection point.
   * @return  Returns true if an intersection exists, false if not.
   */
  bool Intersect(const LineSegment2& segment, Point2& intersect) {
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

  /**
   * Tests if a point is to left, right, or on the line segment
   * @param    p   Point to test
   * @return   Returns >0 for point to the left, < 0 for point to the right,
   *           and 0 for a point on the line
   */
  float IsLeft(const Point2& p) {
    return (b_.x() - a_.x()) * (p.y() - a_.y()) -
           (p.x() - a_.x()) * (b_.y() - a_.y());
  }

 private:
  Point2 a_;
  Point2 b_;
};


}
}

#endif
