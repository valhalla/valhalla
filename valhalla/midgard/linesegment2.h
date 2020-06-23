#ifndef VALHALLA_MIDGARD_LINESEGMENT2_H_
#define VALHALLA_MIDGARD_LINESEGMENT2_H_

#include <math.h>
#include <vector>

#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/vector2.h>

namespace valhalla {
namespace midgard {

/**
 * Line segment in 2D. Template class to work with Point2 (Euclidean x,y)
 * or PointLL (latitude,longitude).
 */
template <class coord_t> class LineSegment2 {
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
  LineSegment2(const coord_t& p1, const coord_t& p2) : a_(p1), b_(p2) {
  }

  /**
   * Get the first point of the segment.
   * @return  Returns first point of the segment.
   */
  coord_t a() const {
    return a_;
  }

  /**
   * Get the second point of the segment.
   * @return  Returns second point of the segment.
   */
  coord_t b() const {
    return b_;
  }

  /**
   * Finds the distance squared of a specified point from the line segment
   * and the closest point on the segment to the specified point.
   * @param   p        Test point.
   * @param   closest  (Return) Closest point on the segment to c.
   * @return  Returns the distance squared from pt to the closest point on
   *          the segment.
   */
  float DistanceSquared(const coord_t& p, coord_t& closest) const {
    // Construct vector v (ab) and w (ap)
    VectorXY<typename coord_t::first_type> v(a_, b_);
    VectorXY<typename coord_t::first_type> w(a_, p);

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

  /**
   * Finds the distance of a specified point from the line segment
   * and the closest point on the segment to the specified point.
   * @param   p        Test point.
   * @param   closest  (Return) Closest point on the segment to c.
   * @return  Returns the distance from p to the closest point on
   *          the segment.
   */
  float Distance(const coord_t& p, coord_t& closest) const {
    return sqrtf(DistanceSquared(p, closest));
  }

  /**
   * Determines if the current segment intersects the specified segment.
   * If an intersect occurs the intersection is computed.  Note: the
   * case where the lines overlap is not considered.
   * @param   segment      Segment to determine intersection with.
   * @param   intersect    (OUT) Intersection point.
   * @return  Returns true if an intersection exists, false if not.
   */
  bool Intersect(const LineSegment2<coord_t>& segment, coord_t& intersect) const {
    // Construct vectors
    VectorXY<typename coord_t::first_type> b = b_ - a_;
    VectorXY<typename coord_t::first_type> d = segment.b() - segment.a();

    // Set 2D perpendicular vector to d
    VectorXY<typename coord_t::first_type> dp = d.GetPerpendicular();

    // Check if denominator will be 0 (lines are parallel)
    float dtb = dp.Dot(b);
    if (dtb == 0.0f) {
      return false;
    }

    // Solve for the parameter t
    VectorXY<typename coord_t::first_type> c = segment.a() - a_;
    float t = dp.Dot(c) / dtb;
    if (t < 0.0f || t > 1.0f) {
      return false;
    }

    // Solve for the parameter u
    VectorXY<typename coord_t::first_type> bp = b.GetPerpendicular();
    float u = bp.Dot(c) / dtb;
    if (u < 0.0f || u > 1.0f) {
      return false;
    }

    // An intersect occurs.  Set the intersect point and return true
    intersect = a_ + b * t;
    return true;
  }

  /**
   * Determines if the line segment intersects specified convex polygon.
   * Based on Cyrus-Beck clipping method.
   * @param   poly   A counter-clockwise oriented polygon.
   * @return  Returns true if any part of the segment intersects the polygon,
   *          false if no intersection.
   */
  bool Intersect(const std::vector<coord_t>& poly) const {
    // Initialize the candidate interval
    float t_out = 1.0f;
    float t_in = 0.0f;

    // Iterate through each edge of the polygon
    VectorXY<typename coord_t::first_type> c = b_ - a_;
    auto pt1 = poly.end() - 1;
    auto pt2 = poly.begin();
    for (; pt2 != poly.end(); pt1 = pt2, pt2++) {
      // Set an outward facing normal (polygon is assumed to be CCW)
      VectorXY<typename coord_t::first_type> n(pt2->y() - pt1->y(), pt1->x() - pt2->x());

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

  /**
   * Clips the line segment to a specified convex polygon.
   * Based on Cyrus-Beck clipping method.
   * @param  poly           A counter-clockwise oriented polygon.
   * @param  clip_segment   Returns the clipped segment.
   * @return  Returns true if any part of the segment intersects the polygon,
   *          false if no intersection.
   */
  bool ClipToPolygon(const std::vector<coord_t>& poly, LineSegment2<coord_t>& clip_segment) const {
    // Initialize the candidate interval
    float t_out = 1.0f;
    float t_in = 0.0f;

    // Iterate through each edge of the polygon
    VectorXY<typename coord_t::first_type> c = b_ - a_;
    auto pt1 = poly.end() - 1;
    auto pt2 = poly.begin();
    for (; pt2 != poly.end(); pt1 = pt2, pt2++) {
      // Set an outward facing normal (polygon is assumed to be CCW)
      VectorXY<typename coord_t::first_type> n(pt2->y() - pt1->y(), pt1->x() - pt2->x());

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

  /**
   * Tests if a point is to left, right, or on the line segment.
   * @param    p   Point to test
   * @return   Returns >0 for point to the left, < 0 for point to the right,
   *           and 0 for a point on the line
   */
  float IsLeft(const coord_t& p) const {
    return (b_.x() - a_.x()) * (p.y() - a_.y()) - (p.x() - a_.x()) * (b_.y() - a_.y());
  }

  /**
   * Equality approximation.
   * @param   other  Line segment to compare to the current line segment.
   * @return  Returns true if two line segments are approximately equal, false otherwise.
   */
  bool ApproximatelyEqual(const LineSegment2<coord_t>& other) {
    return a_.ApproximatelyEqual(other.a()) && b_.ApproximatelyEqual(other.b());
  }

private:
  coord_t a_;
  coord_t b_;
};

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_LINESEGMENT2_H_
