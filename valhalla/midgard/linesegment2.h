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
  float DistanceSquared(const coord_t& p, coord_t& closest) const;

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
  bool Intersect(const LineSegment2<coord_t>& segment, coord_t& intersect) const;

  /**
   * Determines if the line segment intersects specified convex polygon.
   * Based on Cyrus-Beck clipping method.
   * @param   poly   A counter-clockwise oriented polygon.
   * @return  Returns true if any part of the segment intersects the polygon,
   *          false if no intersection.
   */
  bool Intersect(const std::vector<coord_t>& poly) const;

  /**
   * Clips the line segment to a specified convex polygon.
   * Based on Cyrus-Beck clipping method.
   * @param  poly           A counter-clockwise oriented polygon.
   * @param  clip_segment   Returns the clipped segment.
   * @return  Returns true if any part of the segment intersects the polygon,
   *          false if no intersection.
   */
  bool ClipToPolygon(const std::vector<coord_t>& poly, LineSegment2<coord_t>& clip_segment) const;

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
