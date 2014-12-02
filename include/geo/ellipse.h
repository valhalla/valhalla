#ifndef __ellipse_h__
#define __ellipse_h__

#include <math.h>

#include "point2.h"
#include "util.h"
#include "linesegment2.h"
#include "aabb2.h"

namespace valhalla{
namespace geo{

/**
 * Ellipse
 */
class Ellipse {
 public:
  /**
   * Default constructor.
   */
  Ellipse();

  /**
   * Constructor given bounding rectangle and a rotation.
   * @param  p1   One corner point of the bounding rectangle
   * @param
   */
  Ellipse(const Point2& p1, const Point2& p2, float angle);

  /**
   * Determines if a line segment intersects the ellipse and if so
   * finds the point(s) of intersection.
   * @param    seg      Line segment to test for intersection
   * @param    pt0      OUT - intersection point (if it exists)
   * @param    pt1      OUT - second intersection point (if it exists)
   * @return   Returns the number of intersection points (0, 1, or 2).
   */
  unsigned int Intersect(const LineSegment2& seg, Point2& pt0,
                         Point2& pt1) const;

  /**
   * Does the specified axis-aligned bounding box (rectangle) intersect
   * this ellipse.
   * @param  r  Rectangle to intersect.
   * @return Returns the intersection case.
   *                 kContains   - Ellipse fully contains the AABB
   *                 kIntersects - Ellipse and AABB intersect
   *                 kWithin     - Ellipse is fully within the AABB
   *                 kOutside    - Ellipse is fully outside the AABB
   */
  IntersectCase DoesIntersect(const AABB2& r) const;

  /**
   * Tests if a point is inside the ellipse.
   * @param   pt    Point to test if inside the ellipse.
   * @return  Returns true if the point is on or inside the ellipse,
   *          false if outside the ellipse.
   */
  bool Contains(const Point2& pt) const;

 private:
  Point2 center_;
  float a;                  // Half length of major axis
  float b;                  // Half length of minor axis
  float k1_;
  float k2_;
  float k3_;
  float s;                  // sine of rotation angle
  float c;                  // cosine of rotation angle
};

}
}

#endif


