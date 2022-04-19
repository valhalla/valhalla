#ifndef VALHALLA_MIDGARD_ELLIPSE_H_
#define VALHALLA_MIDGARD_ELLIPSE_H_

#include <cstdint>
#include <math.h>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace midgard {

// Intersection cases.
enum IntersectCase { kWithin, kContains, kOutside, kIntersects };

/**
 * Ellipse. Methods to construct an ellipse, test if a line segment intersects,
 * an axis-aligned bounding box intersects, and whether a point is within the
 * ellipse. Template class to work with Point2 (Euclidean x,y) or PointLL
 * (latitude,longitude).
 */
template <class coord_t> class Ellipse {
public:
  /**
   * Default constructor.
   */
  Ellipse();

  /**
   * Constructor given bounding rectangle and a rotation.
   * @param  p1     One corner point of the bounding rectangle
   * @param  p2     Another corner point of the bounding rectangle.
   * @param  angle  Angle of rotation
   */
  Ellipse(const coord_t& p1, const coord_t& p2, float angle);

  /**
   * Determines if a line segment intersects the ellipse and if so
   * finds the point(s) of intersection.
   * @param    seg      Line segment to test for intersection
   * @param    pt0      OUT - intersection point (if it exists)
   * @param    pt1      OUT - second intersection point (if it exists)
   * @return   Returns the number of intersection points (0, 1, or 2).
   */
  uint32_t Intersect(const LineSegment2<coord_t>& seg, coord_t& pt0, coord_t& pt1) const;

  /**
   * Does the specified axis-aligned bounding box (rectangle) intersect
   * this ellipse?
   * @param  r  Rectangle to intersect.
   * @return Returns the intersection case.
   *                 kContains   - Ellipse fully contains the AABB
   *                 kIntersects - Ellipse and AABB intersect
   *                 kWithin     - Ellipse is fully within the AABB
   *                 kOutside    - Ellipse is fully outside the AABB
   */
  IntersectCase DoesIntersect(const AABB2<coord_t>& r) const;

  /**
   * Tests if a point is inside the ellipse.
   * @param   pt    Point to test if inside the ellipse.
   * @return  Returns true if the point is on or inside the ellipse,
   *          false if outside the ellipse.
   */
  bool Contains(const coord_t& pt) const;

  /**
   * Get the center of the ellipse.
   * @return Returns the center of the ellipse.
   */
  coord_t center() const {
    return center_;
  }

private:
  coord_t center_;
  float a; // Half length of major axis
  float b; // Half length of minor axis
  float k1_;
  float k2_;
  float k3_;
  float s; // sine of rotation angle
  float c; // cosine of rotation angle
};

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_ELLIPSE_H_
