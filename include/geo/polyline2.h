#ifndef VALHALLA_GEO_POLYLINE2_H_
#define VALHALLA_GEO_POLYLINE2_H_

#include "point2.h"
#include "aabb2.h"
#include "clipper2.h"
#include "linesegment2.h"

namespace valhalla{
namespace geo{

/**
 * 2-D polyline
 */
class Polyline2 {
 public:
  Polyline2();

  /**
   * Constructor given a list of points.
   * @param  pts  List of points.
   */
  Polyline2(std::vector<Point2>& pts);

  // TODO - do we need copy constructor and = operator?

  /**
   * Gets the list of points.
   * @return  Returns the list of points.
   */
  std::vector<Point2>& pts();

  /**
   * Add a point to the polyline. Checks to see if the input point is
   * equal to the current endpoint of the polyline and does not add the
   * point if it is equal.
   */
  void Add(const Point2& p);

  /**
  * Finds the length of the polyline by accumulating the length of all
  * segments.
  * @return    Returns the length of the polyline.
  */
  float Length() const;

  /**
   * Compute the length of the polyline represented by a set of
   * lat,lng points. Avoids having to copy the points into the
   * polyline.
   * @param  pts  List of lat,lng points.
   * @return  Returns the length in kilometers
   */
  float Length(const std::vector<Point2>& pts) const;

  /**
   * Gets the closest point to this polyline from the specified point.
   * @param  pt      Point to find closest point to.
   * @param  closest (OUT) Closest point along the polyline
   * @param  idx     (OUT) Index of the segment of the polyline which contains
   *                       the closest point.
   * @return   Returns the distance squared of the closest point.
   */
  float ClosestPoint(const Point2& pt, Point2& nearest, int& idx) const;

  /**
   * Generalize this polyline.
   */
  unsigned int Generalize(const float t);

  /**
   * Get a generalized polyline from this polyline. This polyline remains
   * unchanged.
   * @param  t       Generalization tolerance.
   * @param  genpts  Points within a generalized polyline he generalized polyline.
   */
  Polyline2 GeneralizedPolyline(const float t);
  /**
   * Clip this polyline to the specified bounding box.
   * @param box  Bounding box to clip this polyline to.
   */
  unsigned int Clip(const AABB2& box);

  /**
   * Gets a polyline clipped to the supplied bounding box. This polyline
   * remains unchanged.
   */
   Polyline2 ClippedPolyline(const AABB2& box);

  // TODO - add Google polyline encoding

protected:
  // Polyline points
  std::vector<Point2> pts_;

  void DouglasPeucker(const unsigned int i, const unsigned int j,
                      const float t2, std::vector<Point2>& genpts);
};

}
}

#endif  // VALHALLA_GEO_POLYLINE2_H_
