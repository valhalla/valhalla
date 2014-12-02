#include "geo/point2.h"
#include "geo/vector2.h"

namespace valhalla{
namespace geo{

/**
 * Add a vector to the current point.
 * @param   v  Vector to add to the current point.
 * @return  Returns a new point: the result of the current point
 *          plus the specified vector.
 */
Point2 Point2::operator + (const Vector2& v) const{
  return Point2(x_ + v.x(), y_ + v.y());
}

/**
 * Subtract a vector from the current point.
 * @param   v  Vector to subtract from the current point.
 * @return  Returns a new point: the result of the current point
 *          minus the specified vector.
 */
Point2 Point2::operator - (const Vector2& v) const{
  return Point2(x_ - v.x(), y_ - v.y());
}

/**
 * Subtraction of a point from the current point.
 * @param   Point to subtract from the current point.
 * @return  Returns a vector.
 */
Vector2 Point2::operator - (const Point2& p) const{
  return Vector2(x_ - p.x(), y_ - p.y());
}

/**
 * Finds the closest point to the supplied polyline as well as the distance
 * squared to that point.
 * @param  pts     Polyline points
 * @param  closest (OUT) Closest point along the polyline
 * @param  idx     (OUT) Index of the segment of the polyline which contains
 *                       the closest point.
 * @return   Returns the distance squared of the closest point.
 */
inline float Point2::ClosestPoint(const std::vector<Point2>& pts,
          Point2& closest, int& idx) const {
  // TODO - make sure at least 2 points are in list

  // Iterate through the pts
  unsigned int count = pts.size();
  bool beyond_end = true;               // Need to test past the end point?
  Vector2 v1;                                                     // Segment vector (v1)
  Vector2 v2;                                                     // Vector from origin to target (v2)
  Point2 projpt;            // Projected point along v1
  float dot;                                                      // Dot product of v1 and v2
  float comp;               // Component of v2 along v1
  float dist;               // Squared distance from target to closest point on line
  float mindist = 2147483647.0f;
  const Point2* p0 = &pts[0];
  const Point2* p1 = &pts[1];
  const Point2* end = &pts[count - 1];
  for (int index = 0; p1 < end; index++, p0++, p1++) {
    // Construct vector v1 - represents the segment.  Skip 0 length segments
    v1.Set(*p0, *p1);
    if (v1.x() == 0.0f && v1.y() == 0.0f)
      continue;

    // Vector v2 from the segment origin to the target point
    v2.Set(*p0, *this);

    // Find the dot product of v1 and v2.  If less than 0 the segment
    // origin is the closest point.  Find the distance and continue
    // to the next segment.
    dot = v1.Dot(v2);
    if (dot <= 0.0f) {
      beyond_end = false;
      dist = DistanceSquared(*p0);
      if (dist < mindist) {
        mindist = dist;
        closest  = *p0;
        idx      = index;
      }
      continue;
    }

    // Closest point is either beyond the end of the segment or at a point
    // along the segment. Find the component of v2 along v1
    comp = dot / v1.Dot(v1);

    // If component >= 1.0 the segment end is the closest point. A future
    // polyline segment will be closer.  If last segment we need to check
    // distance to the endpoint.  Set flag so this happens.
    if (comp >= 1.0f)
      beyond_end = true;
    else {
      // Closest point is along the segment.  The closest point is found
      // by adding the projection of v2 onto v1 to the origin point.
      // The squared distance from this point to the target is then found.
      beyond_end = false;
      projpt = *p0 + v1 * comp;
      dist = DistanceSquared(projpt);
      if (dist < mindist) {
        mindist = dist;
        closest = projpt;
        idx     = index;
      }
    }
  }

  // Test the end point if flag is set - it may be the closest point
  if (beyond_end)  {
    dist = DistanceSquared(pts[count-1]);
    if (dist < mindist) {
      mindist = dist;
      closest = *end;
      idx     = (int)(count-2);
    }
  }
  return mindist;
}


}
}
