#include "geo/pointll.h"
#include "geo/distanceapproximator.h"
#include "geo/vector2.h"

namespace valhalla{
namespace geo{

/**
 * Finds the closest point to the supplied polyline as well as the distance
 * squared to that point. Uses DistanceApproximator to set the point as the
 * test point - then uses DistanceSquared method to get more accurate
 * distances than just Euclidean distance in lat,lng space
 * @param  pts     Polyline points
 * @param  closest (OUT) Closest point along the polyline
 * @param  idx     (OUT) Index of the segment of the polyline which contains
 *                       the closest point.
 * @return   Returns the distance squared of the closest point.
 */
inline float PointLL::ClosestPoint(const std::vector<PointLL>& pts,
          PointLL& closest, int& idx) const {
  // TODO - make sure at least 2 points are in list

  DistanceApproximator approx;
  approx.SetTestPoint(*this);

  // Iterate through the pts
  unsigned int count = pts.size();
  bool beyond_end = true;               // Need to test past the end point?
  Vector2 v1;                                                     // Segment vector (v1)
  Vector2 v2;                                                     // Vector from origin to target (v2)
  PointLL projpt;           // Projected point along v1
  float dot;                                                      // Dot product of v1 and v2
  float comp;               // Component of v2 along v1
  float dist;               // Squared distance from target to closest point on line
  float mindist = 2147483647.0f;
  const PointLL* p0 = &pts[0];
  const PointLL* p1 = &pts[1];
  const PointLL* end = &pts[count - 1];
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
      dist = approx.DistanceSquared(*p0);
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
      // TODO - why not:  *p0 + v1 * comp;
      beyond_end = false;
      projpt.Set(p0->lat() + v1.y() * comp, p0->lng() + v1.x() * comp);
      dist = approx.DistanceSquared(projpt);
      if (dist < mindist) {
        mindist = dist;
        closest = projpt;
        idx     = index;
      }
    }
  }

  // Test the end point if flag is set - it may be the closest point
  if (beyond_end)  {
    dist = approx.DistanceSquared(pts[count-1]);
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
