#ifndef __geo_h__
#define __geo_h__

#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

// Include individual geo files
#include "point2.h"
#include "pointll.h"
#include "vector2.h"
#include "aabb2.h"
#include "aabbll.h"
#include "distanceapproximator.h"
#include "segment2.h"
#include "obb2.h"
#include "ellipse.h"
#include "clipper2.h"
#include "tiles.h"
#include "polyline2.h"

namespace valhalla{
namespace geo{

// Time constants
constexpr float kSecPerHour = 3600.0f;
constexpr float kHourPerSec = 1.0f / 3600.0f;

/**
 * Compute time (seconds) given a length (km) and speed (km per hour)
 */
inline int GetTime(const float length, const float speed) {
  return (int)(length / (speed * kHourPerSec) + 0.5f);
}

// Distance constants
constexpr float kMilePerKm      = 1.609344f;
constexpr float kKmPerMile      = 1.0f / kMilePerKm;
constexpr float kRadEarthKm     = 6378.160187;
constexpr float kKmPerDegreeLat = 110.567f;

// Angular measures
constexpr float kPi        = 3.14159265f;
constexpr float kPiOver2   = (kPi * 0.5f);
constexpr float kPiOver4   = (kPi * 0.25f);
constexpr float kDegPerRad = (kPi / 180.0f);    // Degrees to radians conversion
constexpr float kRadPerDeg = (180.0f / kPi);    // Radians to degrees conversion
constexpr float kEpsilon   = 0.000001f;

// Intersection cases.
enum IntersectCase { kWithin, kContains, kOutside, kIntersects };

/**
 * Degrees to radians conversion
 * @param   d   Angle in degrees.
 * @return  Returns the angle in radians.
 */
inline float degrees_to_radians(const float d) {
  return d * kDegPerRad;
}

/**
 * Radians to degrees conversion
 * @param   r   Angle in radians.
 * @return   Returns the angle in degrees.
 */
inline float radians_to_degrees(const float r) {
  return r * kRadPerDeg;
}

/**
 * Get a random number between 0 and 1
 */
inline float rand01() {
  return (float)rand() / (float)RAND_MAX;
}

/**
 * Fast inverse sqrt method. Originally used in Quake III
 * @param   x  Value to find inverse sqrt for
 * @return  Returns 1/sqrtf(x)
 */
inline float FastInvSqrt(float x) {
   float xhalf = 0.5f * x;
   int i = *(int*)&x;            // get bits for floating value
   i = 0x5f3759df - (i>>1);      // give initial guess y0
   x = *(float*)&i;              // convert bits back to float
   return x*(1.5f - xhalf*x*x);  // newton step
   // x *= 1.5f - xhalf*x*x;     // repeating step increases accuracy
}



// Define the operators in Point2 that allow a vector to be added
// to and subtracted from a point. Descriptions are in the Point2
// structure.
inline Point2 Point2::operator + (const Vector2& w) const {
  return Point2(x_ + w.x(), y_ + w.y());
}
inline Point2 Point2::operator - (const Vector2 &w) const {
  return Point2(x_ - w.x(), y_ - w.y());
}
inline Vector2 Point2::operator - (const Point2& p) const {
  return Vector2(x_ - p.x(), y_ - p.y());
}

/**
 * Overloading: allows float * Vector2
 */
inline Vector2 operator *(float s, const Vector2 &v){
  return Vector2(v.x() * s, v.y() * s);
}

/**
 * TODO - move to .cc file!
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
  bool beyond_end = true;		// Need to test past the end point?
  Vector2 v1;					      // Segment vector (v1)
  Vector2 v2;					      // Vector from origin to target (v2)
  Point2 projpt;            // Projected point along v1
  float dot;					      // Dot product of v1 and v2
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

/**
 * TODO - move to .cc file!
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
  bool beyond_end = true;		// Need to test past the end point?
  Vector2 v1;					      // Segment vector (v1)
  Vector2 v2;					      // Vector from origin to target (v2)
  PointLL projpt;           // Projected point along v1
  float dot;					      // Dot product of v1 and v2
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
#endif
