#ifndef __geosupport_h__
#define __geosupport_h__

#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

// Time constants
const float kSecPerHour = 3600.0f;
const float kHourPerSec = 1.0f / 3600.0f;

/**
 * Compute time (seconds) given a length (km) and speed (km per hour)
 */
inline int GetTime(const float length, const float speed) {
  return (int)(length / (speed * kHourPerSec) + 0.5f);
}

// Distance constants
const float kMilePerKm      = 1.609344f;
const float kKmPerMile      = 1.0f / kMilePerKm;
const float kRadEarthKm     = 6378.160187;
const float kKmPerDegreeLat = 110.567f;

// Angular measures
const float kPi        = 3.14159265f;
const float kPiOver2   = (kPi * 0.5f);
const float kPiOver4   = (kPi * 0.25f);
const float kDegPerRad = (kPi / 180.0f);    // Degrees to radians conversion
const float kRadPerDeg = (180.0f / kPi);    // Radians to degrees conversion
const float kEpsilon   = 0.000001f;

// TODO - remove!??
#define MAXV(a,b) (((a) > (b)) ? (a) : (b))
#define MINV(a,b) (((a) < (b)) ? (a) : (b))
#define SQR(x)    ((x) * (x))

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

// Include individual geosupport files
#include "point2.h"
#include "pointll.h"
#include "vector2.h"

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

#endif
