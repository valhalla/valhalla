#ifndef __geosupport_h__
#define __geosupport_h__

#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

// Time constants
const float SEC_PER_HOUR = 3600.0f;
const float HOUR_PER_SEC = 1.0f / 3600.0f;

/**
 * Compute time (seconds) given a length (km) and speed (km per hour)
 */
inline int GetTime(const float length, const float speed) {
  return (int)(length / (speed * HOUR_PER_SEC) + 0.5f);
}

// Distance constants
const float MILES_TO_KM   = 1.609344f;
const float KM_TO_MILES   = 1.0f / MILES_TO_KM;
const float RAD_EARTH_KM  = 6378.160187;
const float KM_PER_DEGREE = 110.567f;

// Angular measures
const float PI_F    = 3.14159265f;
const float PIOVER2 = (PI_F * 0.5f);
const float PIOVER4 = (PI_F * 0.25f);
const float D2R     = (PI_F / 180.0f);    // Degrees to radians conversion
const float R2D     = (180.0f / PI_F);    // Radians to degrees conversion

const float EPSILON = 0.000001f;

// TODO - remove!??
#define MAXV(a,b) (((a) > (b)) ? (a) : (b))
#define MINV(a,b) (((a) < (b)) ? (a) : (b))
#define SQR(x)    ((x) * (x))

// Intersection cases.
enum IntersectCase { WITHIN, CONTAINS, OUTSIDE, INTERSECTS };

/**
 * Degrees to radians conversion
 * @param   d   Angle in degrees.
 * @return  Returns the angle in radians.
 */
inline float degrees_to_radians(const float d) {
  return d * D2R;
}

/**
 * Radians to degrees conversion
 * @param   r   Angle in radians.
 * @return   Returns the angle in degrees.
 */
inline float radians_to_degrees(const float r) {
  return r * R2D;
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
