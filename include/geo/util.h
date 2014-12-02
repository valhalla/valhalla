#ifndef __geo_h__
#define __geo_h__

#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

#include "constants.h"

namespace valhalla{
namespace geo{

/**
 * Compute time (seconds) given a length (km) and speed (km per hour)
 */
inline int GetTime(const float length, const float speed) {
  return (int)(length / (speed * kHourPerSec) + 0.5f);
}

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

// Convenience method.
inline float sqr(const float a) {
  return a * a;
}

}
}
#endif
