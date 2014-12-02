#include "geo/util.h"
#include "geo/constants.h"

#include <math.h>
#include <stdlib.h>

namespace valhalla{
namespace geo{

inline int GetTime(const float length, const float speed) {
  return (int)(length / (speed * kHourPerSec) + 0.5f);
}

inline float degrees_to_radians(const float d) {
  return d * kDegPerRad;
}

inline float radians_to_degrees(const float r) {
  return r * kRadPerDeg;
}

inline float rand01() {
  return (float)rand() / (float)RAND_MAX;
}

inline float FastInvSqrt(float x) {
   float xhalf = 0.5f * x;
   int i = *(int*)&x;            // get bits for floating value
   i = 0x5f3759df - (i>>1);      // give initial guess y0
   x = *(float*)&i;              // convert bits back to float
   return x*(1.5f - xhalf*x*x);  // newton step
   // x *= 1.5f - xhalf*x*x;     // repeating step increases accuracy
}

inline float sqr(const float a) {
  return a * a;
}

}
}
