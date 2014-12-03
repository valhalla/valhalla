#ifndef __geo_h__
#define __geo_h__

namespace valhalla{
namespace geo{

// Intersection cases.
enum IntersectCase { kWithin, kContains, kOutside, kIntersects };

/**
 * Compute time (seconds) given a length (km) and speed (km per hour)
 */
int GetTime(const float length, const float speed);

/**
 * Degrees to radians conversion
 * @param   d   Angle in degrees.
 * @return  Returns the angle in radians.
 */
//float degrees_to_radians(const float d);

/**
 * Radians to degrees conversion
 * @param   r   Angle in radians.
 * @return   Returns the angle in degrees.
 */
//float radians_to_degrees(const float r);

/**
 * Get a random number between 0 and 1
 */
float rand01();

/**
 * Fast inverse sqrt method. Originally used in Quake III
 * @param   x  Value to find inverse sqrt for
 * @return  Returns 1/sqrtf(x)
 */
float FastInvSqrt(float x);

// Convenience method.
float sqr(const float a);

}
}
#endif
