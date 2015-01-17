#ifndef VALHALLA_MIDGARD_UTIL_H_
#define VALHALLA_MIDGARD_UTIL_H_

#include <string>

namespace valhalla {
namespace midgard {

// Intersection cases.
enum IntersectCase {
  kWithin,
  kContains,
  kOutside,
  kIntersects
};

/**
 * Compute time (seconds) given a length (km) and speed (km per hour)
 */
int GetTime(const float length, const float speed);

uint32_t GetTurnDegree(const uint32_t from_heading, const uint32_t to_heading);

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

/**
 * Polyline encode a container of points into a string
 * Note: newer versions of this algorithm allow one to specify a zoom level
 * which allows displaying simplified versions of the encoded linestring
 *
 * @param points    the list of points to encode
 * @return string   the encoded container of points
 */
template<class container_t>
std::string encode(const container_t& points);

/**
 * Polyline decode a string into a container of points
 *
 * @param string    the encoded points
 * @return points   the container of points
 */
template<class container_t>
container_t decode(const std::string& encoded);

}
}
#endif  // VALHALLA_MIDGARD_UTIL_H_
