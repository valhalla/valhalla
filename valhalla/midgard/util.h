#ifndef VALHALLA_MIDGARD_UTIL_H_
#define VALHALLA_MIDGARD_UTIL_H_

#include <string>
#include <stdexcept>
#include <string>
#include <ostream>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <memory>

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
 * @param  length  distance in km.
 * @param  speed  in km per hour.
 * @return the computed time in seconds.
 */
int GetTime(const float length, const float speed);

/**
 * Computes the turn degree based on the specified "from heading" and
 * "to heading"
 * @param  from_heading  heading at the end of the "from" edge.
 * @param  to_heading  heading at the begin of the "to" edge.
 * @return the computed turn degree. For example, if one would make a perfect
 *         right turn - the returned value would be 90.
 */
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
template<class T>
T sqr(const T a) {
  return a * a;
}

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

//useful in converting from one iteratable map to another
//for example: ToMap<boost::property_tree::ptree, std::unordered_map<std::string, std::string> >(some_ptree)
/*
 * @param inmap the map to be converted
 * @return the converted map of another type
 */
template <class T1, class T2>
T2 ToMap(const T1& inmap) {
  T2 outmap;
  for(const auto& key_value : inmap)
    outmap[key_value.first] = key_value.second.data();
  return outmap;
}

/**
 * equals with an epsilon for approximation
 * @param first operand
 * @param second operand
 * @param epsilon to help with approximate equality
 */
template <class T>
bool equal(const T a, const T b, const T epsilon = static_cast<T>(.00001)) {
  if(epsilon < static_cast<T>(0))
    throw std::logic_error("Using a negative epsilon is not supported");
  T diff = a - b;
  //if its non-negative it better be less than epsilon, if its negative then it better be bigger than epsilon
  bool negative = diff < static_cast<T>(0);
  return (!negative && diff <= epsilon) || (negative && diff >= -epsilon);
}

/**
 * A means by which you can get some information about the current processes memory footprint
 */
struct memory_status {
  memory_status() = delete;
  memory_status(const std::unordered_set<std::string> interest = std::unordered_set<std::string>{});

  std::unordered_map<std::string, std::pair<double, std::string> > metrics;

  friend std::ostream& operator<<(std::ostream&, const memory_status&);
};
std::ostream& operator<<(std::ostream& stream, const memory_status& s);

/**
 * Implement the missing make_unique for C++11.
 */
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>{new T{std::forward<Args>(args)...}};
}

/* circular range clamp
 */
template <class T>
T circular_range_clamp(T value, T lower, T upper) {
  //yeah..
  if(lower >= upper)
    throw std::runtime_error("invalid range for clamp");

  //easy case
  if(lower <= value && value <= upper)
    return value;

  //see how far off the bottom of the range it is
  auto i = upper - lower;
  if(value < lower) {
    auto d = lower - value;
    d -= (static_cast<int>(d / i) * i);
    return upper - d;
  }

  //its past the top of the range
  auto d = value - upper;
  d -= (static_cast<int>(d / i) * i);
  return lower + d;
}

}
}
#endif  // VALHALLA_MIDGARD_UTIL_H_
