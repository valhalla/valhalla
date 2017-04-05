#ifndef VALHALLA_MIDGARD_UTIL_H_
#define VALHALLA_MIDGARD_UTIL_H_

#include <string>
#include <stdexcept>
#include <string>
#include <ostream>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <memory>
#include <limits>

#include <valhalla/midgard/pointll.h>


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
 * Trim the front of a polyline (represented as a list or vector of Point2).
 * Returns the trimmed portion of the polyline. The supplied polyline is
 * altered (the trimmed part is removed).
 * @param  pts    List of points. This is modified - the result is the
 *                remaining points after trimming the front.
 * @param  dist   Distance to trim.
 * @return Returns a list of points along the supplied polyline. The total
 *         length of the returned polyline is dist.
 */
template <class container_t>
container_t trim_front(container_t& pts, const float dist);

// Compute the length of the polyline represented by a set of lat,lng points.
// Avoids having to copy the points into a polyline, polyline should really just extend
// A container class like vector or list
template <class container_t>
float length(const container_t& pts) {
 float length = 0.0f;
 for(auto p = std::next(pts.cbegin()); p != pts.end(); ++p)
   length += p->Distance(*std::prev(p));
 return length;
}

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

  static bool supported();

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

/**
 * standard clamp
 */
template <class T>
T clamp(T value, T lower, T upper) {
  return std::max(std::min(value, upper), lower);
}

/**
 * Resample a polyline in spherical coordinates to specified resolution optionally keeping all original points in the line
 * @param polyline     the list/vector of points in the line
 * @param resolution   maximum distance between any two points in the resampled line
 * @param preserve     keep input points in resampled line or not
 */
template<class container_t>
container_t resample_spherical_polyline(const container_t& polyline, double resolution, bool preserve = false);

/**
 * A class to wrap a primitive array in something iterable which is useful for loops mostly
 * Basically if you dont have a vector or list, this makes your array a bit more usable in
 * that it fakes up a container for the purpose of ripping through the array
 *
 * TODO: reverse iteration
 */
template <class T>
struct iterable_t {
 public:
  using iterator = T*;
  iterable_t(T* first, size_t size): head(first), tail(first + size), count(size){}
  iterable_t(T* first, T* end): head(first), tail(end), count(end - first){}
  T* begin() { return head; }
  T* end() { return tail; }
  T& operator[](size_t index){ return *(head + index); }
  size_t size() const { return count; }
 protected:
  T* head;
  T* tail;
  size_t count;
};

/**
 * Return the intersection of two infinite lines if any
 * @param u  first point on first line
 * @param v  second point on first line
 * @param a  first point on second line
 * @param b  second point on second line
 * @param i  the intersection point if there was one
 * @return true if there was an intersection false if now
 */
template <class coord_t>
bool intersect(const coord_t& u, const coord_t& v, const coord_t& a, const coord_t& b, coord_t& i);

/**
 * Return the intercept of the line passing through uv with the horizontal line defined by y
 * @param u  first point on line
 * @param v  second point on line
 * @param y  y component of horizontal line
 * @return x component (or NaN if parallel) of the intercept of uv with the horizontal line
 */
template <class coord_t>
typename coord_t::first_type y_intercept(const coord_t& u, const coord_t& v, const typename coord_t::second_type y = 0);
/**
 * Return the intercept of the line passing through uv with the vertical line defined by x
 * @param u  first point on line
 * @param v  second point on line
 * @param x  x component of vertical line
 * @return y component (or NaN if parallel) of the intercept of uv with the vertical line
 */
template <class coord_t>
typename coord_t::first_type x_intercept(const coord_t& u, const coord_t& v, const typename coord_t::second_type x = 0);

/**
 * Compute the area of a polygon. If your polygon is not twisted or self intersecting
 * this will return a positive value for clockwise wound polygons and negative otherwise.
 * Works with rings where the polygons first and last points are the same or not
 *
 * NOTE: this is good for relative area but the units for spherical coordinates
 * are in spherical coordinates treated as euclidean space
 *
 * @param polygon   the list of points comprising the polygon
 * @return the area of the polygon
 */
template <class container_t>
float polygon_area(const container_t& polygon);

}
}
#endif  // VALHALLA_MIDGARD_UTIL_H_
