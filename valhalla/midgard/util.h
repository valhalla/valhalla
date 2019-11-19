#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <ostream>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/tiles.h>

namespace valhalla {
namespace midgard {

// Holds a range plus a default value for that range
template <class T> struct ranged_default_t {
  T min, def, max;

  // Returns the value snapped to the default if outside of the range
  T operator()(const T& value) const {
    if (value < min || value > max) {
      return def;
    }
    return value;
  }
};

// Intersection cases.
enum IntersectCase { kWithin, kContains, kOutside, kIntersects };

/**
 * Compute time (seconds) given a length (km) and speed (km per hour)
 * @param  length  distance in km.
 * @param  speed  in km per hour.
 * @return the computed time in seconds.
 */
inline int GetTime(const float length, const float speed) {
  return (speed > 0.0f) ? static_cast<int>((length / (speed * kHourPerSec) + 0.5f)) : 0;
}

/**
 * Computes the turn degree based on the specified "from heading" and
 * "to heading"
 * @param  from_heading  heading at the end of the "from" edge.
 * @param  to_heading  heading at the begin of the "to" edge.
 * @return the computed turn degree. For example, if one would make a perfect
 *         right turn - the returned value would be 90.
 */
inline uint32_t GetTurnDegree(const uint32_t from_heading, const uint32_t to_heading) {
  return (((to_heading - from_heading) + 360) % 360);
}

/**
 * Compute the turn degree (from 0 to 180) - used in meili (map-matching)
 * @param inbound  Inbound heading
 * @param outbound Outbound heading
 * @return  Returns the turn degree (0-180 degrees).
 */
inline uint8_t get_turn_degree180(const uint16_t inbound, const uint16_t outbound) {
  // TODO - do we need bounds checking here?
  if (!(inbound < 360 && outbound < 360)) {
    throw std::invalid_argument("expect angles to be within [0, 360)");
  }
  const auto turn = std::abs(inbound - outbound);
  return 180 < turn ? 360 - turn : turn;
}

/**
 * Expand an input lat,lon bounding box by the specified distance in meters.
 * @param   box     Bounding box.
 * @param   meters  Meters to expand the bounds
 * @return  Returns the bounding box.
 */
inline AABB2<PointLL> ExpandMeters(const AABB2<PointLL>& box, const float meters) {
  if (meters < 0.f) {
    throw std::invalid_argument("expect non-negative meters");
  }

  // Find the delta latitude and delta longitude (max of the delta at the
  // minimum and maximum latitude)
  float dlat = meters / kMetersPerDegreeLat;
  float dlng1 = (meters / DistanceApproximator::MetersPerLngDegree(box.miny()));
  float dlng2 = (meters / DistanceApproximator::MetersPerLngDegree(box.maxy()));
  float dlng = std::max(dlng1, dlng2);
  return {box.minx() - dlng, box.miny() - dlat, box.maxx() + dlng, box.maxy() + dlat};
}

/**
 * Create a lat,lon bounding box around the specified point - with a distance
 * from the center specified in meters.
 * @param  pt  Lat,lon point to use as the bounding box center.
 * @param  meters  Distance in meters from center to edge.
 * @return Returns the bounding box.
 */
inline AABB2<PointLL> ExpandMeters(const PointLL& pt, const float meters) {
  if (meters < 0.f) {
    throw std::invalid_argument("expect non-negative meters");
  }

  float dlat = meters / kMetersPerDegreeLat;
  float dlng = meters / DistanceApproximator::MetersPerLngDegree(pt.lat());
  PointLL minpt(pt.lng() - dlng, pt.lat() - dlat);
  PointLL maxpt(pt.lng() + dlng, pt.lat() + dlat);
  return {minpt, maxpt};
}

// Convenience method.
template <class T> T sqr(const T a) {
  return a * a;
}

/**
 * Normalize a ratio and clamp to range [0, 1]. Protect against division by 0.
 * @param  num  Numerator
 * @param  den  Denominator
 * @return Returns the ration clamped to range [0,1]
 */
inline float normalize(const float num, const float den) {
  return 0.f == den ? 0.0f : std::min(std::max(num / den, 0.0f), 1.0f);
}

// Compute the length of the polyline represented by a set of lat,lng points.
// Avoids having to copy the points into a polyline, polyline should really just extend
// A container class like vector or list
template <class container_t> float length(const container_t& pts) {
  if (pts.size() < 2) {
    return 0.0f;
  }
  float length = 0.0f;
  for (auto p = std::next(pts.cbegin()); p != pts.end(); ++p) {
    length += p->Distance(*std::prev(p));
  }
  return length;
}

/**
 * Compute the length of a polyline between the 2 specified iterators.
 * @param  begin  Starting point (iterator) within the polyline container.
 * @param  end    Ending point (iterator) within the polyline container.
 * @return Returns the length of the polyline.
 */
template <typename iterator_t> float length(const iterator_t& begin, const iterator_t& end) {
  if (begin == end) {
    return 0.0f;
  }

  float length = 0.0f;
  for (auto vertex = std::next(begin); vertex != end; vertex++) {
    length += std::prev(vertex)->Distance(*vertex);
  }
  return length;
}

/**
 * Create a new polyline by trimming an input polyline by a specified
 * percentage from the start iterator and a specified percentage from
 * the end iterator.
 * @param  begin  Starting point (iterator) within the polyline container.
 * @param  end    Ending point (iterator) within the polyline container.
 * @param  source Percentage of total length to trim from the front.
 * @param  target Percentage of total length to trim from the end.
 * @return Returns a new polyline.
 */
template <typename iterator_t>
std::vector<typename iterator_t::value_type>
trim_polyline(const iterator_t& begin, const iterator_t& end, float source, float target) {
  // Detect invalid cases
  if (target < source || target < 0.f || 1.f < source || begin == end) {
    return {};
  }

  // Clamp source and target to range [0, 1]
  source = std::min(std::max(source, 0.f), 1.f);
  target = std::min(std::max(target, 0.f), 1.f);

  float total_length = length(begin, end), prev_vertex_length = 0.f,
        source_length = total_length * source, target_length = total_length * target;

  // An state indicating if the position of current vertex is larger
  // than source and smaller than target
  bool open = false;

  // Iterate segments and add to output container (clip)
  std::vector<typename iterator_t::value_type> clip;
  iterator_t prev_vertex = begin;
  for (auto vertex = std::next(begin); vertex != end; vertex++) {
    const auto segment_length = prev_vertex->Distance(*vertex),
               vertex_length = prev_vertex_length + segment_length;

    // Open if source is located at current segment
    if (!open && source_length < vertex_length) {
      const auto offset = normalize(source_length - prev_vertex_length, segment_length);
      clip.push_back(prev_vertex->along_segment(*vertex, offset));
      open = true;
    }

    // Open -> Close if target is located at current segment
    if (open && target_length < vertex_length) {
      const auto offset = normalize(target_length - prev_vertex_length, segment_length);
      clip.push_back(prev_vertex->along_segment(*vertex, offset));
      open = false;
      break;
    }

    // Add the end vertex of current segment if it is in open state
    if (open) {
      clip.push_back(*vertex);
    }

    prev_vertex = vertex;
    prev_vertex_length = vertex_length;
  }
  // assert(clip.size() != 1) because when opening state, the source
  // vertex was inserted and then followed by either target vertex
  // inserted or current vertex inserted

  if (clip.empty()) {
    // assert(1.f == source && 1.f == target)
    clip.push_back(*prev_vertex);
    clip.push_back(*prev_vertex);
  }

  // So here we have assert(1 < clip.size())
  return clip;
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
template <class container_t> container_t trim_front(container_t& pts, const float dist);

/**
 * Estimate the angle of the tangent at a point along a discretised curve. We attempt
 * to mostly use the shape coming into the point on the curve but if there
 * isn't enough there we will use the shape coming out of the it.
 * @param index Index into the shape.
 * @param point Point to test for tangent along the curve.
 * @param shape  Shape / polyline geometry.
 * @param sample_distance Distance to sample when computing heading.
 * @param forward Boolean value whether to test in forward or reverse direction.
 * @return Returns the angle in degrees relative to N.
 */
float tangent_angle(size_t index,
                    const PointLL& point,
                    const std::vector<PointLL>& shape,
                    const float sample_distance,
                    bool forward);

// useful in converting from one iteratable map to another
// for example: ToMap<boost::property_tree::ptree, std::unordered_map<std::string, std::string>
// >(some_ptree)
/*
 * @param inmap the map to be converted
 * @return the converted map of another type
 */
template <class T1, class T2> inline T2 ToMap(const T1& inmap) {
  T2 outmap;
  for (const auto& key_value : inmap) {
    outmap[key_value.first] = key_value.second.data();
  }
  return outmap;
}

// useful in converting from one iterable set to another
// for example ToSet<boost::property_tree::ptree, std::unordered_set<std::string> >(some_ptree)
/*
 * @param inset the set to be converted
 * @return the converted set of another type
 */
template <class T1, class T2> inline T2 ToSet(const T1& inset) {
  T2 outset;
  for (const auto& item : inset) {
    outset.emplace(item.second.template get_value<typename T2::value_type>());
  }
  return outset;
}

/**
 * equals with an epsilon for approximation
 * @param a first operand
 * @param b second operand
 * @param epsilon to help with approximate equality
 */
template <class T> bool equal(const T a, const T b, const T epsilon = static_cast<T>(.00001)) {
  if (epsilon < static_cast<T>(0)) {
    throw std::logic_error("Using a negative epsilon is not supported");
  }
  T diff = a - b;
  // if its non-negative it better be less than epsilon, if its negative then it better be bigger
  // than epsilon
  bool negative = diff < static_cast<T>(0);
  return (!negative && diff <= epsilon) || (negative && diff >= -epsilon);
}

template <class T> bool similar(const T a, const T b, const double similarity = .99) {
  if (a == 0 || b == 0) {
    return a == b;
  }
  if ((a < 0) != (b < 0)) {
    return false;
  }
  return (double)std::min(a, b) / (double)std::max(a, b) >= similarity;
}

/**
 * A means by which you can get some information about the current processes memory footprint
 */
struct memory_status {
  memory_status() = delete;
  memory_status(const std::unordered_set<std::string>& interest = std::unordered_set<std::string>{});

  std::unordered_map<std::string, std::pair<double, std::string>> metrics;

  static bool supported();

  friend std::ostream& operator<<(std::ostream&, const memory_status&);
};
std::ostream& operator<<(std::ostream& stream, const memory_status& s);

/* circular range clamp
 */
template <class T> T circular_range_clamp(T value, T lower, T upper) {
  // yeah..
  if (lower >= upper) {
    throw std::runtime_error("invalid range for clamp");
  }

  // easy case
  if (lower <= value && value <= upper) {
    return value;
  }

  // see how far off the bottom of the range it is
  auto i = upper - lower;
  if (value < lower) {
    auto d = lower - value;
    d -= (static_cast<int>(d / i) * i);
    return upper - d;
  }

  // its past the top of the range
  auto d = value - upper;
  d -= (static_cast<int>(d / i) * i);
  return lower + d;
}

/**
 * standard clamp
 */
template <class T> T clamp(T value, T lower, T upper) {
  return std::max(std::min(value, upper), lower);
}

/**
 * Resample a polyline in spherical coordinates to specified resolution optionally keeping all
 * original points in the line
 * @param polyline     the list/vector of points in the line
 * @param resolution   maximum distance between any two points in the resampled line
 * @param preserve     keep input points in resampled line or not
 */
template <class container_t>
container_t
resample_spherical_polyline(const container_t& polyline, double resolution, bool preserve = false);

/**
 * Resample a polyline to the specified resolution. This is less precise than the spherical
 * resampling.
 * @param polyline     vector of points in the line
 * @param length       length of the polyline
 * @param resolution   desired resolution(meters) between any two points in the resampled line.
 *                     The polyline is sampled equally at a spacing that is close to the resolution.
 * @return Returns a vector of resampled points.
 */
std::vector<PointLL>
resample_polyline(const std::vector<PointLL>& polyline, const float length, const float resolution);

/**
 * A class to wrap a primitive array in something iterable which is useful for loops mostly
 * Basically if you dont have a vector or list, this makes your array a bit more usable in
 * that it fakes up a container for the purpose of ripping through the array
 *
 * TODO: reverse iteration
 */
template <class T> struct iterable_t {
public:
  using iterator = T*;
  iterable_t(T* first, size_t size) : head(first), tail(first + size), count(size) {
  }
  iterable_t(T* first, T* end) : head(first), tail(end), count(end - first) {
  }
  T* begin() {
    return head;
  }
  T* end() {
    return tail;
  }
  T& operator[](size_t index) {
    return *(head + index);
  }
  size_t size() const {
    return count;
  }

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
typename coord_t::first_type
y_intercept(const coord_t& u, const coord_t& v, const typename coord_t::second_type y = 0);
/**
 * Return the intercept of the line passing through uv with the vertical line defined by x
 * @param u  first point on line
 * @param v  second point on line
 * @param x  x component of vertical line
 * @return y component (or NaN if parallel) of the intercept of uv with the vertical line
 */
template <class coord_t>
typename coord_t::first_type
x_intercept(const coord_t& u, const coord_t& v, const typename coord_t::second_type x = 0);

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
template <class container_t> float polygon_area(const container_t& polygon);

template <typename T> struct ring_queue_t {
  ring_queue_t(size_t limit) : limit(limit), i(0) {
    v.reserve(limit);
  }
  void emplace_back(T&& t) {
    if (v.size() < limit) {
      v.emplace_back(t);
    } else {
      v[i] = t;
    }
    i = (i + 1) % limit;
  };
  const T& front() const {
    return i < v.size() ? v[i] : v[0];
  }
  const T& back() const {
    return v[i - 1];
  }
  size_t size() const {
    return v.size();
  }
  bool full() const {
    return v.size() == limit;
  }

  size_t limit, i;
  std::vector<T> v;

  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;
  iterator begin() {
    return v.begin();
  }
  const_iterator begin() const {
    return v.begin();
  }
  iterator end() {
    return v.end();
  }
  const_iterator end() const {
    return v.end();
  }
};

struct gps_segment_t {
  std::vector<midgard::PointLL> shape;
  float speed; // in meters/second
};

/**
 * Generate a gps path from an original path (with speeds) adding in some random noise
 *
 * @param segments    The representation of the segments of the original path including a speed per
 * segment this is usually maneuvers or edges in a route path generated by thor
 * @param accuracies  The returned minimum accuracy per simulated point, that is the simulated point
 * should be no more than accuracy from the original point (after resampling, see below)
 * @param smoothing   Controls the variability of adjacent simulated points by smoothing the error
 * over a window of simulated points. In this case smoothing determines the window size
 * @param accuracy    The maximum noise (arc distance) an one simulated point can exhibit with
 * respect to the original point (after resampling, see below)
 * @param sample_rate The final sample rate for the returned points. The determines how much time,
 *                    as a function of the segments speed, passes between individual simulated
 * @param seed        The seed to use for random number generation
 * points
 */
std::vector<midgard::PointLL> simulate_gps(const std::vector<gps_segment_t>& segments,
                                           std::vector<float>& accuracies,
                                           float smoothing = 30,
                                           float accuracy = 10.f,
                                           size_t sample_rate = 1,
                                           unsigned seed = 0);

/**
 * Generate a polygon geometry from a list of tile ids within a tileset
 *
 * @param region  the list of tiles form the tile set to be joined into a single geometry
 * @param tiles   the geometry defining the full set of tiles so we can gind the four conerers of
 * individual tiles
 * @return  a list of polygon rings
 */
using ring_t = std::list<PointLL>;
using polygon_t = std::list<ring_t>;
polygon_t to_boundary(const std::unordered_set<uint32_t>& region, const Tiles<PointLL>& tiles);

/**
 * A place where we can share the projecting of a single point onto any number of geometries
 * where the point is long lived and we survey many many shape segments such as is done in
 * both loki and in meili
 * */
struct projector_t {
  projector_t(const PointLL& ll)
      : lon_scale(cos(ll.lat() * kRadPerDegD)), lat(ll.lat()), lng(ll.lng()), approx(ll) {
  }

  // non default constructible and move only type
  projector_t() = delete;
  projector_t(const projector_t&) = delete;
  projector_t& operator=(const projector_t&) = delete;
  projector_t(projector_t&&) = default;
  projector_t& operator=(projector_t&&) = default;

  // Test if a segment is a candidate to the projection.  This method
  // is performance critical.  Copy, function call, cache locality and
  // useless computation must be handled with care.
  inline PointLL operator()(const PointLL& u, const PointLL& v) const {
    // we're done if this is a zero length segment
    if (u == v) {
      return u;
    }

    // project a onto b where b is the origin vector representing this segment
    // and a is the origin vector to the point we are projecting, (a.b/b.b)*b
    auto bx = double(v.first) - u.first;
    auto by = double(v.second) - u.second;

    // Scale longitude when finding the projection
    auto bx2 = bx * lon_scale;
    auto sq = bx2 * bx2 + by * by;
    auto scale =
        (lng - u.lng()) * lon_scale * bx2 + (lat - u.lat()) * by; // only need the numerator at first

    // projects along the ray before u
    if (scale <= 0.0) {
      return u;
      // projects along the ray after v
    } else if (scale >= sq) {
      return v;
    }
    // projects along the ray between u and v
    scale /= sq;
    return {u.first + bx * scale, u.second + by * scale};
  }

  // critical data
  double lon_scale;
  double lat;
  double lng;
  DistanceApproximator approx;
};
} // namespace midgard
} // namespace valhalla
