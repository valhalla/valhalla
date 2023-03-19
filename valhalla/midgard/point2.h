#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <valhalla/midgard/util_core.h>

namespace {
constexpr float LL_EPSILON = .00002f;
}

namespace valhalla {
namespace midgard {

// Forward references
template <typename PrecisionT> class VectorXY;

/**
 * 2D Point (cartesian). float x,y components.
 * @author David W. Nesbitt
 */
template <typename PrecisionT> class PointXY : public std::pair<PrecisionT, PrecisionT> {

public:
  /**
   * Use the constructors provided by pair
   */
  using std::pair<PrecisionT, PrecisionT>::pair;
  using std::pair<PrecisionT, PrecisionT>::first;
  using std::pair<PrecisionT, PrecisionT>::second;
  using typename std::pair<PrecisionT, PrecisionT>::first_type;
  using typename std::pair<PrecisionT, PrecisionT>::second_type;
  using value_type = typename std::pair<PrecisionT, PrecisionT>::first_type;

  virtual ~PointXY() = default;

  /**
   * Get the x component.
   * @return  Returns the x component of the point.
   */
  PrecisionT x() const {
    return first;
  }

  /**
   * Get the y component.
   * @return  Returns the y component of the point.
   */
  PrecisionT y() const {
    return second;
  }

  /**
   * Set the x component.
   * @param  x  x coordinate value.
   */
  void set_x(const PrecisionT x) {
    first = x;
  }

  /**
   * Set the y component.
   * @param  y  y coordinate value.
   */
  void set_y(const PrecisionT y) {
    second = y;
  }

  /**
   * Set the coordinate components to the specified values.
   * @param   x   x coordinate position.
   * @param   y   y coordinate position.
   */
  virtual void Set(const PrecisionT x, const PrecisionT y) {
    first = x;
    second = y;
  }

  /**
   * Equality approximation.
   * @param   p  Point to compare to the current point.
   * @param   e  An epsilon which determines how close they must be to be considered equal
   * @return  Returns true if two points are approximately equal, false otherwise.
   */
  bool ApproximatelyEqual(const PointXY<PrecisionT>& p, PrecisionT e = LL_EPSILON) const {
    return equal<value_type>(first, p.first, e) && equal<second_type>(second, p.second, e);
  }

  /**
   * Get the distance squared from this point to point p.
   * @param   p  Other point.
   * @return  Returns the distance squared between this point and p.
   */
  PrecisionT DistanceSquared(const PointXY<PrecisionT>& p) const {
    auto a = first - p.first;
    auto b = second - p.second;
    return a * a + b * b;
  }

  /**
   * Get the distance from this point to point p.
   * @param   p  Other point.
   * @return  Returns the distance between this point and p.
   */
  PrecisionT Distance(const PointXY<PrecisionT>& p) const {
    return sqrtf(DistanceSquared(p));
  }

  /**
   * Returns the point along the segment between this point and the provided point using the provided
   * distance along. A distance of .5 would be the point halfway between the two points. A distance of
   * .25 would be 25% of the way from this point to the provided point. The default distance is .5, so
   * the midpoint
   * @param   p1         second point of the line segment
   * @param   distance   the percentage along the segment to place the output point
   * @return  returns the point along the line segment at the specified distance
   */
  PointXY<PrecisionT> PointAlongSegment(const PointXY<PrecisionT>& p1,
                                        PrecisionT distance = .5) const {
    return PointXY(first + distance * (p1.first - first), second + distance * (p1.second - second));
  }

  /**
   * Add a vector to the current point.
   * @param   v  Vector to add to the current point.
   * @return  Returns a new point: the result of the current point
   *          plus the specified vector.
   */
  PointXY<PrecisionT> operator+(const VectorXY<PrecisionT>& v) const {
    return PointXY<PrecisionT>(first + v.x(), second + v.y());
  }

  /**
   * Subtract a vector from the current point.
   * @param   v  Vector to subtract from the current point.
   * @return  Returns a new point: the result of the current point
   *          minus the specified vector.
   */
  PointXY<PrecisionT> operator-(const VectorXY<PrecisionT>& v) const {
    return PointXY<PrecisionT>(first - v.x(), second - v.y());
  }

  /**
   * Subtraction of a point from the current point.
   * @param p Point to subtract from the current point.
   * @return  Returns a vector.
   */
  VectorXY<PrecisionT> operator-(const PointXY<PrecisionT>& p) const {
    return VectorXY<PrecisionT>(first - p.first, second - p.second);
  }

  /**
   * Finds the closest point to the supplied polyline as well as the distance
   * to that point and the index of the segment where the closest point lies.
   * @param  pts     List of points on the polyline.
   * @return  tuple of <Closest point along the polyline,
   *                    Returns the distance of the closest point,
   *                    Index of the segment of the polyline which contains the closest point
   *                   >
   */
  std::tuple<PointXY<PrecisionT>, PrecisionT, int>
  ClosestPoint(const std::vector<PointXY<PrecisionT>>& pts) const;

  /**
   * Test whether this point is to the left of a segment from p1 to p2.
   * @param  p1  First point of the segment.
   * @param  p2  End point of the segment.
   * @return  Returns true if this point is left of the segment.
   */
  virtual PrecisionT IsLeft(const PointXY<PrecisionT>& p1, const PointXY<PrecisionT>& p2) const {
    return (p2.x() - p1.x()) * (y() - p1.y()) - (x() - p1.x()) * (p2.y() - p1.y());
  }

  /**
   * Tests whether this point is within a polygon.
   * @param  poly  List of vertices that form a polygon. Assumes
   *               the following:
   *                  Only the first and last vertices may be duplicated.
   * @return  Returns true if the point is within the polygon, false if not.
   */
  template <class container_t> bool WithinPolygon(const container_t& poly) const;

  /**
   * Handy for templated functions that use both Point2 or PointLL to know whether or not
   * the classes coordinate system is spherical or in the plane
   *
   * @return true if the system is spherical false if not
   */
  static bool IsSpherical() {
    return false;
  }

  /**
   * Return a string in format "lon,lat"
   *
   * @return String in format "lon,lat"
   */
  inline const std::string to_string() const {
    return std::to_string(this->first) + "," + std::to_string(this->second);
  }
};

using Point2 = PointXY<float>;
using Point2d = PointXY<double>;

} // namespace midgard
} // namespace valhalla

namespace std {
template <typename PrecisionT> struct hash<valhalla::midgard::PointXY<PrecisionT>> {
  size_t operator()(const valhalla::midgard::PointXY<PrecisionT>& p) const;
};
} // namespace std
