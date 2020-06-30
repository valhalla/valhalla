#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
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
  bool ApproximatelyEqual(const PointXY<PrecisionT>& p, float e = LL_EPSILON) const {
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
   * Returns the point a specified percentage along a segment from this point
   * to an end point.
   * @param  end  End point.
   * @param  pct  Percentage along the segment.
   * @return Returns the point along the segment.
   */
  PointXY<PrecisionT> along_segment(const PointXY<PrecisionT>& end, const PrecisionT pct) const {
    return {x() + (end.x() - x()) * pct, y() + (end.y() - y()) * pct};
  }

  /**
   * Affine combination of this point with another point. 2 scalars are
   * provided (a0 and a1) and the must add to 1.
   * @param  a0  Scalar for this point
   * @param  a1  Scalar for p1
   * @param  p1  Point 1
   */
  PointXY<PrecisionT>
  AffineCombination(const PrecisionT a0, const PrecisionT a1, const PointXY<PrecisionT>& p1) const {
    return PointXY<PrecisionT>(a0 * first + a1 * p1.first, a0 * second + a1 * p1.second);
  }
  /**
   * Gets the midpoint on a line segment between this point and point p1.
   * @param   p1  Point
   * @return  Returns the midpoint between this point and p1.
   */
  PointXY<PrecisionT> MidPoint(const PointXY<PrecisionT>& p1) const {
    return PointXY(0.5f * (first + p1.first), 0.5f * (second + p1.second));
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
  virtual float IsLeft(const PointXY<PrecisionT>& p1, const PointXY<PrecisionT>& p2) const {
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

protected:
};

using Point2 = PointXY<float>;

} // namespace midgard
} // namespace valhalla

namespace std {
template <> struct hash<valhalla::midgard::Point2> {
  size_t operator()(const valhalla::midgard::Point2& p) const {
    uint64_t h;
    char* b = static_cast<char*>(static_cast<void*>(&h));
    std::memcpy(b, &p.first, 4);
    std::memcpy(b + 4, &p.second, 4);
    return std::hash<uint64_t>()(h);
  }
};
} // namespace std
