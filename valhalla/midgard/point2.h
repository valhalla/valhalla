#ifndef VALHALLA_MIDGARD_POINT2_H_
#define VALHALLA_MIDGARD_POINT2_H_

#include <cstring>
#include <functional>
#include <tuple>
#include <utility>
#include <vector>

namespace {
constexpr float LL_EPSILON = .00002f;
}

namespace valhalla {
namespace midgard {

// Forward references
class Vector2;

/**
 * 2D Point (cartesian). float x,y components.
 * @author David W. Nesbitt
 */
class Point2 : public std::pair<float, float> {

public:
  /**
   * Use the constructors provided by pair
   */
  using std::pair<float, float>::pair;

  /**
   * Destructor
   */
  virtual ~Point2() {
  }

  /**
   * Get the x component.
   * @return  Returns the x component of the point.
   */
  float x() const {
    return first;
  }

  /**
   * Get the y component.
   * @return  Returns the y component of the point.
   */
  float y() const {
    return second;
  }

  /**
   * Set the x component.
   * @param  x  x coordinate value.
   */
  void set_x(const float x) {
    first = x;
  }

  /**
   * Set the y component.
   * @param  y  y coordinate value.
   */
  void set_y(const float y) {
    second = y;
  }

  /**
   * Set the coordinate components to the specified values.
   * @param   x   x coordinate position.
   * @param   y   y coordinate position.
   */
  virtual void Set(const float x, const float y) {
    first = x;
    second = y;
  }

  /**
   * Equality approximation.
   * @param   p  Point to compare to the current point.
   * @param   e  An epsilon which determines how close they must be to be considered equal
   * @return  Returns true if two points are approximately equal, false otherwise.
   */
  bool ApproximatelyEqual(const Point2& p, float e = LL_EPSILON) const;

  /**
   * Get the distance squared from this point to point p.
   * @param   p  Other point.
   * @return  Returns the distance squared between this point and p.
   */
  virtual float DistanceSquared(const Point2& p) const;

  /**
   * Get the distance from this point to point p.
   * @param   p  Other point.
   * @return  Returns the distance between this point and p.
   */
  virtual float Distance(const Point2& p) const;

  /**
   * Returns the point a specified percentage along a segment from this point
   * to an end point.
   * @param  end  End point.
   * @param  pct  Percentage along the segment.
   * @return Returns the point along the segment.
   */
  Point2 along_segment(const Point2& end, const float pct) const {
    return {x() + (end.x() - x()) * pct, y() + (end.y() - y()) * pct};
  }

  /**
   * Affine combination of this point with another point. 2 scalars are
   * provided (a0 and a1) and the must add to 1.
   * @param  a0  Scalar for this point
   * @param  a1  Scalar for p1
   * @param  p1  Point 1
   */
  Point2 AffineCombination(const float a0, const float a1, const Point2& p1) const;
  /**
   * Gets the midpoint on a line segment between this point and point p1.
   * @param   p1  Point
   * @return  Returns the midpoint between this point and p1.
   */
  Point2 MidPoint(const Point2& p1) const;

  /**
   * Add a vector to the current point.
   * @param   v  Vector to add to the current point.
   * @return  Returns a new point: the result of the current point
   *          plus the specified vector.
   */
  Point2 operator+(const Vector2& v) const;

  /**
   * Subtract a vector from the current point.
   * @param   v  Vector to subtract from the current point.
   * @return  Returns a new point: the result of the current point
   *          minus the specified vector.
   */
  Point2 operator-(const Vector2& v) const;

  /**
   * Subtraction of a point from the current point.
   * @param p Point to subtract from the current point.
   * @return  Returns a vector.
   */
  Vector2 operator-(const Point2& p) const;

  /**
   * Finds the closest point to the supplied polyline as well as the distance
   * to that point and the index of the segment where the closest point lies.
   * @param  pts     List of points on the polyline.
   * @return  tuple of <Closest point along the polyline,
   *                    Returns the distance of the closest point,
   *                    Index of the segment of the polyline which contains the closest point
   *                   >
   */
  std::tuple<Point2, float, int> ClosestPoint(const std::vector<Point2>& pts) const;

  /**
   * Test whether this point is to the left of a segment from p1 to p2.
   * @param  p1  First point of the segment.
   * @param  p2  End point of the segment.
   * @return  Returns true if this point is left of the segment.
   */
  virtual float IsLeft(const Point2& p1, const Point2& p2) const {
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
  static bool IsSpherical();

protected:
};

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

#endif // VALHALLA_MIDGARD_POINT2_H_
