#ifndef VALHALLA_MIDGARD_POINT2_H_
#define VALHALLA_MIDGARD_POINT2_H_

#include <math.h>
#include <vector>
#include <utility>

namespace valhalla{
namespace midgard{

// Forward references
class Vector2;

/**
 * 2D Point (cartesian). float x,y components.
 * @author David W. Nesbitt
 */
class Point2 : public std::pair<float, float>{

 public:
  /**
   * Default constructor
   */
  Point2();

  /**
   * Constructor with initial values for x,y.
   * @param   x   x coordinate position.
   * @param   y   y coordinate position.
   */
  Point2(const float x, const float y);

  /**
   * Copy constructor.
   * @param   p   Point to copy to the new point.
   */
  Point2(const Point2& p);

  /**
   * Assignment operator
   * @param   p   Point to assign to this point.
   * @return  Returns the address of this point.
   */
  Point2& operator = (const Point2& p);

  /**
   * Destructor
   */
  virtual ~Point2();

  /**
   * Get the x component.
   * @return  Returns the x component of the point.
   */
  float x() const;

  /**
   * Get the y component.
   * @return  Returns the y component of the point.
   */
  float y() const;

  /**
   * Set the x component.
   * @param  x  x coordinate value.
   */
  void set_x(const float x);

  /**
   * Set the y component.
   * @param  y  y coordinate value.
   */
  void set_y(const float y);

  /**
   * Set the coordinate components to the specified values.
   * @param   x   x coordinate position.
   * @param   y   y coordinate position.
   */
  virtual void Set(const float x, const float y);

  /**
   * Equality operator.
   * @param   p  Point to compare to the current point.
   * @return  Returns true if two points are equal, false otherwise.
   */
  bool operator == (const Point2& p) const;

  /**
   * Inequality operator.
   * @param   p  Point to compare to the current point.
   * @return  Returns true if the supplied point is not equal to the
   *          point, false otherwise.
   */
  bool operator!= (const Point2& p) const;

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
   * Affine combination of this point with another point. 2 scalars are
   * provided (a0 and a1) and the must add to 1.
   * @param  a0  Scalar for this point
   * @param  a1  Scalar for p1
   * @param  p1  Point 1
   */
  Point2 AffineCombination(const float a0, const float a1,
                           const Point2& p1) const;
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
  Point2 operator + (const Vector2& v) const;

  /**
   * Subtract a vector from the current point.
   * @param   v  Vector to subtract from the current point.
   * @return  Returns a new point: the result of the current point
   *          minus the specified vector.
   */
  Point2 operator - (const Vector2& v) const;

  /**
   * Subtraction of a point from the current point.
   * @param   Point to subtract from the current point.
   * @return  Returns a vector.
   */
  Vector2 operator - (const Point2& p) const;

  /**
   * Finds the closest point to the supplied polyline as well as the distance
   * squared to that point.
   * @param  pts     List of points on the polyline.
   * @param  closest (OUT) Closest point along the polyline
   * @param  idx     (OUT) Index of the segment of the polyline which contains
   *                       the closest point.
   * @return   Returns the distance squared of the closest point.
   */
  float ClosestPoint(const std::vector<Point2>& pts, Point2& closest,
            int& idx) const;

 protected:
  float x_;
  float y_;

 private:

};

}
}

#endif  // VALHALLA_MIDGARD_POINT2_H_
