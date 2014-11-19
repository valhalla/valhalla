#ifndef __point2_h__
#define __point2_h__

#include <math.h>

// Forward references
class Vector2;

/**
 * 2D Point (cartesian). float x,y components.
 * @author David W. Nesbitt
 */
class Point2 {
 protected:
  float x_;
  float y_;

 public:
  /**
   * Default constructor
   */
  Point2() : x_(0.0f), y_(0.0f) { }

  /**
   * Constructor with initial values for x,y.
   * @param   x   x coordinate position.
   * @param   y   y coordinate position.
   */
  Point2(const float x, const float y) : x_(x), y_(y) { }

  /**
   * Copy constructor.
   * @param   p   Point to copy to the new point.
   */
	Point2(const Point2& p) : x_(p.x_), y_(p.y_) { }

  /**
   * Assignment operator
   * @param   p   Point to assign to this point.
   * @return  Returns the address of this point.
   */
  Point2& operator = (const Point2& p) {
    x_ = p.x();
    y_ = p.y();
    return *this;
  }

  /**
   * Destructor
   */
  ~Point2() { }

  /**
   * Get the x component.
   * @return  Returns the x component of the point.
   */
  float x() const {
    return x_;
  }

  /**
   * Get the y component.
   * @return  Returns the y component of the point.
   */
  float y() const {
    return y_;
  }

  /**
   * Set the x component.
   * @param  x  x coordinate value.
   */
  void set_x(const float x) {
    x_ = x;
  }

  /**
   * Set the y component.
   * @param  y  y coordinate value.
   */
  void set_y(const float y) {
    y_ = y;
  }

  /**
   * Set the coordinate components to the specified values.
   * @param   x   x coordinate position.
   * @param   y   y coordinate position.
   */
  void Set(const float x, const float y) {
    x_ = x;
    y_ = y;
  }

  /**
   * Equality operator.
   * @param   p  Point to compare to the current point.
   * @return  Returns true if two points are equal, false otherwise.
   */
  bool operator == (const Point2& p) const {
    return (x_ == p.x() && y_ == p.y());
  }

  /**
   * Inequality operator.
   * @param   p  Point to compare to the current point.
   * @return  Returns true if the supplied point is not equal to the
   *          point, false otherwise.
   */
  bool operator!= (const Point2& p) const {
    return (x_ != p.x() || y_ != p.y());
  }

  /**
   * Get the distance from this point to point p.
   * @param   p  Other point.
   * @return  Returns the distance between this point and p.
   */
  virtual float Distance(const Point2& p) const {
    return sqrtf(sqr(x_ - p.x()) + sqr(y_ - p.y()) );
  }

  /**
   * Affine combination of this point with another point. 2 scalars are
   * provided (a0 and a1) and the must add to 1.
   * @param  a0  Scalar for this point
   * @param  a1  Scalar for p1
   * @param  p1  Point 1
   */
  Point2 affineCombination(const float a0, const float a1,
                           const Point2& p1) const {
    return Point2(a0*x_ + a1*p1.x(), a0*y_ + a1*p1.y());
  }

  /**
   * Gets the midpoint on a line segment between this point and point p1.
   * @param   p1  Point
   * @return  Returns the midpoint between this point and p1.
   */
  Point2 midPoint(const Point2& p1) {
    return Point2(0.5f*x_ + 0.5f*p1.x(), 0.5f*y_ + 0.5f*p1.y());
  }

  // The following methods are defined in geosupport.h

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

 private:

  // Convenience method to square a value
  float sqr(const float v) const {
    return v*v;
  }
};

#endif
