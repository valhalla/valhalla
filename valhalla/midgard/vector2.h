#ifndef VALHALLA_MIDGARD_VECTOR2_H_
#define VALHALLA_MIDGARD_VECTOR2_H_

#include <stdarg.h>
#include <math.h>

#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/point2.h>

namespace valhalla {
namespace midgard {

/**
 * 2D vector class. float x,y components.
 * @author  David W. Nesbitt
 */
class Vector2 {
 public:
  /**
   * Default constructor
   */
  Vector2();

  /**
   * Constructor given a point.  Essentially a vector from the
   * origin to the point.
   * @param   p  Point.
   */
  Vector2(const Point2& p);

  /**
   * Constructor given components of the vector.
   * @param   x   x component of the vector.
   * @param   y   y component of the vector.
   */
  Vector2(const float x, const float y);

  /**
   * Constructor from one point to another.
   * @param   from  Point at origin of the vector.
   * @param   to    Point at end of vector
   */
  Vector2(const Point2& from, const Point2& to);

  /**
   * Copy constructor.
   * @param   w  Vector to copy to the new vector.
   */
  Vector2(const Vector2& w);

  /**
   * Assignment operator
   * @param   w  Vector to copy to the current vector.
   * @return  Returns the address of the current vector.
   */
  Vector2& operator =(const Vector2& w);

  /**
   * Destructor
   */
  ~Vector2();

  /**
   * Get the x component.
   * @return  Returns the x component of the vector.
   */
  float x() const;

  /**
   * Get the y component.
   * @return  Returns the y component of the vector.
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
   * Set the current vector to the specified components.
   * @param   x   x component of the vector.
   * @param   y   y component of the vector.
   */
  void Set(const float x, const float y);

  /**
   * Set the vector components to those of a point.  Essentially a
   * vector from the origin to the point.
   * @param   p  Point.
   */
  void Set(const Point2& p);

  /**
   * Set the current vector to be from one point to another.
   * @param   from  Point at origin of the vector.
   * @param   to    Point at end of vector
   */
  void Set(const Point2& from, const Point2& to);

  /**
   * Creates a new vector that is the current vector plus the
   * specified vector.
   * @param   w  Vector to add to the current vector.
   * @return   Returns the resulting vector.
   */
  Vector2 operator +(const Vector2& w) const;

  /**
   * Adds vector w to the current vector.
   * @param   w  Vector to add to the current vector.
   * @return  Returns the address of the current vector.
   */
  Vector2& operator +=(const Vector2& w);

  /**
   * Creates a new vector that is the current vector minus the
   * specified vector.
   * @param   w  Vector to subtract from the current vector.
   * @return   Returns the resulting vector.
   */
  Vector2 operator -(const Vector2& w) const;

  /**
   * Subtracts vector w from the current vector.
   * @param   w  Vector to subtract from the current vector.
   * @return  Returns the address of the current vector.
   */
  Vector2& operator -=(const Vector2& w);

  /**
   * Creates a new vector that is the current vector multiplied
   * with the specified scalar.
   * @param   scalar   Scalar to muliply the vector with.
   * @return  Returns the resulting vector
   */
  Vector2 operator *(const float scalar) const;

  /**
   * Multiplies the current vector by a scalar
   * @param   scalar   Scalar to muliply the vector with.
   * @return  Returns the address of the current vector.
   */
  Vector2& operator *=(const float scalar);

  /**
   * Equality operator.
   * @param   w  Vector to test if equal to the current vector.
   * @return  Returns true if vector w equals the current vector,
   *          false otherwise.
   */
  bool operator ==(const Vector2& w) const;

  /**
   * Computes the dot product of the current vector with the
   * specified vector.
   * @param   w  Vector
   * @return  Returns the dot product (a scalar).
   */
  float Dot(const Vector2& w) const;

  /**
   * Computes the 2D cross product of current vector with w0.
   * @param   w  Vector to take the cross product with (current X w)
   * @return  Returns the magnitude of the resulting vector (which is
   *          along the z axis)
   */
  float Cross(const Vector2& w) const;

  /**
   * Get a perpendicular vector to this vector.
   * @param  clockwise  If true. get the clockwise oriented perpendicular.
   *                    If false, get the counter-clockwise oriented
   *                    perpendicular.
   */
  Vector2 GetPerpendicular(const bool clockwise = false) const;

  /**
   * Computes the norm (length) of the current vector.
   * @return  Returns the length of the vector.
   */
  float Norm() const;

  /**
   * Computes the squared norm of a vector
   * (Useful when absolute distance is not required)
   * @return  Returns the length squared of the vector.
   */
  float NormSquared(void) const;

  /**
   * Normalizes the vector.
   * @return  Returns the address of the current vector.
   */
  Vector2& Normalize();

  /**
   * Calculates the component of the current vector along the
   * specified vector
   * @param   w  Vector to determine component along.
   * @return  Returns the component of the current vector along w.
   */
  float Component(const Vector2& w) const;

  /**
   * Creates a new vector that is the projection of the current
   * vector along the specified vector.
   * @param   w  Vector to determine projection along.
   * @return  Returns the new vector.
   */
  Vector2 Projection(const Vector2& w) const;

  /**
   * Calculates the angle (radians) between the current vector and
   * the specified vector.
   * @param   w  Vector to determine angle from current vector.
   * @return  Returns the angle in radians between the two vectors.
   */
  float AngleBetween(const Vector2& w) const;

  /**
   * Reflects the current vector given a normal to the reflecting surface.
   * Assumes the normal is unit length.  Note that if done properly the
   * magnitude of the reflected vector will equal the magnitude of the
   * incoming vector.
   * @param   normal  unit length normal to the vector where reflection occurs
   * @return  Returns the reflected vector
   */
  Vector2 Reflect(const Vector2& normal) const;

 private:
  // x,y coordinate of the point
  float x_;
  float y_;
};

/**
 * Creates a new vector that is the specified vector multiplied
 * with the specified scalar.
 * @param   scalar   Scalar to muliply the vector with.
 * @param   Vector2  Vector to be multiplied with the scalar
 * @return  Returns the resulting vector
 */
Vector2 operator *(float s, const Vector2 &v);

}
}

#endif  // VALHALLA_MIDGARD_VECTOR2_H_
