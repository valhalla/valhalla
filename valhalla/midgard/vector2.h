#ifndef VALHALLA_MIDGARD_VECTOR2_H_
#define VALHALLA_MIDGARD_VECTOR2_H_

#include <cmath>
#include <cstdarg>

#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/point2.h>

namespace valhalla {
namespace midgard {

/**
 * 2D vector class. float x,y components.
 * @author  David W. Nesbitt
 */
template <typename PrecisionT> class VectorXY {
public:
  /**
   * Default constructor
   */
  VectorXY<PrecisionT>() : x_(0.0), y_(0.0) {
  }

  /**
   * Constructor given a point.  Essentially a vector from the
   * origin to the point.
   * @param   p  Point.
   */
  VectorXY<PrecisionT>(const PointXY<PrecisionT>& p) : x_(p.x()), y_(p.y()) {
  }

  /**
   * Constructor given components of the vector.
   * @param   x   x component of the vector.
   * @param   y   y component of the vector.
   */
  VectorXY<PrecisionT>(const PrecisionT x, const PrecisionT y) : x_(x), y_(y) {
  }

  /**
   * Constructor from one point to another.
   * @param   from  Point at origin of the vector.
   * @param   to    Point at end of vector
   */
  VectorXY<PrecisionT>(const PointXY<PrecisionT>& from, const PointXY<PrecisionT>& to)
      : x_(to.x() - from.x()), y_(to.y() - from.y()) {
  }

  /**
   * Copy constructor.
   * @param   w  Vector to copy to the new vector.
   */
  VectorXY<PrecisionT>(const VectorXY<PrecisionT>& w) : x_(w.x()), y_(w.y()) {
  }

  /**
   * Assignment operator
   * @param   w  Vector to copy to the current vector.
   * @return  Returns the address of the current vector.
   */
  VectorXY<PrecisionT>& operator=(const VectorXY<PrecisionT>& w) {
    x_ = w.x();
    y_ = w.y();
    return *this;
  }

  /**
   * Get the x component.
   * @return  Returns the x component of the vector.
   */
  PrecisionT x() const {
    return x_;
  }

  /**
   * Get the y component.
   * @return  Returns the y component of the vector.
   */
  PrecisionT y() const {
    return y_;
  }

  /**
   * Set the x component.
   * @param  x  x coordinate value.
   */
  void set_x(const PrecisionT x) {
    x_ = x;
  }

  /**
   * Set the y component.
   * @param  y  y coordinate value.
   */
  void set_y(const PrecisionT y) {
    y_ = y;
  }

  /**
   * Set the current vector to the specified components.
   * @param   x   x component of the vector.
   * @param   y   y component of the vector.
   */
  void Set(const PrecisionT x, const PrecisionT y) {
    x_ = x;
    y_ = y;
  }

  /**
   * Set the vector components to those of a point.  Essentially a
   * vector from the origin to the point.
   * @param   p  Point.
   */
  void Set(const PointXY<PrecisionT>& p) {
    x_ = p.x();
    y_ = p.y();
  }

  /**
   * Set the current vector to be from one point to another.
   * @param   from  Point at origin of the vector.
   * @param   to    Point at end of vector
   */
  void Set(const PointXY<PrecisionT>& from, const PointXY<PrecisionT>& to) {
    x_ = to.x() - from.x();
    y_ = to.y() - from.y();
  }

  /**
   * Creates a new vector that is the current vector plus the
   * specified vector.
   * @param   w  Vector to add to the current vector.
   * @return   Returns the resulting vector.
   */
  VectorXY<PrecisionT> operator+(const VectorXY<PrecisionT>& w) const {
    return VectorXY<PrecisionT>(x_ + w.x(), y_ + w.y());
  }

  /**
   * Adds vector w to the current vector.
   * @param   w  Vector to add to the current vector.
   * @return  Returns the address of the current vector.
   */
  VectorXY<PrecisionT>& operator+=(const VectorXY<PrecisionT>& w) {
    x_ += w.x();
    y_ += w.y();
    return *this;
  }

  /**
   * Creates a new vector that is the current vector minus the
   * specified vector.
   * @param   w  Vector to subtract from the current vector.
   * @return   Returns the resulting vector.
   */
  VectorXY<PrecisionT> operator-(const VectorXY<PrecisionT>& w) const {
    return VectorXY<PrecisionT>(x_ - w.x(), y_ - w.y());
  }

  /**
   * Subtracts vector w from the current vector.
   * @param   w  Vector to subtract from the current vector.
   * @return  Returns the address of the current vector.
   */
  VectorXY<PrecisionT>& operator-=(const VectorXY<PrecisionT>& w) {
    x_ -= w.x();
    y_ -= w.y();
    return *this;
  }

  /**
   * Creates a new vector that is the current vector multiplied
   * with the specified scalar.
   * @param   scalar   Scalar to muliply the vector with.
   * @return  Returns the resulting vector
   */
  VectorXY<PrecisionT> operator*(const PrecisionT scalar) const {
    return VectorXY<PrecisionT>(x_ * scalar, y_ * scalar);
  }

  /**
   * Multiplies the current vector by a scalar
   * @param   scalar   Scalar to muliply the vector with.
   * @return  Returns the address of the current vector.
   */
  VectorXY<PrecisionT>& operator*=(const PrecisionT scalar) {
    x_ *= scalar;
    y_ *= scalar;
    return *this;
  }

  /**
   * Equality operator.
   * @param   w  Vector to test if equal to the current vector.
   * @return  Returns true if vector w equals the current vector,
   *          false otherwise.
   */
  bool operator==(const VectorXY<PrecisionT>& w) const {
    return (x_ == w.x() && y_ == w.y());
  }

  /**
   * Computes the dot product of the current vector with the
   * specified vector.
   * @param   w  Vector
   * @return  Returns the dot product (a scalar).
   */
  PrecisionT Dot(const VectorXY<PrecisionT>& w) const {
    return (x_ * w.x() + y_ * w.y());
  }

  /**
   * Computes the 2D cross product of current vector with w0.
   * @param   w  Vector to take the cross product with (current X w)
   * @return  Returns the magnitude of the resulting vector (which is
   *          along the z axis)
   */
  PrecisionT Cross(const VectorXY<PrecisionT>& w) const {
    return (x_ * w.y() - y_ * w.x());
  }

  /**
   * Get a perpendicular vector to this vector.
   * @param  clockwise  If true. get the clockwise oriented perpendicular.
   *                    If false, get the counter-clockwise oriented
   *                    perpendicular.
   */
  VectorXY<PrecisionT> GetPerpendicular(const bool clockwise = false) const {
    return (clockwise) ? VectorXY<PrecisionT>(y_, -x_) : VectorXY<PrecisionT>(-y_, x_);
  }

  /**
   * Computes the norm (length) of the current vector.
   * @return  Returns the length of the vector.
   */
  PrecisionT Norm() const {
    return sqrtf(Dot(*this));
  }

  /**
   * Computes the squared norm of a vector
   * (Useful when absolute distance is not required)
   * @return  Returns the length squared of the vector.
   */
  PrecisionT NormSquared(void) const {
    return (Dot(*this));
  }

  /**
   * Normalizes the vector.
   * @return  Returns the address of the current vector.
   */
  VectorXY<PrecisionT>& Normalize() {
    // Normalize the vector if the norm is not 0 or 1
    PrecisionT n = Norm();
    if (n > kEpsilon && n != 1.0) {
      x_ /= n;
      y_ /= n;
    }
    return *this;
  }

  /**
   * Calculates the component of the current vector along the
   * specified vector
   * @param   w  Vector to determine component along.
   * @return  Returns the component of the current vector along w.
   */
  PrecisionT Component(const VectorXY<PrecisionT>& w) const {
    PrecisionT n = w.Dot(w);
    return (n != 0.0) ? (Dot(w) / n) : 0.0;
  }

  /**
   * Creates a new vector that is the projection of the current
   * vector along the specified vector.
   * @param   w  Vector to determine projection along.
   * @return  Returns the new vector.
   */
  VectorXY<PrecisionT> Projection(const VectorXY<PrecisionT>& w) const {
    return w * Component(w);
  }

  /**
   * Calculates the angle (radians) between the current vector and
   * the specified vector.
   * @param   w  Vector to determine angle from current vector.
   * @return  Returns the angle in radians between the two vectors.
   */
  PrecisionT AngleBetween(const VectorXY<PrecisionT>& w) const {
    return acosf(Dot(w) / (Norm() * w.Norm()));
  }

  /**
   * Reflects the current vector given a normal to the reflecting surface.
   * Assumes the normal is unit length.  Note that if done properly the
   * magnitude of the reflected vector will equal the magnitude of the
   * incoming vector.
   * @param   normal  unit length normal to the vector where reflection occurs
   * @return  Returns the reflected vector
   */
  VectorXY<PrecisionT> Reflect(const VectorXY<PrecisionT>& normal) const {
    VectorXY<PrecisionT> d = *this;
    return (d - (normal * (2.0 * (d.Dot(normal)))));
  }

private:
  // x,y coordinate of the point
  PrecisionT x_;
  PrecisionT y_;
};

using Vector2 = VectorXY<float>;
using Vector2d = VectorXY<double>;

/**
 * Creates a new vector that is the specified vector multiplied
 * with the specified scalar.
 * @param   s  Scalar to muliply the vector with.
 * @param   v  Vector to be multiplied with the scalar
 * @return  Returns the resulting vector
 */
Vector2 operator*(float s, const Vector2& v);

Vector2d operator*(double s, const Vector2d& v);

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_VECTOR2_H_
