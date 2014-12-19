#include "valhalla/midgard/vector2.h"

namespace valhalla {
namespace midgard {

Vector2::Vector2()
    : x_(0.0f),
      y_(0.0f) {
}

Vector2::Vector2(const Point2& p)
    : x_(p.x()),
      y_(p.y()) {
}

Vector2::Vector2(const float x, const float y)
    : x_(x),
      y_(y) {
}

Vector2::Vector2(const Point2& from, const Point2& to)
    : x_(to.x() - from.x()),
      y_(to.y() - from.y()) {
}

Vector2::Vector2(const Vector2& w)
    : x_(w.x()),
      y_(w.y()) {
}

Vector2& Vector2::operator =(const Vector2& w) {
  x_ = w.x();
  y_ = w.y();
  return *this;
}

Vector2::~Vector2() {
}

float Vector2::x() const {
  return x_;
}

float Vector2::y() const {
  return y_;
}

void Vector2::set_x(const float x) {
  x_ = x;
}

void Vector2::set_y(const float y) {
  y_ = y;
}

void Vector2::Set(const float x, const float y) {
  x_ = x;
  y_ = y;
}

void Vector2::Set(const Point2& p) {
  x_ = p.x();
  y_ = p.y();
}

void Vector2::Set(const Point2& from, const Point2& to) {
  x_ = to.x() - from.x();
  y_ = to.y() - from.y();
}

Vector2 Vector2::operator +(const Vector2& w) const {
  return Vector2(x_ + w.x(), y_ + w.y());
}

Vector2& Vector2::operator +=(const Vector2& w) {
  x_ += w.x();
  y_ += w.y();
  return *this;
}

Vector2 Vector2::operator -(const Vector2& w) const {
  return Vector2(x_ - w.x(), y_ - w.y());
}

Vector2& Vector2::operator -=(const Vector2& w) {
  x_ -= w.x();
  y_ -= w.y();
  return *this;
}

Vector2 Vector2::operator *(const float scalar) const {
  return Vector2(x_ * scalar, y_ * scalar);
}

Vector2& Vector2::operator *=(const float scalar) {
  x_ *= scalar;
  y_ *= scalar;
  return *this;
}

bool Vector2::operator ==(const Vector2& w) const {
  return (x_ == w.x() && y_ == w.y());
}

float Vector2::Dot(const Vector2& w) const {
  return (x_ * w.x() + y_ * w.y());
}

float Vector2::Cross(const Vector2& w) const {
  return (x_ * w.y() - y_ * w.x());
}

Vector2 Vector2::GetPerpendicular(const bool clockwise) const {
  return (clockwise) ? Vector2(y_, -x_) : Vector2(-y_, x_);
}

float Vector2::Norm() const {
  return sqrtf(Dot(*this));
}

float Vector2::NormSquared(void) const {
  return (Dot(*this));
}

Vector2& Vector2::Normalize() {
  // Normalize the vector if the norm is not 0 or 1
  float n = Norm();
  if (n > kEpsilon && n != 1.0f) {
    x_ /= n;
    y_ /= n;
  }
  return *this;
}

float Vector2::Component(const Vector2& w) const {
  float n = w.Dot(w);
  return (n != 0.0f) ? (Dot(w) / n) : 0.0f;
}

Vector2 Vector2::Projection(const Vector2& w) const {
  return w * Component(w);
}

float Vector2::AngleBetween(const Vector2& w) const {
  return acosf(Dot(w) / (Norm() * w.Norm()));
}

Vector2 Vector2::Reflect(const Vector2& normal) const {
  Vector2 d = *this;
  return (d - (normal * (2.0f * (d.Dot(normal)))));
}

Vector2 operator *(float s, const Vector2 &v) {
  return Vector2(v.x() * s, v.y() * s);
}

}
}
