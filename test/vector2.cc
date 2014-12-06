#include "test.h"

#include "include/config.h"
#include "geo/point2.h"
#include "geo/vector2.h"

using namespace std;
using namespace valhalla::geo;

namespace {

void TestCtorDefault() {
  Vector2 target;
  Vector2 expected { 0.0f, 0.0f };
  if (!(expected == target))
    throw runtime_error("CtorDefault test failed");
}

void TryCtorPoint2(const Point2& pt, const Vector2& expected) {
  Vector2 result(pt);
  if (!(expected == result))
    throw runtime_error("CtorPoint2 test failed");
}

void TestCtorPoint2() {
  TryCtorPoint2(Point2(3.0f, 0.0f), Vector2(3.0f, 0.0f));
  TryCtorPoint2(Point2(-8.0f, 6.0f), Vector2(-8.0f, 6.0f));
}

void TryCtorFloatFloat(const float x, const float y, const Vector2& expected) {
  Vector2 result(x, y);
  if (!(expected == result))
    throw runtime_error("CtorFloatFloat test failed");
}

void TestCtorFloatFloat() {
  TryCtorFloatFloat(3.0f, 0.0f, Vector2(3.0f, 0.0f));
  TryCtorFloatFloat(-8.0f, 6.0f, Vector2(-8.0f, 6.0f));
}

void TryCtorPoint2Point2(const Point2& from, const Point2& to,
                         const Vector2& expected) {
  Vector2 result(from, to);
  if (!(expected == result))
    throw runtime_error("CtorPoint2Point2 test failed");
}

void TestCtorPoint2Point2() {
  TryCtorPoint2Point2(Point2(4.0f, 0.0f), Point2(3.0f, 3.0f),
                      Vector2(-1.0f, 3.0f));
  TryCtorPoint2Point2(Point2(4.0f, 2.0f), Point2(4.0f, -2.0f),
                      Vector2(0.0f, -4.0f));
}

void TryCtorVector2(const Vector2& v, const Vector2& expected) {
  Vector2 result(v);
  if (!(expected == result))
    throw runtime_error("CtorVector2 test failed");
}

void TestCtorVector2() {
  TryCtorVector2(Vector2(3.0f, 0.0f), Vector2(3.0f, 0.0f));
  TryCtorVector2(Vector2(-8.0f, 6.0f), Vector2(-8.0f, 6.0f));
}

void TryOpAssignment(const Vector2& v, const Vector2& expected) {
  Vector2 result;
  result = v;
  if (!(expected == result))
    throw runtime_error("OpAssignment test failed");
}

void TestOpAssignment() {
  TryOpAssignment(Vector2(3.0f, 0.0f), Vector2(3.0f, 0.0f));
  TryOpAssignment(Vector2(-8.0f, 6.0f), Vector2(-8.0f, 6.0f));
}

void TryDotProduct(const Vector2& a, const Vector2& b, float expected) {
  float result = a.Dot(b);
  if (expected != result)
    throw runtime_error("DotProduct test failed");
}

void TestDotProduct() {
  TryDotProduct(Vector2(3.0f, 0.0f), Vector2(5.0f, 5.0f), 15.0f);
  TryDotProduct(Vector2(3.0f, 4.0f), Vector2(-8.0f, 6.0f), 0.0f);
}

void TryCrossProduct(const Vector2& a, const Vector2& b, float expected) {
  float result = a.Cross(b);
  if (expected != result)
    throw runtime_error("CrossProduct test failed");
}

void TestCrossProduct() {
  TryCrossProduct(Vector2(3.0f, 0.0f), Vector2(5.0f, 5.0f), 15.0f);
  TryCrossProduct(Vector2(3.0f, 4.0f), Vector2(-8.0f, 6.0f), 50.0f);
}

void TryPerpendicular(const Vector2& a, const Vector2& expected) {
  Vector2 result = a.GetPerpendicular();
  if (!(expected == result))
    throw runtime_error("Perpendicular test failed");
}

void TestPerpendicular() {
  TryPerpendicular(Vector2(3.0f, 4.0f), Vector2(-4.0f, 3.0f));
}

void TryNorm(const Vector2& a, float expected) {
  float result = a.Norm();
  if (expected != result)
    throw runtime_error("Norm test failed");
}

void TestNorm() {
  TryNorm(Vector2(3.0f, 4.0f), 5.0f);
  TryNorm(Vector2(6.0f, 8.0f), 10.0f);
}

void TryNormSquared(const Vector2& a, float expected) {
  float result = a.NormSquared();
  if (expected != result)
    throw runtime_error("NormSquared test failed");
}

void TestNormSquared() {
  TryNormSquared(Vector2(3.0f, 4.0f), 25.0f);
  TryNormSquared(Vector2(6.0f, 8.0f), 100.0f);
}

void TryNormalize(Vector2& a, const Vector2& expected) {
  a.Normalize();
  if (!(expected == a))
    throw runtime_error("Normalize test failed");
}

void TestNormalize() {
  Vector2 v(3.0f, 4.0f);
  TryNormalize(v, Vector2(3.0f / 5.0f, 4.0f / 5.0f));
  Vector2 w(6.0f, 8.0f);
  TryNormalize(w, Vector2(6.0f / 10.0f, 8.0f / 10.0f));
}

void TryComponent(const Vector2& a, const Vector2& b, float expected) {
  float result = a.Component(b);
  if (expected != result)
    throw runtime_error("Component test failed");
}

void TestComponent() {
  TryComponent(Vector2(3.0f, 4.0f), Vector2(6.0f, 8.0f), 0.5f);
  TryComponent(Vector2(6.0f, 8.0f), Vector2(3.0f, 4.0f), 2.0f);
}

void TryProjection(const Vector2& a, const Vector2& b,
                   const Vector2& expected) {
  Vector2 result = a.Projection(b);
  if (!(expected == result))
    throw runtime_error("Projection test failed");
}

void TestProjection() {
  TryProjection(Vector2(3.0f, 4.0f), Vector2(6.0f, 8.0f), Vector2(3.0f, 4.0f));
  TryProjection(Vector2(6.0f, 8.0f), Vector2(3.0f, 4.0f), Vector2(6.0f, 8.0f));
  TryProjection(Vector2(2.0f, 1.0f), Vector2(-3.0f, 4.0f),
                Vector2(6.0f / 25.0f, -8.0f / 25.0f));
}

void TryAngleBetween(const Vector2& a, const Vector2& b, float expected) {
  float result = (a.AngleBetween(b) * kDegPerRad);
  if (expected != result)
    throw runtime_error("AngleBetween test failed");
}

void TestAngleBetween() {
  TryAngleBetween(Vector2(3.0f, 0.0f), Vector2(5.0f, 5.0f), 45.0f);
  TryAngleBetween(Vector2(3.0f, 4.0f), Vector2(-8.0f, 6.0f), 90.0f);
}

void TryReflect(const Vector2& a, const Vector2& b, const Vector2& expected) {
  Vector2 result = a.Reflect(b);
  if (!(expected == result))
    throw runtime_error("Reflect test failed");
}

void TestReflect() {
  Vector2 n1 { 0.0f, 2.0f };
  n1.Normalize();
  TryReflect(Vector2(4.0f, -2.0f), n1, Vector2(4.0f, 2.0f));
  Vector2 n2 { -3.0f, 0.0f };
  n2.Normalize();
  TryReflect(Vector2(3.0f, -4.0f), n2, Vector2(-3.0f, -4.0f));
}

}

int main() {
  test::suite suite("vector2");

  // Ctor default
  suite.test(TEST_CASE(TestCtorDefault));

  // Ctor Point2
  suite.test(TEST_CASE(TestCtorPoint2));

  // Ctor float, float
  suite.test(TEST_CASE(TestCtorFloatFloat));

  // Ctor Point2, Point2
  suite.test(TEST_CASE(TestCtorPoint2Point2));

  // Ctor Vector2
  suite.test(TEST_CASE(TestCtorVector2));

  // Op Assignment
  suite.test(TEST_CASE(TestOpAssignment));

  // Dot Product
  suite.test(TEST_CASE(TestDotProduct));

  // Cross Product
  suite.test(TEST_CASE(TestCrossProduct));

  // Perpendicular
  suite.test(TEST_CASE(TestPerpendicular));

  // Norm
  suite.test(TEST_CASE(TestNorm));

  // NormSquared
  suite.test(TEST_CASE(TestNormSquared));

  // Normalize
  suite.test(TEST_CASE(TestNormalize));

  // Component
  suite.test(TEST_CASE(TestComponent));

  // Projection
  suite.test(TEST_CASE(TestProjection));

  // Angle Between
  suite.test(TEST_CASE(TestAngleBetween));

  // Reflect
  suite.test(TEST_CASE(TestReflect));

  return suite.tear_down();
}
