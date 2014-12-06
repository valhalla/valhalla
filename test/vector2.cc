#include "test.h"

#include "include/config.h"
#include "geo/point2.h"
#include "geo/vector2.h"

using namespace std;
using namespace valhalla::geo;

namespace {

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
