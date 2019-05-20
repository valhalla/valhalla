#include "midgard/vector2.h"
#include "midgard/point2.h"
#include "midgard/util.h"
#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

void TestCtorDefault() {
  Vector2 target;
  Vector2 expected{0.0f, 0.0f};
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

void TryCtorPoint2Point2(const Point2& from, const Point2& to, const Vector2& expected) {
  Vector2 result(from, to);
  if (!(expected == result))
    throw runtime_error("CtorPoint2Point2 test failed");
}

void TestCtorPoint2Point2() {
  TryCtorPoint2Point2(Point2(4.0f, 0.0f), Point2(3.0f, 3.0f), Vector2(-1.0f, 3.0f));
  TryCtorPoint2Point2(Point2(4.0f, 2.0f), Point2(4.0f, -2.0f), Vector2(0.0f, -4.0f));
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

void TryGet_x(const Vector2& v, const float expected) {
  if (!equal(expected, v.x()))
    throw runtime_error("Get_x test failed");
}

void TestGet_x() {
  TryGet_x(Vector2(3.0f, 0.0f), 3.0f);
  TryGet_x(Vector2(-8.0f, 6.0f), -8.0f);
}

void TryGet_y(const Vector2& v, const float expected) {
  if (!equal(expected, v.y()))
    throw runtime_error("Get_y test failed");
}

void TestGet_y() {
  TryGet_y(Vector2(3.0f, 2.0f), 2.0f);
  TryGet_y(Vector2(-8.0f, 6.0f), 6.0f);
}

void TrySet_x(Vector2& v, const float expected) {
  v.set_x(expected);
  if (!equal(expected, v.x()))
    throw runtime_error("Set_x test failed");
}

void TestSet_x() {
  Vector2 v{1.0f, 1.0f};
  TrySet_x(v, 3.0f);
  TrySet_x(v, -8.0f);
}

void TrySet_y(Vector2& v, const float expected) {
  v.set_y(expected);
  if (!equal(expected, v.y()))
    throw runtime_error("Set_y test failed");
}

void TestSet_y() {
  Vector2 v{1.0f, 1.0f};
  TrySet_y(v, 3.0f);
  TrySet_y(v, -8.0f);
}

void TrySetFloatFloat(const float x, const float y, const Vector2& expected) {
  Vector2 result;
  result.Set(x, y);
  if (!(expected == result))
    throw runtime_error("SetFloatFloat test failed");
}

void TestSetFloatFloat() {
  TrySetFloatFloat(3.0f, 0.0f, Vector2(3.0f, 0.0f));
  TrySetFloatFloat(-8.0f, 6.0f, Vector2(-8.0f, 6.0f));
}

void TrySetPoint2(const Point2& pt, const Vector2& expected) {
  Vector2 result;
  result.Set(pt);
  if (!(expected == result))
    throw runtime_error("SetPoint2 test failed");
}

void TestSetPoint2() {
  TrySetPoint2(Point2(3.0f, 0.0f), Vector2(3.0f, 0.0f));
  TrySetPoint2(Point2(-8.0f, 6.0f), Vector2(-8.0f, 6.0f));
}

void TrySetPoint2Point2(const Point2& from, const Point2& to, const Vector2& expected) {
  Vector2 result;
  result.Set(from, to);
  if (!(expected == result))
    throw runtime_error("SetPoint2Point2 test failed");
}

void TestSetPoint2Point2() {
  TrySetPoint2Point2(Point2(4.0f, 0.0f), Point2(3.0f, 3.0f), Vector2(-1.0f, 3.0f));
  TrySetPoint2Point2(Point2(4.0f, 2.0f), Point2(4.0f, -2.0f), Vector2(0.0f, -4.0f));
}

void TryOpAddition(const Vector2& v, const Vector2& w, const Vector2& expected) {
  Vector2 result = v + w;
  if (!(expected == result))
    throw runtime_error("OpAddition test failed");
}

void TestOpAddition() {
  TryOpAddition(Vector2(4.0f, -2.0f), Vector2(3.0f, 3.0f), Vector2(7.0f, 1.0f));
  TryOpAddition(Vector2(4.0f, 2.0f), Vector2(-2.0f, -2.0f), Vector2(2.0f, 0.0f));
}

void TryOpAdditionAssignment(Vector2& v, const Vector2& w, const Vector2& expected) {
  v += w;
  if (!(expected == v))
    throw runtime_error("OpAdditionAssignment test failed");
}

void TestOpAdditionAssignment() {
  Vector2 v1{4.0f, -2.0f};
  TryOpAdditionAssignment(v1, Vector2(3.0f, 3.0f), Vector2(7.0f, 1.0f));
  Vector2 v2{4.0f, 2.0f};
  TryOpAdditionAssignment(v2, Vector2(-2.0f, -2.0f), Vector2(2.0f, 0.0f));
}

void TryOpSubtraction(const Vector2& v, const Vector2& w, const Vector2& expected) {
  Vector2 result = v - w;
  if (!(expected == result))
    throw runtime_error("OpSubtraction test failed");
}

void TestOpSubtraction() {
  TryOpSubtraction(Vector2(4.0f, -2.0f), Vector2(3.0f, 3.0f), Vector2(1.0f, -5.0f));
  TryOpSubtraction(Vector2(4.0f, 2.0f), Vector2(-2.0f, -2.0f), Vector2(6.0f, 4.0f));
}

void TryOpSubtractionAssignment(Vector2& v, const Vector2& w, const Vector2& expected) {
  v -= w;
  if (!(expected == v))
    throw runtime_error("OpSubtractionAssignment test failed");
}

void TestOpSubtractionAssignment() {
  Vector2 v1{4.0f, -2.0f};
  TryOpSubtractionAssignment(v1, Vector2(3.0f, 3.0f), Vector2(1.0f, -5.0f));
  Vector2 v2{4.0f, 2.0f};
  TryOpSubtractionAssignment(v2, Vector2(-2.0f, -2.0f), Vector2(6.0f, 4.0f));
}

void TryOpMultiplication(const Vector2& v, const float scalar, const Vector2& expected) {
  Vector2 result = v * scalar;
  if (!(expected == result))
    throw runtime_error("OpMultiplication test failed");

  Vector2 result2 = scalar * v;
  if (!(expected == result))
    throw runtime_error("OpMultiplication (pre) test failed");
}

void TestOpMultiplication() {
  TryOpMultiplication(Vector2(4.0f, -2.0f), 3.0f, Vector2(12.0f, -6.0f));
  TryOpMultiplication(Vector2(-4.0f, 2.0f), -2.0f, Vector2(8.0f, -4.0f));
}

void TryOpMultiplicationAssignment(Vector2& v, const float scalar, const Vector2& expected) {
  v *= scalar;
  if (!(expected == v))
    throw runtime_error("OpMultiplicationAssignment test failed");
}

void TestOpMultiplicationAssignment() {
  Vector2 v1{4.0f, -2.0f};
  TryOpMultiplicationAssignment(v1, 3.0f, Vector2(12.0f, -6.0f));
  Vector2 v2{-4.0f, 2.0f};
  TryOpMultiplicationAssignment(v2, -2.0f, Vector2(8.0f, -4.0f));
}

void TryOpEqualTo(const Vector2& v, const Vector2& expected) {
  if (!(expected == v))
    throw runtime_error("OpEqualTo test failed");
  if (!(v == expected))
    throw runtime_error("OpEqualTo test failed");
}

void TestOpEqualTo() {
  TryOpEqualTo(Vector2(1.0f, 3.0f), Vector2(1.0f, 3.0f));
  TryOpEqualTo(Vector2(4.0f, -2.0f), Vector2(4.0f, -2.0f));
  TryOpEqualTo(Vector2(-4.0f, 2.0f), Vector2(-4.0f, 2.0f));
}

void TryDotProduct(const Vector2& a, const Vector2& b, float expected) {
  float result = a.Dot(b);
  if (!equal(expected, result))
    throw runtime_error("DotProduct test failed");
}

void TestDotProduct() {
  TryDotProduct(Vector2(3.0f, 0.0f), Vector2(5.0f, 5.0f), 15.0f);
  TryDotProduct(Vector2(3.0f, 4.0f), Vector2(-8.0f, 6.0f), 0.0f);
}

void TryCrossProduct(const Vector2& a, const Vector2& b, float expected) {
  float result = a.Cross(b);
  if (!equal(expected, result))
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
  if (!equal(expected, result))
    throw runtime_error("Norm test failed");
}

void TestNorm() {
  TryNorm(Vector2(3.0f, 4.0f), 5.0f);
  TryNorm(Vector2(6.0f, 8.0f), 10.0f);
}

void TryNormSquared(const Vector2& a, float expected) {
  float result = a.NormSquared();
  if (!equal(expected, result))
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
  if (!equal(expected, result))
    throw runtime_error("Component test failed");
}

void TestComponent() {
  TryComponent(Vector2(3.0f, 4.0f), Vector2(6.0f, 8.0f), 0.5f);
  TryComponent(Vector2(6.0f, 8.0f), Vector2(3.0f, 4.0f), 2.0f);
}

void TryProjection(const Vector2& a, const Vector2& b, const Vector2& expected) {
  Vector2 result = a.Projection(b);
  if (!(expected == result))
    throw runtime_error("Projection test failed");
}

void TestProjection() {
  TryProjection(Vector2(3.0f, 4.0f), Vector2(6.0f, 8.0f), Vector2(3.0f, 4.0f));
  TryProjection(Vector2(6.0f, 8.0f), Vector2(3.0f, 4.0f), Vector2(6.0f, 8.0f));
  TryProjection(Vector2(2.0f, 1.0f), Vector2(-3.0f, 4.0f), Vector2(6.0f / 25.0f, -8.0f / 25.0f));
}

void TryAngleBetween(const Vector2& a, const Vector2& b, float expected) {
  float result = (a.AngleBetween(b) * kDegPerRad);
  if (!equal(expected, result))
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
  Vector2 n1{0.0f, 2.0f};
  n1.Normalize();
  TryReflect(Vector2(4.0f, -2.0f), n1, Vector2(4.0f, 2.0f));
  Vector2 n2{-3.0f, 0.0f};
  n2.Normalize();
  TryReflect(Vector2(3.0f, -4.0f), n2, Vector2(-3.0f, -4.0f));
}

} // namespace

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

  // Get x
  suite.test(TEST_CASE(TestGet_x));

  // Get y
  suite.test(TEST_CASE(TestGet_y));

  // Set x
  suite.test(TEST_CASE(TestSet_x));

  // Set y
  suite.test(TEST_CASE(TestSet_y));

  // Set float, float
  suite.test(TEST_CASE(TestSetFloatFloat));

  // Set Point2
  suite.test(TEST_CASE(TestSetPoint2));

  // Set Point2, Point2
  suite.test(TEST_CASE(TestSetPoint2Point2));

  // Op Addition
  suite.test(TEST_CASE(TestOpAddition));

  // Op Addition Assignment
  suite.test(TEST_CASE(TestOpAdditionAssignment));

  // Op Subtraction
  suite.test(TEST_CASE(TestOpSubtraction));

  // Op Subtraction Assignment
  suite.test(TEST_CASE(TestOpSubtractionAssignment));

  // Op Multiplication
  suite.test(TEST_CASE(TestOpMultiplication));

  // Op Multiplication Assignment
  suite.test(TEST_CASE(TestOpMultiplicationAssignment));

  // Op Equal To
  suite.test(TEST_CASE(TestOpEqualTo));

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
