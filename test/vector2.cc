#include "midgard/vector2.h"
#include "midgard/point2.h"

#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

TEST(Vector2, TestCtorDefault) {
  Vector2 target;
  Vector2 expected{0.0f, 0.0f};
  EXPECT_EQ(expected, target);
}

void TryCtorPoint2(const Point2& pt, const Vector2& expected) {
  Vector2 result(pt);
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestCtorPoint2) {
  TryCtorPoint2(Point2(3.0f, 0.0f), Vector2(3.0f, 0.0f));
  TryCtorPoint2(Point2(-8.0f, 6.0f), Vector2(-8.0f, 6.0f));
}

void TryCtorFloatFloat(const float x, const float y, const Vector2& expected) {
  Vector2 result(x, y);
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestCtorFloatFloat) {
  TryCtorFloatFloat(3.0f, 0.0f, Vector2(3.0f, 0.0f));
  TryCtorFloatFloat(-8.0f, 6.0f, Vector2(-8.0f, 6.0f));
}

void TryCtorPoint2Point2(const Point2& from, const Point2& to, const Vector2& expected) {
  Vector2 result(from, to);
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestCtorPoint2Point2) {
  TryCtorPoint2Point2(Point2(4.0f, 0.0f), Point2(3.0f, 3.0f), Vector2(-1.0f, 3.0f));
  TryCtorPoint2Point2(Point2(4.0f, 2.0f), Point2(4.0f, -2.0f), Vector2(0.0f, -4.0f));
}

void TryCtorVector2(const Vector2& v, const Vector2& expected) {
  const Vector2& result(v);
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestCtorVector2) {
  TryCtorVector2(Vector2(3.0f, 0.0f), Vector2(3.0f, 0.0f));
  TryCtorVector2(Vector2(-8.0f, 6.0f), Vector2(-8.0f, 6.0f));
}

void TryOpAssignment(const Vector2& v, const Vector2& expected) {
  Vector2 result;
  result = v;
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestOpAssignment) {
  TryOpAssignment(Vector2(3.0f, 0.0f), Vector2(3.0f, 0.0f));
  TryOpAssignment(Vector2(-8.0f, 6.0f), Vector2(-8.0f, 6.0f));
}

void TryGet_x(const Vector2& v, const float expected) {
  EXPECT_FLOAT_EQ(expected, v.x());
}

TEST(Vector2, TestGet_x) {
  TryGet_x(Vector2(3.0f, 0.0f), 3.0f);
  TryGet_x(Vector2(-8.0f, 6.0f), -8.0f);
}

void TryGet_y(const Vector2& v, const float expected) {
  EXPECT_FLOAT_EQ(expected, v.y());
}

TEST(Vector2, TestGet_y) {
  TryGet_y(Vector2(3.0f, 2.0f), 2.0f);
  TryGet_y(Vector2(-8.0f, 6.0f), 6.0f);
}

void TrySet_x(Vector2& v, const float expected) {
  v.set_x(expected);
  EXPECT_FLOAT_EQ(expected, v.x());
}

TEST(Vector2, TestSet_x) {
  Vector2 v{1.0f, 1.0f};
  TrySet_x(v, 3.0f);
  TrySet_x(v, -8.0f);
}

void TrySet_y(Vector2& v, const float expected) {
  v.set_y(expected);
  EXPECT_FLOAT_EQ(expected, v.y());
}

TEST(Vector2, TestSet_y) {
  Vector2 v{1.0f, 1.0f};
  TrySet_y(v, 3.0f);
  TrySet_y(v, -8.0f);
}

void TrySetFloatFloat(const float x, const float y, const Vector2& expected) {
  Vector2 result;
  result.Set(x, y);
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestSetFloatFloat) {
  TrySetFloatFloat(3.0f, 0.0f, Vector2(3.0f, 0.0f));
  TrySetFloatFloat(-8.0f, 6.0f, Vector2(-8.0f, 6.0f));
}

void TrySetPoint2(const Point2& pt, const Vector2& expected) {
  Vector2 result;
  result.Set(pt);
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestSetPoint2) {
  TrySetPoint2(Point2(3.0f, 0.0f), Vector2(3.0f, 0.0f));
  TrySetPoint2(Point2(-8.0f, 6.0f), Vector2(-8.0f, 6.0f));
}

void TrySetPoint2Point2(const Point2& from, const Point2& to, const Vector2& expected) {
  Vector2 result;
  result.Set(from, to);
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestSetPoint2Point2) {
  TrySetPoint2Point2(Point2(4.0f, 0.0f), Point2(3.0f, 3.0f), Vector2(-1.0f, 3.0f));
  TrySetPoint2Point2(Point2(4.0f, 2.0f), Point2(4.0f, -2.0f), Vector2(0.0f, -4.0f));
}

void TryOpAddition(const Vector2& v, const Vector2& w, const Vector2& expected) {
  Vector2 result = v + w;
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestOpAddition) {
  TryOpAddition(Vector2(4.0f, -2.0f), Vector2(3.0f, 3.0f), Vector2(7.0f, 1.0f));
  TryOpAddition(Vector2(4.0f, 2.0f), Vector2(-2.0f, -2.0f), Vector2(2.0f, 0.0f));
}

void TryOpAdditionAssignment(Vector2& v, const Vector2& w, const Vector2& expected) {
  v += w;
  EXPECT_EQ(expected, v);
}

TEST(Vector2, TestOpAdditionAssignment) {
  Vector2 v1{4.0f, -2.0f};
  TryOpAdditionAssignment(v1, Vector2(3.0f, 3.0f), Vector2(7.0f, 1.0f));
  Vector2 v2{4.0f, 2.0f};
  TryOpAdditionAssignment(v2, Vector2(-2.0f, -2.0f), Vector2(2.0f, 0.0f));
}

void TryOpSubtraction(const Vector2& v, const Vector2& w, const Vector2& expected) {
  Vector2 result = v - w;
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestOpSubtraction) {
  TryOpSubtraction(Vector2(4.0f, -2.0f), Vector2(3.0f, 3.0f), Vector2(1.0f, -5.0f));
  TryOpSubtraction(Vector2(4.0f, 2.0f), Vector2(-2.0f, -2.0f), Vector2(6.0f, 4.0f));
}

void TryOpSubtractionAssignment(Vector2& v, const Vector2& w, const Vector2& expected) {
  v -= w;
  EXPECT_EQ(expected, v);
}

TEST(Vector2, TestOpSubtractionAssignment) {
  Vector2 v1{4.0f, -2.0f};
  TryOpSubtractionAssignment(v1, Vector2(3.0f, 3.0f), Vector2(1.0f, -5.0f));
  Vector2 v2{4.0f, 2.0f};
  TryOpSubtractionAssignment(v2, Vector2(-2.0f, -2.0f), Vector2(6.0f, 4.0f));
}

void TryOpMultiplication(const Vector2& v, const float scalar, const Vector2& expected) {
  Vector2 result = v * scalar;
  EXPECT_EQ(expected, result) << "scalar pre";

  Vector2 result2 = scalar * v;
  EXPECT_EQ(expected, result2) << "scalar post";
}

TEST(Vector2, TestOpMultiplication) {
  TryOpMultiplication(Vector2(4.0f, -2.0f), 3.0f, Vector2(12.0f, -6.0f));
  TryOpMultiplication(Vector2(-4.0f, 2.0f), -2.0f, Vector2(8.0f, -4.0f));
}

void TryOpMultiplicationAssignment(Vector2& v, const float scalar, const Vector2& expected) {
  v *= scalar;
  EXPECT_EQ(expected, v);
}

TEST(Vector2, TestOpMultiplicationAssignment) {
  Vector2 v1{4.0f, -2.0f};
  TryOpMultiplicationAssignment(v1, 3.0f, Vector2(12.0f, -6.0f));
  Vector2 v2{-4.0f, 2.0f};
  TryOpMultiplicationAssignment(v2, -2.0f, Vector2(8.0f, -4.0f));
}

void TryOpEqualTo(const Vector2& v, const Vector2& expected) {
  EXPECT_EQ(expected, v);
  EXPECT_EQ(v, expected);
}

TEST(Vector2, TestOpEqualTo) {
  TryOpEqualTo(Vector2(1.0f, 3.0f), Vector2(1.0f, 3.0f));
  TryOpEqualTo(Vector2(4.0f, -2.0f), Vector2(4.0f, -2.0f));
  TryOpEqualTo(Vector2(-4.0f, 2.0f), Vector2(-4.0f, 2.0f));
}

void TryDotProduct(const Vector2& a, const Vector2& b, float expected) {
  float result = a.Dot(b);
  EXPECT_FLOAT_EQ(expected, result);
}

TEST(Vector2, TestDotProduct) {
  TryDotProduct(Vector2(3.0f, 0.0f), Vector2(5.0f, 5.0f), 15.0f);
  TryDotProduct(Vector2(3.0f, 4.0f), Vector2(-8.0f, 6.0f), 0.0f);
}

void TryCrossProduct(const Vector2& a, const Vector2& b, float expected) {
  float result = a.Cross(b);
  EXPECT_FLOAT_EQ(expected, result);
}

TEST(Vector2, TestCrossProduct) {
  TryCrossProduct(Vector2(3.0f, 0.0f), Vector2(5.0f, 5.0f), 15.0f);
  TryCrossProduct(Vector2(3.0f, 4.0f), Vector2(-8.0f, 6.0f), 50.0f);
}

void TryPerpendicular(const Vector2& a, const Vector2& expected) {
  Vector2 result = a.GetPerpendicular();
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestPerpendicular) {
  TryPerpendicular(Vector2(3.0f, 4.0f), Vector2(-4.0f, 3.0f));
}

void TryNorm(const Vector2& a, float expected) {
  float result = a.Norm();
  EXPECT_FLOAT_EQ(expected, result);
}

TEST(Vector2, TestNorm) {
  TryNorm(Vector2(3.0f, 4.0f), 5.0f);
  TryNorm(Vector2(6.0f, 8.0f), 10.0f);
}

void TryNormSquared(const Vector2& a, float expected) {
  float result = a.NormSquared();
  EXPECT_FLOAT_EQ(expected, result);
}

TEST(Vector2, TestNormSquared) {
  TryNormSquared(Vector2(3.0f, 4.0f), 25.0f);
  TryNormSquared(Vector2(6.0f, 8.0f), 100.0f);
}

void TryNormalize(Vector2& a, const Vector2& expected) {
  a.Normalize();
  EXPECT_EQ(expected, a);
}

TEST(Vector2, TestNormalize) {
  Vector2 v(3.0f, 4.0f);
  TryNormalize(v, Vector2(3.0f / 5.0f, 4.0f / 5.0f));
  Vector2 w(6.0f, 8.0f);
  TryNormalize(w, Vector2(6.0f / 10.0f, 8.0f / 10.0f));
}

void TryComponent(const Vector2& a, const Vector2& b, float expected) {
  float result = a.Component(b);
  EXPECT_FLOAT_EQ(expected, result);
}

TEST(Vector2, TestComponent) {
  TryComponent(Vector2(3.0f, 4.0f), Vector2(6.0f, 8.0f), 0.5f);
  TryComponent(Vector2(6.0f, 8.0f), Vector2(3.0f, 4.0f), 2.0f);
}

void TryProjection(const Vector2& a, const Vector2& b, const Vector2& expected) {
  Vector2 result = a.Projection(b);
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestProjection) {
  TryProjection(Vector2(3.0f, 4.0f), Vector2(6.0f, 8.0f), Vector2(3.0f, 4.0f));
  TryProjection(Vector2(6.0f, 8.0f), Vector2(3.0f, 4.0f), Vector2(6.0f, 8.0f));
  TryProjection(Vector2(2.0f, 1.0f), Vector2(-3.0f, 4.0f), Vector2(6.0f / 25.0f, -8.0f / 25.0f));
}

void TryAngleBetween(const Vector2& a, const Vector2& b, float expected) {
  float result = (a.AngleBetween(b) * kDegPerRad);
  EXPECT_FLOAT_EQ(expected, result);
}

TEST(Vector2, TestAngleBetween) {
  TryAngleBetween(Vector2(3.0f, 0.0f), Vector2(5.0f, 5.0f), 45.0f);
  TryAngleBetween(Vector2(3.0f, 4.0f), Vector2(-8.0f, 6.0f), 90.0f);
}

void TryReflect(const Vector2& a, const Vector2& b, const Vector2& expected) {
  Vector2 result = a.Reflect(b);
  EXPECT_EQ(expected, result);
}

TEST(Vector2, TestReflect) {
  Vector2 n1{0.0f, 2.0f};
  n1.Normalize();
  TryReflect(Vector2(4.0f, -2.0f), n1, Vector2(4.0f, 2.0f));
  Vector2 n2{-3.0f, 0.0f};
  n2.Normalize();
  TryReflect(Vector2(3.0f, -4.0f), n2, Vector2(-3.0f, -4.0f));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
