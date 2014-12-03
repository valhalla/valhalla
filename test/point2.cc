#include "test.h"

#include "include/config.h"

#include "geo/point2.h"
#include "geo/vector2.h"

using namespace std;
using namespace valhalla::geo;

namespace {

void TryPointSubtraction(const Point2& a, const Point2& b, const Vector2& res) {
  Vector2 v = b - a;
  if (v.x() != res.x() && v.y() != res.y())
    throw runtime_error("Point subtraction test failed");
}
void TestPointSubtraction() {
  TryPointSubtraction(Point2(4.0f, 4.0f), Point2(8.0f, 8.0f),
                      Vector2(4.0f, 4.0f));
}

void TryPointMinusVector(const Point2& p, const Vector2& v,
                         const Vector2& res) {
  Point2 c = p - v;
  if (c.x() != res.x() && c.y() != res.y())
    throw runtime_error("Point minus vector test failed");
}
void TestPointMinusVector() {
  TryPointMinusVector(Point2(8.0f, 8.0f), Vector2(4.0f, 4.0f),
                      Point2(4.0f, 4.0f));
}

void TryPointPlusVector(const Point2& p, const Vector2& v, const Point2& res) {
  Point2 c = p + v;
  if (c.x() != res.x() && c.y() != res.y())
    throw runtime_error("Point plus vector test failed");
}
void TestPointPlusVector() {
  TryPointPlusVector(Point2(4.0f, 4.0f), Vector2(4.0f, 4.0f),
                      Point2(8.0f, 8.0f));
}

void TryMidpoint(const Point2& a, const Point2& b, const Point2& res) {
  Point2 m = a.MidPoint(b);
  if (m.x() != res.x() && m.y() != res.y())
    throw runtime_error("MidPoint test failed");
}
void TestMidpoint() {
  TryMidpoint(Point2(4.0f, 4.0f), Point2(8.0f, 8.0f), Point2(6.0f, 6.0f));
}

void TryAffineCombination(const Point2& a, const Point2& b, const float af,
                          const float bf, const Point2& res) {
  Point2 m = a.AffineCombination(af, bf, b);
  if (m.x() != res.x() && m.y() != res.y())
    throw runtime_error("AffineCombination test failed");
}
void TestAffineCombination() {
  TryAffineCombination(Point2(4.0f, 4.0f), Point2(8.0f, 8.0f), 0.25f, 0.75f,
                       Point2(7.0f, 7.0f));
}

void TryDistance(const Point2& a, const Point2&b, const float res) {
  float d = a.Distance(b);
  if (fabs(d - res) > kEpsilon)
    throw runtime_error("Distance test failed");
}
void TestDistance() {
  TryDistance(Point2(4.0f, 4.0f), Point2(7.0f, 8.0f), 5.0f);
}

void TryDistanceSquared(const Point2& a, const Point2&b, const float res) {
  float d = a.DistanceSquared(b);
  if (fabs(d - res) > kEpsilon)
    throw runtime_error("Distance test failed");
}
void TestDistanceSquared() {
  TryDistanceSquared(Point2(4.0f, 4.0f), Point2(8.0f, 8.0f), 32.0f);
}

}

int main() {
  test::suite suite("point2");

  // Subtraction of a point from another point yields a vector
  suite.test(TEST_CASE(TestPointSubtraction));

  // Subtract a vector from a point yields another point
  suite.test(TEST_CASE(TestPointMinusVector));

  // Add a vector to a point yields another point
  suite.test(TEST_CASE(TestPointPlusVector));

  // Midpoint
  suite.test(TEST_CASE(TestMidpoint));

  // AffineCombination
  suite.test(TEST_CASE(TestAffineCombination));

  // Distance
  suite.test(TEST_CASE(TestDistance));

  // DistanceSquared
  suite.test(TEST_CASE(TestDistanceSquared));

  // TODO: Closest point to a list of points

  return suite.tear_down();
}
