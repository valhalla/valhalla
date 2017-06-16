#include "midgard/point2.h"

#include "test.h"

#include <vector>

#include "midgard/vector2.h"

using namespace std;
using namespace valhalla::midgard;

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

void TryClosestPoint(const std::vector<Point2>& pts, const Point2&p,
                     const Point2& c, const int idx, const float res) {
  auto result = p.ClosestPoint(pts);
  if (fabs(std::get<1>(result) - res) > kEpsilon)
    throw runtime_error("ClosestPoint test failed - distance squared is wrong");
  if (idx != std::get<2>(result) )
      throw runtime_error("ClosestPoint test failed -index of closest segment is wrong");
  if (fabs(c.x() - std::get<0>(result).x()) > kEpsilon ||
      fabs(c.y() - std::get<0>(result).y()) > kEpsilon)
    throw runtime_error("ClosestPoint test failed - closest point is wrong");
}
void TestClosestPoint() {
  // Construct a simple polyline
  std::vector<Point2> pts = {
      { 0.0f, 0.0f },
      { 2.0f, 2.0f },
      { 4.0f, 2.0f },
      { 4.0f, 0.0f },
      { 12.0f, 0.0f }
  };

  // Closest to the first point
  TryClosestPoint(pts, Point2(-4.0f, 0.0f), Point2(0.0f, 0.0f), 0, 4.0f);

  // Closest along the last segment
  TryClosestPoint(pts, Point2(10.0f, -4.0f), Point2(10.0f, 0.0f), 3, 4.0f);

  // Closest to the last point
  TryClosestPoint(pts, Point2(15.0f, 4.0f), Point2(12.0f, 0.0f), 3, 5.0f);
}

void TryWithinConvexPolygon(const std::vector<Point2>& pts, const Point2&p,
                            const bool res) {
  if (p.WithinPolygon(pts) != res)
    throw runtime_error("TryWithinConvexPolygon test failed");
}

void TestWithinConvexPolygon() {
  // Construct a convex polygon
  std::vector<Point2> pts = {
      {   2.0f,  2.0f },
      {   0.0f,  4.0f },
      { -10.0f,  0.0f },
      {   0.0f, -4.0f },
      {   2.0f, -2.0f }
  };

  // Inside
  TryWithinConvexPolygon(pts, Point2(0.0f, 0.0f), true);

  // Check a vertex - should be inside
  TryWithinConvexPolygon(pts, Point2( 0.0f, -3.99f), true);
  TryWithinConvexPolygon(pts, Point2( 1.99f, -2.0f), true);

  // Outside
  TryWithinConvexPolygon(pts, Point2(15.0f, 4.0f), false);
  TryWithinConvexPolygon(pts, Point2( 2.5f, 0.0f), false);
  TryWithinConvexPolygon(pts, Point2(-3.0f, 3.0f), false);
  TryWithinConvexPolygon(pts, Point2( 1.0f,-3.5f), false);
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

  // Closest point to a list of points
  suite.test(TEST_CASE(TestClosestPoint));

  // Test if within polygon
  suite.test(TEST_CASE(TestWithinConvexPolygon));

  return suite.tear_down();
}
