#include "valhalla/midgard/linesegment2.h"

#include "test.h"

#include <vector>

#include "valhalla/midgard/point2.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

void TryDistance(const Point2& p, const LineSegment2& s, const float res,
                 const Point2& exp) {
  Point2 closest;
  float d = s.Distance(p, closest);
  if (fabs(d - res) > kEpsilon)
    throw runtime_error("Distance test failed - incorrect distance");
  if ((fabs(closest.x() - exp.x()) > kEpsilon ||
       fabs(closest.y() - exp.y()) > kEpsilon))
    throw runtime_error("Distance test failed - incorrect closest point");
}
void TestDistance() {
  // Test segment
  Point2 a(-2.0f, -2.0f);
  Point2 b( 4.0f, 4.0f);
  LineSegment2 s1(a, b);

  // Case 1 - point is "before start" of segment. a is closest
  TryDistance(Point2(-4.0f, -2.0f), s1, 2.0f, a);

  // Case 2 - point is after end of segment. b is closest
  TryDistance(Point2(6.0f, 4.0f), s1, 2.0f, b);

  // Case 3 - closest point is along segment
  TryDistance(Point2(0.0f, 2.0f), s1, sqrtf(2.0f), Point2(1.0f, 1.0f));
}

void TryIntersect(const LineSegment2& s1, const LineSegment2& s2,
                  const bool res, const Point2& exp) {
  Point2 intersect;
  bool doesintersect = s1.Intersect(s2, intersect);
  if (doesintersect != res)
    throw runtime_error("Intersect test failed - intersects is incorrect");
  if (doesintersect && (fabs(intersect.x() - exp.x()) > kEpsilon ||
          fabs(intersect.y() - exp.y()) > kEpsilon))
    throw runtime_error("Intersect test failed - intersection point is incorrect");
}
void TestIntersect() {
  LineSegment2 s1(Point2(-2.0f, -2.0f), Point2(4.0f, 4.0f));

  // Case 1 - beyond end of s1
  LineSegment2 s2(Point2(8.0f, 10.0f), Point2(4.0f, 2.0f));
  TryIntersect(s1, s2, false, Point2(0.0f, 0.0f));

  // Case 2 - before start of s1
  LineSegment2 s3(Point2(-10.0f, 5.0f), Point2(-14.0f, -5.0f));
  TryIntersect(s1, s3, false, Point2(0.0f, 0.0f));

  // Case 3 - s1 beyond end of s4
  LineSegment2 s4(Point2(0.0f, -5.0f), Point2(-1.0f, -2.0f));
  TryIntersect(s1, s4, false, Point2(0.0f, 0.0f));

  // Case 4 - s1 before start of s5
  LineSegment2 s5(Point2(0.0f, 5.0f), Point2(1.0f, 3.0f));
  TryIntersect(s1, s5, false, Point2(0.0f, 0.0f));

  // Case 3 - intersection
  LineSegment2 s6(Point2(-2.0f, 2.0f), Point2(2.0f, -2.0f));
  TryIntersect(s1, s6, true, Point2(0.0f, 0.0f));
}

void TryIsLeft(const Point2& p, const LineSegment2& s, const int res) {
  float d = s.IsLeft(p);
  if (res == 0 && fabs(d) > kEpsilon) {
    throw runtime_error("IsLeft test failed - should be on the segment");
  }
  if (res == -1 && d > -kEpsilon) {
    throw runtime_error("IsLeft test failed - should be right of the segment");
  }
  if (res == 1 && d < kEpsilon) {
      throw runtime_error("IsLeft test failed - should be right of the segment");
  }
}
void TestIsLeft() {
  // Use -1 for right of the segment, 0 for on the segment, and 1 for left
  // of the segment
  LineSegment2 s(Point2(-2.0f, -2.0f), Point2(4.0f, 4.0f));
  TryIsLeft(Point2(2.0f, 2.0f), s, 0);  // Should be on the line segment
  TryIsLeft(Point2(0.0f, 2.0f), s, 1);  // Should be left of the segment
  TryIsLeft(Point2(2.0f, 0.0f), s, -1); // Should be right of the segment
}

}

int main() {
  test::suite suite("point2");

  // Test distance of a point to a line segment
  suite.test(TEST_CASE(TestDistance));

  // Test if 2 line segments intersect
  suite.test(TEST_CASE(TestIntersect));

  // Test if a point is left, right, or on a line segment
  suite.test(TEST_CASE(TestIsLeft));

  return suite.tear_down();
}
