#include "midgard/linesegment2.h"

#include "test.h"

#include <vector>

#include "midgard/point2.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

void TestDefaultConstructor() {
  LineSegment2<Point2> l;
  LineSegment2<Point2> expected(Point2(0.0f, 0.0f), Point2(0.0f, 0.0f));
  if (!l.ApproximatelyEqual(expected)) {
    throw runtime_error("LineSegment2 default constructor test failed");
  }
}

void TryDistance(const Point2& p, const LineSegment2<Point2>& s, const float res, const Point2& exp) {
  Point2 closest;
  float d = s.Distance(p, closest);
  if (fabs(d - res) > kEpsilon)
    throw runtime_error("Distance test failed - incorrect distance");
  if ((fabs(closest.x() - exp.x()) > kEpsilon || fabs(closest.y() - exp.y()) > kEpsilon))
    throw runtime_error("Distance test failed - incorrect closest point");
}
void TestDistance() {
  // Test segment
  Point2 a(-2.0f, -2.0f);
  Point2 b(4.0f, 4.0f);
  LineSegment2<Point2> s1(a, b);

  // Case 1 - point is "before start" of segment. a is closest
  TryDistance(Point2(-4.0f, -2.0f), s1, 2.0f, a);

  // Case 2 - point is after end of segment. b is closest
  TryDistance(Point2(6.0f, 4.0f), s1, 2.0f, b);

  // Case 3 - closest point is along segment
  TryDistance(Point2(0.0f, 2.0f), s1, sqrtf(2.0f), Point2(1.0f, 1.0f));
}

void TryIntersect(const LineSegment2<Point2>& s1,
                  const LineSegment2<Point2>& s2,
                  const bool res,
                  const Point2& exp) {
  Point2 intersect;
  bool doesintersect = s1.Intersect(s2, intersect);
  if (doesintersect != res)
    throw runtime_error("Intersect test failed - intersects is incorrect");
  if (doesintersect &&
      (fabs(intersect.x() - exp.x()) > kEpsilon || fabs(intersect.y() - exp.y()) > kEpsilon))
    throw runtime_error("Intersect test failed - intersection point is incorrect");
}

void TryIntersectLL(const LineSegment2<PointLL>& s1,
                    const LineSegment2<PointLL>& s2,
                    const bool res,
                    const PointLL& exp) {
  PointLL intersect;
  bool doesintersect = s1.Intersect(s2, intersect);
  if (doesintersect != res)
    throw runtime_error("Intersect test failed - intersects is incorrect");
  if (doesintersect &&
      (fabs(intersect.x() - exp.x()) > kEpsilon || fabs(intersect.y() - exp.y()) > kEpsilon))
    throw runtime_error("Intersect test failed - intersection point is incorrect");
}

void TestIntersect() {
  LineSegment2<Point2> s1(Point2(-2.0f, -2.0f), Point2(4.0f, 4.0f));

  // Case 1 - beyond end of s1
  LineSegment2<Point2> s2(Point2(8.0f, 10.0f), Point2(4.0f, 2.0f));
  TryIntersect(s1, s2, false, Point2(0.0f, 0.0f));

  // Case 2 - before start of s1
  LineSegment2<Point2> s3(Point2(-10.0f, 5.0f), Point2(-14.0f, -5.0f));
  TryIntersect(s1, s3, false, Point2(0.0f, 0.0f));

  // Case 3 - s1 beyond end of s4
  LineSegment2<Point2> s4(Point2(0.0f, -5.0f), Point2(-1.0f, -2.0f));
  TryIntersect(s1, s4, false, Point2(0.0f, 0.0f));

  // Case 4 - s1 before start of s5
  LineSegment2<Point2> s5(Point2(0.0f, 5.0f), Point2(1.0f, 3.0f));
  TryIntersect(s1, s5, false, Point2(0.0f, 0.0f));

  // Case 3 - intersection
  LineSegment2<Point2> s6(Point2(-2.0f, 2.0f), Point2(2.0f, -2.0f));
  TryIntersect(s1, s6, true, Point2(0.0f, 0.0f));

  // Case 4 - parallel line segments should not intersect
  LineSegment2<Point2> s7(Point2(-3.0f, -2.0f), Point2(3.0f, 4.0f));
  TryIntersect(s1, s7, false, Point2(0.0f, 0.0f));

  // Same cases with PointLL
  // Case 1 - beyond end of s1
  LineSegment2<PointLL> s1ll(PointLL(-2.0f, -2.0f), PointLL(4.0f, 4.0f));
  LineSegment2<PointLL> s2ll(PointLL(8.0f, 10.0f), PointLL(4.0f, 2.0f));
  TryIntersectLL(s1ll, s2ll, false, PointLL(0.0f, 0.0f));

  // Case 2 - before start of s1
  LineSegment2<PointLL> s3ll(PointLL(-10.0f, 5.0f), PointLL(-14.0f, -5.0f));
  TryIntersectLL(s1ll, s3ll, false, PointLL(0.0f, 0.0f));

  // Case 3 - s1 beyond end of s4
  LineSegment2<PointLL> s4ll(PointLL(0.0f, -5.0f), PointLL(-1.0f, -2.0f));
  TryIntersectLL(s1ll, s4ll, false, PointLL(0.0f, 0.0f));

  // Case 4 - s1 before start of s5
  LineSegment2<PointLL> s5ll(PointLL(0.0f, 5.0f), PointLL(1.0f, 3.0f));
  TryIntersectLL(s1ll, s5ll, false, PointLL(0.0f, 0.0f));

  // Case 3 - intersection
  LineSegment2<PointLL> s6ll(PointLL(-2.0f, 2.0f), PointLL(2.0f, -2.0f));
  TryIntersectLL(s1ll, s6ll, true, PointLL(0.0f, 0.0f));

  // Case 4 - parallel line segments should not intersect
  LineSegment2<PointLL> s7ll(PointLL(-3.0f, -2.0f), PointLL(3.0f, 4.0f));
  TryIntersectLL(s1ll, s7ll, false, PointLL(0.0f, 0.0f));
}

void TryPolyIntersect(const LineSegment2<Point2>& s1,
                      const std::vector<Point2>& poly,
                      const bool res) {
  if (s1.Intersect(poly) != res) {
    throw runtime_error("Polygon Intersect test failed");
  }
}

void TryPolyClip(const LineSegment2<Point2>& s1,
                 const std::vector<Point2>& poly,
                 const bool res,
                 LineSegment2<Point2>& clip_res) {
  LineSegment2<Point2> clip_segment;
  bool intersects = s1.ClipToPolygon(poly, clip_segment);
  if (intersects != res) {
    throw runtime_error("LineSegment ClipToPolygon intersection test failed");
  }
  if (!clip_res.ApproximatelyEqual(clip_segment)) {
    throw runtime_error(
        "LineSegment ClipToPolygon clipped segment mismatch: should be " +
        std::to_string(clip_segment.a().x()) + "," + std::to_string(clip_segment.a().y()) +
        " to: " + std::to_string(clip_segment.b().x()) + "," + std::to_string(clip_segment.b().y()));
  }
}

void TestPolyIntersect() {

  // Construct a convex polygon
  std::vector<Point2> poly = {{2.0f, 2.0f},
                              {0.0f, 4.0f},
                              {-10.0f, 0.0f},
                              {0.0f, -4.0f},
                              {2.0f, -2.0f}};

  // First point inside
  LineSegment2<Point2> s1(Point2(0.0f, 0.0f), Point2(4.0f, 12.0f));
  TryPolyIntersect(s1, poly, true);
  LineSegment2<Point2> clip1(Point2(0.0f, 0.0f), Point2(1.0f, 3.0f));
  TryPolyClip(s1, poly, true, clip1);

  // Second point inside
  LineSegment2<Point2> s2(Point2(4.0f, 12.0f), Point2(0.0f, 0.0f));
  TryPolyIntersect(s2, poly, true);
  LineSegment2<Point2> clip2(Point2(1.0f, 3.0f), Point2(0.0f, 0.0f));
  TryPolyClip(s2, poly, true, clip2);

  // Segment parallel to an edge and outside
  LineSegment2<Point2> s3(Point2(4.0f, -5.0f), Point2(4.0f, 5.0f));
  TryPolyIntersect(s3, poly, false);
  LineSegment2<Point2> clip3(Point2(0.0f, 0.0f), Point2(0.0f, 0.0f));
  TryPolyClip(s3, poly, false, clip3);

  // Passing through
  LineSegment2<Point2> s4(Point2(-5.0f, -5.0f), Point2(5.0f, 5.0f));
  TryPolyIntersect(s4, poly, true);
  LineSegment2<Point2> clip4(Point2(-2.857143f, -2.857143f), Point2(2.0f, 2.0f)); // TODO
  TryPolyClip(s4, poly, true, clip4);

  // No intersect with early out
  LineSegment2<Point2> s5(Point2(-10.0f, 5.0f), Point2(2.0f, 5.0f));
  TryPolyIntersect(s5, poly, false);
  LineSegment2<Point2> clip5(Point2(0.0f, 0.0f), Point2(0.0f, 0.0f));
  TryPolyClip(s5, poly, false, clip5);

  // Segment ends along an edge
  LineSegment2<Point2> s6(Point2(10.0f, 5.0f), Point2(2.0f, 0.0f));
  TryPolyIntersect(s6, poly, true);
  LineSegment2<Point2> clip6(Point2(2.0f, 0.0f), Point2(2.0f, 0.0f));
  TryPolyClip(s6, poly, true, clip6);
}

void TryIsLeft(const Point2& p, const LineSegment2<Point2>& s, const int res) {
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
  LineSegment2<Point2> s(Point2(-2.0f, -2.0f), Point2(4.0f, 4.0f));
  TryIsLeft(Point2(2.0f, 2.0f), s, 0);  // Should be on the line segment
  TryIsLeft(Point2(0.0f, 2.0f), s, 1);  // Should be left of the segment
  TryIsLeft(Point2(2.0f, 0.0f), s, -1); // Should be right of the segment
}

} // namespace

int main() {
  test::suite suite("point2");

  // Test the default constructor
  suite.test(TEST_CASE(TestDefaultConstructor));

  // Test distance of a point to a line segment
  suite.test(TEST_CASE(TestDistance));

  // Test if 2 line segments intersect
  suite.test(TEST_CASE(TestIntersect));

  // Test if line segment intersects polygon
  suite.test(TEST_CASE(TestPolyIntersect));

  // Test if a point is left, right, or on a line segment
  suite.test(TEST_CASE(TestIsLeft));

  return suite.tear_down();
}
