#include "midgard/point2.h"

#include <list>
#include <unordered_map>
#include <vector>

#include "midgard/vector2.h"

#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

void TryPointSubtraction(const Point2& a, const Point2& b, const Vector2& res) {
  Vector2 v = b - a;
  EXPECT_EQ(v.x(), res.x());
  EXPECT_EQ(v.y(), res.y());
}
TEST(Point2, TestPointSubtraction) {
  TryPointSubtraction(Point2(4.0f, 4.0f), Point2(8.0f, 8.0f), Vector2(4.0f, 4.0f));
}

void TryPointMinusVector(const Point2& p, const Vector2& v, const Vector2& res) {
  Point2 c = p - v;
  EXPECT_EQ(c.x(), res.x());
  EXPECT_EQ(c.y(), res.y());
}
TEST(Point2, TestPointMinusVector) {
  TryPointMinusVector(Point2(8.0f, 8.0f), Vector2(4.0f, 4.0f), Point2(4.0f, 4.0f));
}

void TryPointPlusVector(const Point2& p, const Vector2& v, const Point2& res) {
  Point2 c = p + v;
  EXPECT_EQ(c.x(), res.x());
  EXPECT_EQ(c.y(), res.y());
}
TEST(Point2, TestPointPlusVector) {
  TryPointPlusVector(Point2(4.0f, 4.0f), Vector2(4.0f, 4.0f), Point2(8.0f, 8.0f));
}

void TryMidpoint(const Point2& a, const Point2& b, const Point2& res, float mp = .5f) {
  Point2 m = a.PointAlongSegment(b, mp);
  EXPECT_EQ(m.x(), res.x());
  EXPECT_EQ(m.y(), res.y());
}
TEST(Point2, TestMidpoint) {
  TryMidpoint(Point2(4.0f, 4.0f), Point2(8.0f, 8.0f), Point2(6.0f, 6.0f));
  TryMidpoint(Point2(4.0f, 4.0f), Point2(8.0f, 8.0f), Point2(7.0f, 7.0f), 0.75f);
}

void TryDistance(const Point2& a, const Point2& b, const float res) {
  float d = a.Distance(b);
  EXPECT_NEAR(d, res, kEpsilon);
}
TEST(Point2, TestDistance) {
  TryDistance(Point2(4.0f, 4.0f), Point2(7.0f, 8.0f), 5.0f);
}

void TryDistanceSquared(const Point2& a, const Point2& b, const float res) {
  float d = a.DistanceSquared(b);
  EXPECT_NEAR(d, res, kEpsilon);
}
TEST(Point2, TestDistanceSquared) {
  TryDistanceSquared(Point2(4.0f, 4.0f), Point2(8.0f, 8.0f), 32.0f);
}

void TryClosestPoint(const std::vector<Point2>& pts,
                     const Point2& p,
                     const Point2& c,
                     const int idx,
                     const float res) {
  auto result = p.ClosestPoint(pts);

  EXPECT_NEAR(std::get<1>(result), res, kEpsilon) << "ClosestPoint test failed - distance is wrong";

  EXPECT_EQ(idx, std::get<2>(result))
      << "ClosestPoint test failed -index of closest segment is wrong";

  EXPECT_NEAR(c.x(), std::get<0>(result).x(), kEpsilon);
  EXPECT_NEAR(c.y(), std::get<0>(result).y(), kEpsilon);
}
TEST(Point2, TestClosestPoint) {
  // Construct a simple polyline (duplicate a point to make sure it is properly skipped)
  std::vector<Point2> pts = {{0.0f, 0.0f}, {2.0f, 2.0f}, {4.0f, 2.0f},
                             {4.0f, 0.0f}, {4.0f, 0.0f}, {12.0f, 0.0f}};

  // Closest to the first point
  TryClosestPoint(pts, Point2(-4.0f, 0.0f), Point2(0.0f, 0.0f), 0, 4.0f);

  // Closest along the last segment
  TryClosestPoint(pts, Point2(10.0f, -4.0f), Point2(10.0f, 0.0f), 4, 4.0f);

  // Closest to the last point
  TryClosestPoint(pts, Point2(15.0f, 4.0f), Point2(12.0f, 0.0f), 4, 5.0f);

  // Test ClosestPoint with empty vector
  std::vector<Point2> empty_pts;
  TryClosestPoint(empty_pts, Point2(5.0f, 0.0f), Point2(0.0f, 0.0f), 0,
                  std::numeric_limits<float>::max());

  // Test ClosestPoint with only 1 point in the list
  std::vector<Point2> pts1 = {{1.0f, 0.0f}};
  TryClosestPoint(pts1, Point2(5.0f, 0.0f), Point2(1.0f, 0.0f), 0, 4.0f);
}

void TryWithinConvexPolygon(const std::vector<Point2>& pts, const Point2& p, const bool res) {
  EXPECT_EQ(p.WithinPolygon(pts), res);
}

void TryWithinConvexPolygonList(const std::list<Point2>& pts, const Point2& p, const bool res) {
  EXPECT_EQ(p.WithinPolygon(pts), res);
}

TEST(Point2, TestWithinConvexPolygon) {
  // Construct a convex polygon
  std::vector<Point2> pts = {{2.0f, 2.0f},
                             {0.0f, 4.0f},
                             {-10.0f, 0.0f},
                             {0.0f, -4.0f},
                             {2.0f, -2.0f}};

  // Inside
  TryWithinConvexPolygon(pts, Point2(0.0f, 0.0f), true);

  // Check a vertex - should be inside
  TryWithinConvexPolygon(pts, Point2(0.0f, -3.99f), true);
  TryWithinConvexPolygon(pts, Point2(1.99f, -2.0f), true);

  // Outside
  TryWithinConvexPolygon(pts, Point2(15.0f, 4.0f), false);
  TryWithinConvexPolygon(pts, Point2(2.5f, 0.0f), false);
  TryWithinConvexPolygon(pts, Point2(-3.0f, 3.0f), false);
  TryWithinConvexPolygon(pts, Point2(1.0f, -3.5f), false);

  std::list<Point2> ptslist = {{2.0f, 2.0f},
                               {0.0f, 4.0f},
                               {-10.0f, 0.0f},
                               {0.0f, -4.0f},
                               {2.0f, -2.0f}};
  TryWithinConvexPolygonList(ptslist, Point2(0.0f, 0.0f), true);
}

TEST(Point2, TestHash) {
  Point2 a(10.5f, -100.0f);
  std::unordered_map<Point2, int> m{{a, 1}};
  EXPECT_NE(m.find(a), m.cend()) << "Should have found a";

  Point2 b(1.5f, 1.0f);
  EXPECT_TRUE(m.insert({b, 2}).second) << "Should not have found b";
  EXPECT_NE(m.find(b), m.cend()) << "Should have found b";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
