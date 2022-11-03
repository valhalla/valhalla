#include "midgard/aabb2.h"

#include <vector>

#include "midgard/point2.h"
#include "midgard/vector2.h"

#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

void TryDefaultConstructor(const AABB2<Point2>& b) {
  if (!(b.minx() == 0.0f && b.miny() == 0.0f && b.maxx() == 0.0f && b.maxy() == 0.0f))
    throw runtime_error("Default constructor test failed");
}

TEST(AABB2, TestConstructor) {
  AABB2<Point2> b;
  TryDefaultConstructor(b);
}

void TryIntersectsBb(const AABB2<Point2>& a, const AABB2<Point2>& b) {
  EXPECT_TRUE(a.Intersects(b)) << "Intersecting BB test failed";
}

TEST(AABB2, TestIntersectsBb) {
  TryIntersectsBb(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                  AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryContainsBb(const AABB2<Point2>& a, const AABB2<Point2>& b) {
  EXPECT_TRUE(a.Contains(b)) << "Contains BB test failed";
}

TEST(AABB2, TestContainsBb) {
  TryContainsBb(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryIntesectsLn(const AABB2<Point2>& box, const Point2& a, const Point2& b, bool expected) {
  EXPECT_EQ(box.Intersects(a, b), expected) << "Intersects line test failed";
}

TEST(AABB2, TestIntersectsLn) {
  AABB2<Point2> box(40.0f, -76.0f, 41.0f, -75.0f);

  // Test with one or both points in the box
  TryIntesectsLn(box, Point2(40.5f, -75.5f), Point2(41.5f, -75.5f), true);
  TryIntesectsLn(box, Point2(38.0f, -80.0f), Point2(40.5f, -75.5f), true);
  TryIntesectsLn(box, Point2(40.5f, -75.5f), Point2(40.8f, -75.8f), true);

  // Quick rejection tests
  TryIntesectsLn(box, Point2(42.5f, -76.5f), Point2(41.5f, -75.5f), false);
  TryIntesectsLn(box, Point2(42.5f, -80.5f), Point2(41.5f, -85.5f), false);
  TryIntesectsLn(box, Point2(42.5f, -70.5f), Point2(41.5f, -75.5f), false);
  TryIntesectsLn(box, Point2(26.5f, -80.5f), Point2(39.5f, -85.5f), false);

  // Endpoint on the boundary
  TryIntesectsLn(box, Point2(40.0f, -75.5f), Point2(36.5f, -74.0f), true);
  TryIntesectsLn(box, Point2(40.0f, -75.0f), Point2(36.5f, -74.0f), true);

  // Through the box (horizontal, vertical, other)
  TryIntesectsLn(box, Point2(40.5f, -77.0f), Point2(40.5f, -74.0f), true);
  TryIntesectsLn(box, Point2(39.5f, -75.5f), Point2(41.5f, -75.5f), true);
  TryIntesectsLn(box, Point2(39.5f, -75.9f), Point2(40.5f, -74.8f), true);

  // Outside the corner
  TryIntesectsLn(box, Point2(39.2f, -75.5f), Point2(40.5f, -74.5f), false);
}

void TryContainsPt(const AABB2<Point2>& a, const AABB2<Point2>& b) {
  EXPECT_TRUE(a.Contains(b.Center())) << "Contains point test failed";
}

TEST(AABB2, TestContainsPt) {
  TryContainsPt(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryEquality(const AABB2<Point2>& a, const AABB2<Point2>& b) {
  // Point2 does not have operator !=
  EXPECT_TRUE(!(a == b)) << "Equality test failed";
}

TEST(AABB2, TestEquality) {
  TryEquality(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f),
              AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryExpand(AABB2<Point2> a, const AABB2<Point2>& b) {
  a.Expand(b);
  EXPECT_EQ(a, b) << "Expand test failed";
}

TEST(AABB2, TestExpand) {
  TryExpand(AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f),
            AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f));
}

void TryExpandPointMin(AABB2<Point2> a, const Point2& p) {
  a.Expand(p);
  EXPECT_EQ(a.miny(), p.y());
  EXPECT_EQ(a.minx(), p.x());
}

void TryExpandPointMax(AABB2<Point2> a, const Point2& p) {
  a.Expand(p);
  EXPECT_EQ(a.maxy(), p.y());
  EXPECT_EQ(a.maxx(), p.x());
}

TEST(AABB2, TestExpandPoint) {
  TryExpandPointMin(AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f), Point2(39.8f, -76.8f));

  TryExpandPointMax(AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f), Point2(40.8f, -76.1f));
}

void TryPtConstructor(const AABB2<Point2>& a) {
  AABB2<Point2> b = AABB2<Point2>(a.minpt(), a.maxpt());
  EXPECT_EQ(a, b) << "Point Constructor test failed";
}

TEST(AABB2, TestPtConstructor) {
  TryPtConstructor(AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryMinMaxValues(const AABB2<Point2>& a,
                     float minx_res,
                     float maxx_res,
                     float miny_res,
                     float maxy_res) {
  EXPECT_NEAR(a.minx(), minx_res, kEpsilon);
  EXPECT_NEAR(a.maxx(), maxx_res, kEpsilon);
  EXPECT_NEAR(a.miny(), miny_res, kEpsilon);
  EXPECT_NEAR(a.maxy(), maxy_res, kEpsilon);
}

TEST(AABB2, TestMinMaxValues) {
  TryMinMaxValues(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f), 39.8249f, 40.2559f,
                  -76.8013f, -75.8997f);
}

void TryTestWidth(const AABB2<Point2>& a, float res) {
  EXPECT_NEAR(a.Width(), res, kEpsilon);
}

TEST(AABB2, TestWidth) {
  TryTestWidth(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f), 0.431f);
}

void TryTestHeight(const AABB2<Point2>& a, float res) {
  EXPECT_NEAR(a.Height(), res, kEpsilon);
}

TEST(AABB2, TestHeight) {
  TryTestHeight(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f), 0.901604f);
}

void TryTestVector(const AABB2<Point2>& a, std::vector<Point2>& pts) {
  AABB2<Point2> b(pts);
  EXPECT_EQ(a, b) << "Test Vector test failed";
}

TEST(AABB2, TestVector) {
  std::vector<Point2> pts;
  AABB2<Point2> a(39.8249f, -76.8013f, 40.2559f, -75.8997f);
  AABB2<Point2> b(40.0f, -76.4f, 40.1f, -76.3f);

  pts.push_back(a.Center());
  pts.push_back(a.maxpt());
  pts.push_back(a.minpt());

  pts.push_back(b.Center());
  pts.push_back(b.maxpt());
  pts.push_back(b.minpt());

  TryTestVector(a, pts);
}

TEST(AABB2, TestIntersectsCircle) {
  auto check = [](bool a, bool b) {
    EXPECT_EQ(a, b) << (a ? "Circle DOES intersect" : "Circle DOESNT intersect");
  };

  AABB2<Point2> box(-1, -1, 1, 1);
  check(box.Intersects({0, 0}, 1), true);
  check(box.Intersects({0, 0}, 100), true);
  check(box.Intersects({2, 1}, 1), true);
  check(box.Intersects({-2, -1}, 1), true);
  check(box.Intersects({-1.5, -1.5}, 0.1), false);
  check(box.Intersects({0, 5}, 4.1), true);
  check(box.Intersects({2, -2}, 1.415), true);
  check(box.Intersects({-2, 2}, 1.413), false);
}

TEST(AABB2, TestIntersect) {
  // Test if bounding boxes intersect
  AABB2<Point2> box(-1, -1, 1, 1);

  // Test intersection of bounding boxes
  // Case 1 - no intersection
  AABB2<Point2> intersect1 = box.Intersection({2, 2, 3, 3});
  EXPECT_EQ(intersect1.minx(), 0.0f);
  EXPECT_EQ(intersect1.miny(), 0.0f);
  EXPECT_EQ(intersect1.maxx(), 0.0f);
  EXPECT_EQ(intersect1.maxy(), 0.0f);

  // Case 2 intersection.
  AABB2<Point2> intersect2 = box.Intersection({0, 0, 3, 3});
  EXPECT_EQ(intersect2.minx(), 0.0f);
  EXPECT_EQ(intersect2.miny(), 0.0f);
  EXPECT_EQ(intersect2.maxx(), 1.0f);
  EXPECT_EQ(intersect2.maxy(), 1.0f);

  // Case 3 - other bounding box contains this box
  AABB2<Point2> intersect3 = box.Intersection({-3, -3, 3, 3});
  EXPECT_EQ(intersect3.minx(), -1.0f);
  EXPECT_EQ(intersect3.miny(), -1.0f);
  EXPECT_EQ(intersect3.maxx(), 1.0f);
  EXPECT_EQ(intersect3.maxy(), 1.0f);

  // Case 4 - box contains other bounding box
  AABB2<Point2> intersect4 = box.Intersection({-0.5f, -0.5f, 0.5f, 0.5f});
  EXPECT_EQ(intersect4.minx(), -0.5f);
  EXPECT_EQ(intersect4.miny(), -0.5f);
  EXPECT_EQ(intersect4.maxx(), 0.5f);
  EXPECT_EQ(intersect4.maxy(), 0.5f);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
