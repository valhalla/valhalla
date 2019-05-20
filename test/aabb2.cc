#include "midgard/aabb2.h"

#include "test.h"

#include <vector>

#include "midgard/point2.h"
#include "midgard/vector2.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

void TryDefaultConstructor(const AABB2<Point2>& b) {
  if (!(b.minx() == 0.0f && b.miny() == 0.0f && b.maxx() == 0.0f && b.maxy() == 0.0f))
    throw runtime_error("Default constructor test failed");
}

void TestConstructor() {
  AABB2<Point2> b;
  TryDefaultConstructor(b);
}

void TryIntersectsBb(const AABB2<Point2>& a, const AABB2<Point2>& b) {
  if (!a.Intersects(b))
    throw runtime_error("Intersecting BB test failed");
}

void TestIntersectsBb() {
  TryIntersectsBb(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                  AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryContainsBb(const AABB2<Point2>& a, const AABB2<Point2>& b) {
  if (!a.Contains(b))
    throw runtime_error("Contains BB test failed");
}

void TestContainsBb() {
  TryContainsBb(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryIntesectsLn(const AABB2<Point2>& box, const Point2& a, const Point2& b, bool expected) {
  if (box.Intersects(a, b) != expected)
    throw runtime_error("Intersects line test failed");
}

void TestIntersectsLn() {
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
  if (!a.Contains(b.Center()))
    throw runtime_error("Contains point test failed");
}

void TestContainsPt() {
  TryContainsPt(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryEquality(const AABB2<Point2>& a, const AABB2<Point2>& b) {
  if (a == b)
    throw runtime_error("Equality test failed");
}

void TestEquality() {
  TryEquality(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f),
              AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryExpand(AABB2<Point2> a, const AABB2<Point2>& b) {
  a.Expand(b);
  if (!(a == b))
    throw runtime_error("Expand test failed");
}

void TestExpand() {
  TryExpand(AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f),
            AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f));
}

void TryExpandPointMin(AABB2<Point2> a, const Point2& p) {
  a.Expand(p);
  if (!(a.miny() == p.y() && a.minx() == p.x()))
    throw runtime_error("Expand test failed");
}

void TryExpandPointMax(AABB2<Point2> a, const Point2& p) {
  a.Expand(p);
  if (!(a.maxy() == p.y() && a.maxx() == p.x()))
    throw runtime_error("Expand test failed");
}

void TestExpandPoint() {
  TryExpandPointMin(AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f), Point2(39.8f, -76.8f));

  TryExpandPointMax(AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f), Point2(40.8f, -76.1f));
}

void TryPtConstructor(const AABB2<Point2>& a) {
  AABB2<Point2> b = AABB2<Point2>(a.minpt(), a.maxpt());
  if (!(a == b))
    throw runtime_error("Point Constructor test failed");
}

void TestPtConstructor() {
  TryPtConstructor(AABB2<Point2>(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryMinMaxValues(const AABB2<Point2>& a,
                     float minx_res,
                     float maxx_res,
                     float miny_res,
                     float maxy_res) {
  if (fabs(a.minx() - minx_res) > kEpsilon)
    throw runtime_error("Min x value test failed");

  if (fabs(a.maxx() - maxx_res) > kEpsilon)
    throw runtime_error("Max x value test failed");

  if (fabs(a.miny() - miny_res) > kEpsilon)
    throw runtime_error("Min y value test failed");

  if (fabs(a.maxy() - maxy_res) > kEpsilon)
    throw runtime_error("Max y value test failed");
}

void TestMinMaxValues() {
  TryMinMaxValues(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f), 39.8249f, 40.2559f,
                  -76.8013f, -75.8997f);
}

void TryTestWidth(const AABB2<Point2>& a, float res) {
  if (fabs(a.Width() - res) > kEpsilon)
    throw runtime_error("Width test failed");
}

void TestWidth() {
  TryTestWidth(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f), 0.431f);
}

void TryTestHeight(const AABB2<Point2>& a, float res) {
  if (fabs(a.Height() - res) > kEpsilon)
    throw runtime_error("Height test failed");
}

void TestHeight() {
  TryTestHeight(AABB2<Point2>(39.8249f, -76.8013f, 40.2559f, -75.8997f), 0.901604f);
}

void TryTestVector(const AABB2<Point2>& a, std::vector<Point2>& pts) {

  AABB2<Point2> b(pts);
  if (!(a == b))
    throw runtime_error("Test Vector test failed");
}

void TestVector() {
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

void TestIntersectsCircle() {
  auto check = [](bool a, bool b) {
    if (a != b)
      throw std::logic_error(a ? "Circle DOES intersect" : "Circle DOESNT intersect");
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

void TestIntersect() {
  // Test if bounding boxes intersect
  AABB2<Point2> box(-1, -1, 1, 1);
  Point2 a, b;
  if (!box.Intersect((a = {0, 0}), (b = {1, 1})) || a != Point2{0, 0} || b != Point2{1, 1})
    throw std::logic_error("Wrong intersection");
  if (!box.Intersect((a = {-2, 0}), (b = {2, 0})) || a != Point2{-1, 0} || b != Point2{1, 0})
    throw std::logic_error("Wrong intersection");
  if (!box.Intersect((a = {-2, -2}), (b = {2, 2})) || a != Point2{-1, -1} || b != Point2{1, 1})
    throw std::logic_error("Wrong intersection");
  if (!box.Intersect((a = {-2, -2}), (b = {0, 0})) || a != Point2{-1, -1} || b != Point2{0, 0})
    throw std::logic_error("Wrong intersection");
  if (!box.Intersect((a = {0, 0}), (b = {2, 2})) || a != Point2{0, 0} || b != Point2{1, 1})
    throw std::logic_error("Wrong intersection");
  if (!box.Intersect((a = {-1, 1}), (b = {1, -1})) || a != Point2{-1, 1} || b != Point2{1, -1})
    throw std::logic_error("Wrong intersection");
  if (!box.Intersect((a = {0, 2}), (b = {2, 0})) || a != Point2{1, 1} || b != Point2{1, 1})
    throw std::logic_error("Wrong intersection");

  LineSegment2<Point2> ab(a, b);
  if (!box.Intersects(ab))
    throw std::logic_error("LineSegment intersects test failed");

  if (box.Intersect((a = {-2, -2}), (b = {-1, -1.001})))
    throw std::logic_error("Wrong intersection");
  if (box.Intersect((a = {0, 2.1}), (b = {2.1, 0})))
    throw std::logic_error("Wrong intersection");
  if (box.Intersect((a = {0, 1.1}), (b = {1, 1.1})))
    throw std::logic_error("Wrong intersection");
  if (box.Intersect((a = {1.1, 0}), (b = {1, 1.1})))
    throw std::logic_error("Wrong intersection");

  // Test intersection of bounding boxes
  // Case 1 - no intersection
  AABB2<Point2> intersect1 = box.Intersection({2, 2, 3, 3});
  if (intersect1.minx() != 0.0f || intersect1.miny() != 0.0f || intersect1.maxx() != 0.0f ||
      intersect1.maxy() != 0.0f)
    throw std::logic_error("Wrong intersection 1");

  // Case 2 intersection.
  AABB2<Point2> intersect2 = box.Intersection({0, 0, 3, 3});
  if (intersect2.minx() != 0.0f || intersect2.miny() != 0.0f || intersect2.maxx() != 1.0f ||
      intersect2.maxy() != 1.0f)
    throw std::logic_error("Wrong intersection 2");

  // Case 3 - other bounding box contains this box
  AABB2<Point2> intersect3 = box.Intersection({-3, -3, 3, 3});
  if (intersect3.minx() != -1.0f || intersect3.miny() != -1.0f || intersect3.maxx() != 1.0f ||
      intersect3.maxy() != 1.0f)
    throw std::logic_error("Wrong intersection 3");

  // Case 4 - box contains other bounding box
  AABB2<Point2> intersect4 = box.Intersection({-0.5f, -0.5f, 0.5f, 0.5f});
  if (intersect4.minx() != -0.5f || intersect4.miny() != -0.5f || intersect4.maxx() != 0.5f ||
      intersect4.maxy() != 0.5f)
    throw std::logic_error("Wrong intersection 4");
}

} // namespace

int main() {
  test::suite suite("aabb2");

  // Test the default constructor
  suite.test(TEST_CASE(TestConstructor));

  // Tests if another bounding box intersects this bounding box
  suite.test(TEST_CASE(TestIntersectsBb));

  // Tests if another bounding box is inside this bounding box
  suite.test(TEST_CASE(TestContainsBb));

  // Tests if a segment intersects the bounding box
  suite.test(TEST_CASE(TestIntersectsLn));

  // Tests if a specified point is within the bounding box.
  suite.test(TEST_CASE(TestContainsPt));

  // Test equality operator.
  suite.test(TEST_CASE(TestEquality));

  // Test expand bounding box.
  suite.test(TEST_CASE(TestExpand));

  // Test expand bounding box by a point.
  suite.test(TEST_CASE(TestExpandPoint));

  // Test minimum and maximum point constructor.
  suite.test(TEST_CASE(TestPtConstructor));

  // Test minimum and maximum values.
  suite.test(TEST_CASE(TestMinMaxValues));

  // Test width.
  suite.test(TEST_CASE(TestWidth));

  // Test height.
  suite.test(TEST_CASE(TestHeight));

  // Test vector.
  suite.test(TEST_CASE(TestVector));

  suite.test(TEST_CASE(TestIntersectsCircle));

  suite.test(TEST_CASE(TestIntersect));

  return suite.tear_down();
}
