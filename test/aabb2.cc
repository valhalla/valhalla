#include "midgard/aabb2.h"

#include "test.h"

#include "config.h"
#include <vector>

#include "midgard/point2.h"
#include "midgard/vector2.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

void TryIntersectsBb(const AABB2& a, const AABB2& b) {
  if (!a.Intersects(b))
    throw runtime_error("Intersecting BB test failed");
}

void TestIntersectsBb() {
  TryIntersectsBb(AABB2(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                AABB2(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryContainsBb(const AABB2& a, const AABB2& b) {
  if (!a.Contains(b))
    throw runtime_error("Contains BB test failed");
}

void TestContainsBb() {
  TryContainsBb(AABB2(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                AABB2(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryIntesectsLn(const AABB2& a, const AABB2& b) {
  if (!a.Intersect(a.Center(), b.Center()))
    throw runtime_error("Intersects line test failed");
}

void TestIntersectsLn() {
  TryIntesectsLn(AABB2(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                 AABB2(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryContainsPt(const AABB2& a, const AABB2& b) {
  if (!a.Contains(b.Center()))
    throw runtime_error("Contains point test failed");
}

void TestContainsPt() {
  TryContainsPt(AABB2(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                AABB2(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryEquality(const AABB2& a, const AABB2& b) {
  if (a == b)
    throw runtime_error("Equality test failed");
}

void TestEquality() {
  TryEquality(AABB2(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                AABB2(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryExpand(AABB2 a, const AABB2& b) {
  a.Expand(b);
  if (!(a == b))
    throw runtime_error("Expand test failed");
}

void TestExpand() {
  TryExpand(AABB2(40.0f, -76.4f, 40.1f, -76.3f),
            AABB2(39.8249f, -76.8013f, 40.2559f, -75.8997f));
}

void TryPtConstructor(const AABB2& a) {
  AABB2 b = AABB2(a.minpt(), a.maxpt());
  if (!(a == b))
    throw runtime_error("Point Constructor test failed");
}

void TestPtConstructor() {
  TryPtConstructor(AABB2(40.0f, -76.4f, 40.1f, -76.3f));
}

void TryMinMaxValues(const AABB2& a, float minx_res, float maxx_res,
                     float miny_res, float maxy_res) {
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
  TryMinMaxValues(AABB2(39.8249f, -76.8013f, 40.2559f, -75.8997f),
                  39.8249f, 40.2559f, -76.8013f, -75.8997f);
}

void TryTestWidth(const AABB2& a, float res) {
  if (fabs(a.Width() - res) > kEpsilon)
    throw runtime_error("Width test failed");
}

void TestWidth() {
  TryTestWidth(AABB2(39.8249f, -76.8013f, 40.2559f, -75.8997f), 0.431f);
}

void TryTestHeight(const AABB2& a, float res) {
  if (fabs(a.Height() - res) > kEpsilon)
    throw runtime_error("Area test failed");
}

void TestHeight() {
  TryTestHeight(AABB2(39.8249f, -76.8013f, 40.2559f, -75.8997f), 0.901604f);
}

void TryTestArea(const AABB2& a, float res) {
  if (fabs(a.Area() - res) > kEpsilon)
    throw runtime_error("Area test failed");
}

void TestArea() {
  TryTestArea(AABB2(39.8249f, -76.8013f, 40.2559f, -75.8997f), 0.388591f);
}

void TryTestVector(const AABB2& a, std::vector<Point2>& pts) {

  AABB2 b;
  b.Create(pts);
  if (!(a == b))
    throw runtime_error("Test Vector test failed");
}

void TestVector() {
  std::vector<Point2> pts;
  AABB2 a(39.8249f, -76.8013f, 40.2559f, -75.8997f);
  AABB2 b(40.0f, -76.4f, 40.1f, -76.3f);

  pts.push_back(a.Center());
  pts.push_back(a.maxpt());
  pts.push_back(a.minpt());

  pts.push_back(b.Center());
  pts.push_back(b.maxpt());
  pts.push_back(b.minpt());

  TryTestVector(a, pts);
}

}

int main() {
  test::suite suite("aabb2");

  // Tests if another bounding box intersects this bounding box
  suite.test(TEST_CASE(TestIntersectsBb));

  // Tests if another bounding box is inside this bounding box
  suite.test(TEST_CASE(TestContainsBb));

  // Tests if a segment intersects the bounding box
  suite.test(TEST_CASE(TestIntersectsLn));

  // Tests if a specified point is within the bounding box.
  suite.test(TEST_CASE(TestContainsPt));

  //Test equality operator.
  suite.test(TEST_CASE(TestEquality));

  //Test expand bounding box.
  suite.test(TEST_CASE(TestExpand));

  //Test minimum and maximum point constructor.
  suite.test(TEST_CASE(TestPtConstructor));

  //Test minimum and maximum values.
  suite.test(TEST_CASE(TestMinMaxValues));

  //Test width.
  suite.test(TEST_CASE(TestWidth));

  //Test height.
  suite.test(TEST_CASE(TestHeight));

  //Test area.
  suite.test(TEST_CASE(TestArea));

  //Test vector.
  suite.test(TEST_CASE(TestVector));

  return suite.tear_down();
}
