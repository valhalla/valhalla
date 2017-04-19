#include <cstdint>
#include "midgard/polyline2.h"

#include "test.h"

#include <vector>

#include "midgard/point2.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

void TryGeneralizeAndLength(Polyline2<Point2>& pl, const float& gen, const float& res) {
  uint32_t size = pl.Generalize(gen);

  std::vector<Point2> pts = pl.pts();

  if (pl.pts().size() != 2)
    throw runtime_error("Generalize #1 test failed.");

  if ((pts[0] != Point2(25.0f, 25.0f)) || (pts[1] != Point2(50.0f, 100.0f))) {
    throw runtime_error("Generalize #2 test failed.");
  }

  Polyline2<Point2> pl2;
  pl2 = pl.GeneralizedPolyline(gen);

  if (pl2.pts().size() != 2)
    throw runtime_error("Generalize #3 test failed.");

  if ((pl2.pts().at(0) != Point2(25.0f, 25.0f))
      || (pl2.pts().at(1) != Point2(50.0f, 100.0f))) {
    throw runtime_error("Generalize #2 test failed.");
  }

  if (!(fabs(pl2.Length() - res) > kEpsilon))
    throw runtime_error("Length test failed.");
}

void TestGeneralizeAndLength() {
  std::vector<Point2> pts = { Point2(25.0f, 25.0f), Point2(50.0f, 50.0f),
      Point2(25.0f, 75.0f), Point2(50.0f, 100.0f) };
  Polyline2<Point2> pl(pts);
  TryGeneralizeAndLength(pl, 100.0f, 79.0569f);
}

void TryClosestPoint(const Polyline2<Point2>& pl, const Point2& a, const Point2& b) {

  auto result = pl.ClosestPoint(a);

  if (std::get<0>(result)!= b)
    throw runtime_error("ClosestPoint test failed.");
}

void TestClosestPoint() {
  Point2 a(25.0f, 25.0f);
  Point2 b(50.0f, 50.0f);
  Point2 c(25.0f, 75.0f);
  Point2 d(50.0f, 100.0f);

  // Test adding points to polyline
  Polyline2<Point2> pl;
  pl.Add(a);
  pl.Add(b);
  pl.Add(c);
  pl.Add(d);

  Point2 beg(0.0f, 0.0f);
  TryClosestPoint(pl, beg, a);

  Point2 mid(60.0f, 50.0f);
  TryClosestPoint(pl, mid, b);

  Point2 end(50.0f, 125.0f);
  TryClosestPoint(pl, end, d);
}

void TryClip(Polyline2<Point2>& pl, const AABB2<Point2>& a, const uint32_t exp) {
  // Clip and check vertex count and 1st 2 points
  uint32_t x = pl.Clip(a);
  if (x != exp)
    throw runtime_error("Clip test failed: count not correct");

  if ((pl.pts().at(0) != Point2(25.0f, 25.0f))
      || (pl.pts().at(1) != Point2(50.0f, 50.0f))) {
    throw runtime_error("Clip test failed: clipped points not correct");
  }
}

void TestClip() {
  std::vector<Point2> pts = { Point2(25.0f, 25.0f), Point2(50.0f, 50.0f),
      Point2(25.0f, 75.0f), Point2(50.0f, 100.0f) };
  Polyline2<Point2> pl(pts);
  TryClip(pl, AABB2<Point2>(Point2(0.0f, 0.0f), Point2(75.0f, 50.0f)), 2);

  // Test with vertices on edges
  Polyline2<Point2> pl2(pts);
  TryClip(pl2, AABB2<Point2>(Point2(25.0f, 25.0f), Point2(50.0f, 100.0f)), 4);
}

void TryClippedPolyline(Polyline2<Point2>& pl, const AABB2<Point2>& a, const uint32_t exp) {
  Polyline2<Point2> pl2 = pl.ClippedPolyline(a);
  uint32_t x = pl2.pts().size();
  if (x != 2)
    throw runtime_error("ClippedPolyline test failed: count not correct");

  if ((pl2.pts().at(0) != Point2(25.0f, 25.0f))
      || (pl2.pts().at(1) != Point2(50.0f, 50.0f))) {
    throw runtime_error(
        "ClippedPolyline test failed: clipped points not correct");
  }
}

void TestClippedPolyline() {
  std::vector<Point2> pts = { Point2(25.0f, 25.0f), Point2(50.0f, 50.0f),
      Point2(25.0f, 75.0f), Point2(50.0f, 100.0f) };
  Polyline2<Point2> pl(pts);
  TryClippedPolyline(pl, AABB2<Point2>(Point2(0.0f, 0.0f), Point2(75.0f, 50.0f)), 2);
}

}

int main() {
  test::suite suite("polyline2");

  // Test Generalize
  suite.test(TEST_CASE(TestGeneralizeAndLength));

  // Test distance of a point to a line segment
  suite.test(TEST_CASE(TestClosestPoint));

  // Test clip
  suite.test(TEST_CASE(TestClip));

  // Test clipped polyline
  suite.test(TEST_CASE(TestClippedPolyline));

  return suite.tear_down();
}
