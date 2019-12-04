#include "midgard/polyline2.h"
#include <cstdint>

#include "test.h"

#include <algorithm>
#include <iostream>
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

  if ((pl2.pts().at(0) != Point2(25.0f, 25.0f)) || (pl2.pts().at(1) != Point2(50.0f, 100.0f))) {
    throw runtime_error("Generalize #2 test failed.");
  }

  if (!(fabs(pl2.Length() - res) > kEpsilon))
    throw runtime_error("Length test failed.");
}

void TestGeneralizeAndLength() {
  std::vector<Point2> pts = {Point2(25.0f, 25.0f), Point2(50.0f, 50.0f), Point2(25.0f, 75.0f),
                             Point2(50.0f, 100.0f)};
  Polyline2<Point2> pl(pts);
  TryGeneralizeAndLength(pl, 100.0f, 79.0569f);
}

void TestGeneralizeSimplification() {
  Polyline2<Point2> line{{{17, 0}, {17, 1}, {17, 2}, {17, 3}, {17, 4}, {17, 5}}};

  line.Generalize(1, std::unordered_set<size_t>{2, 4});
  if (!(line == Polyline2<Point2>{{{17, 0}, {17, 2}, {17, 4}, {17, 5}}}))
    throw std::logic_error("Should have removed all but the first, last and marked points");

  line.Generalize(1, std::unordered_set<size_t>{2});
  if (!(line == Polyline2<Point2>{{{17, 0}, {17, 4}, {17, 5}}}))
    throw std::logic_error("Should have removed all but the first, last and marked points");

  Polyline2<PointLL> line1{
      {{-76.58489, 40.31402}, {-76.58496, 40.31411}, {-76.58506, 40.31416}, {-76.58521, 40.31414},
       {-76.58586, 40.31383}, {-76.58596, 40.31379}, {-76.58658, 40.31349}, {-76.58723, 40.31319},
       {-76.58787, 40.31286}, {-76.58842, 40.31362}, {-76.58865, 40.31427}, {-76.58895, 40.31514},
       {-76.58921, 40.31579}, {-76.58923, 40.31582}, {-76.58924, 40.31586}, {-76.58924, 40.31589},
       {-76.58994, 40.31779}, {-76.59043, 40.31924}, {-76.59077, 40.32019}, {-76.5922, 40.32265},
       {-76.5927, 40.32239},  {-76.59319, 40.32216}, {-76.59346, 40.32202}, {-76.59371, 40.32189},
       {-76.59594, 40.32079}, {-76.59701, 40.32033}, {-76.59809, 40.31994}, {-76.59971, 40.31932},
       {-76.60005, 40.31922}, {-76.60037, 40.31917}, {-76.6011, 40.31905}}};
  line1.Generalize(10, std::unordered_set<size_t>{3, 7, 11, 15, 19, 23, 27});
  std::vector<PointLL> remaining = {{-76.58489, 40.31402}, {-76.58521, 40.31414},
                                    {-76.58723, 40.31319}, {-76.58895, 40.31514},
                                    {-76.58924, 40.31589}, {-76.5922, 40.32265},
                                    {-76.59371, 40.32189}, {-76.59971, 40.31932},
                                    {-76.6011, 40.31905}};
  for (const auto& p : remaining) {
    if (std::find(line1.pts().cbegin(), line1.pts().cend(), p) == line1.pts().cend())
      throw std::logic_error("Should still have at least the first, last and marked points");
  }

  Polyline2<Point2> line2{{{-79.3837, 43.6481},
                           {-79.3839, 43.6485},
                           {-79.3839, 43.6485},
                           {-79.3839, 43.6486},
                           {-79.3842, 43.6491},
                           {-79.3842, 43.6492},
                           {-79.3842, 43.6492},
                           {-79.3841, 43.6493},
                           {-79.3841, 43.6493},
                           {-79.384, 43.6493},
                           {-79.3841, 43.6496},
                           {-79.384, 43.6496},
                           {-79.384, 43.6496},
                           {-79.3839, 43.6496},
                           {-79.3839, 43.6496},
                           {-79.3838, 43.6496}}};
  line2.Generalize(2.6f, std::unordered_set<size_t>{15, 14, 13, 0, 10, 6, 9});

  if (!(line2 == Polyline2<Point2>{{{-79.3837, 43.6481},
                                    {-79.3842, 43.6492},
                                    {-79.384, 43.6493},
                                    {-79.3841, 43.6496},
                                    {-79.3839, 43.6496},
                                    {-79.3839, 43.6496},
                                    {-79.3838, 43.6496}}}))
    throw std::logic_error("Wrong points removed.");

  Polyline2<Point2> line3{{{-79.3837, 43.6481},
                           {-79.3839, 43.6485},
                           {-79.3839, 43.6485},
                           {-79.3839, 43.6486},
                           {-79.3842, 43.6491},
                           {-79.3842, 43.6492},
                           {-79.3842, 43.6492},
                           {-79.3841, 43.6493},
                           {-79.3841, 43.6493},
                           {-79.384, 43.6493},
                           {-79.3841, 43.6496},
                           {-79.384, 43.6496},
                           {-79.384, 43.6496},
                           {-79.3839, 43.6496},
                           {-79.3839, 43.6496},
                           {-79.3838, 43.6496}}};
  line3.Generalize(0.f);

  if (line3.pts().size() != 16)
    throw std::logic_error("No points should be removed.");
}

void TryClosestPoint(const Polyline2<Point2>& pl, const Point2& a, const Point2& b) {

  auto result = pl.ClosestPoint(a);

  if (std::get<0>(result) != b)
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

  if ((pl.pts().at(0) != Point2(25.0f, 25.0f)) || (pl.pts().at(1) != Point2(50.0f, 50.0f))) {
    throw runtime_error("Clip test failed: clipped points not correct");
  }
}

void TryClipOutside(Polyline2<Point2>& pl, const AABB2<Point2>& a) {
  uint32_t x = pl.Clip(a);
  if (x != 0) {
    throw runtime_error("Clip test failed: all vertices outside so count should be 0");
  }
}

void TestClip() {
  std::vector<Point2> pts = {Point2(25.0f, 25.0f), Point2(50.0f, 50.0f), Point2(25.0f, 75.0f),
                             Point2(50.0f, 100.0f)};
  Polyline2<Point2> pl(pts);
  TryClip(pl, AABB2<Point2>(Point2(0.0f, 0.0f), Point2(75.0f, 50.0f)), 2);

  // Test with vertices on edges
  Polyline2<Point2> pl2(pts);
  TryClip(pl2, AABB2<Point2>(Point2(25.0f, 25.0f), Point2(50.0f, 100.0f)), 4);

  // Test Clip with all vertices above top of AABB
  Polyline2<Point2> pl3(pts);
  TryClipOutside(pl3, AABB2<Point2>(Point2(0.0f, 0.0f), Point2(50.0f, 20.0f)));

  // Test Clip with all vertices left of AABB
  Polyline2<Point2> pl4(pts);
  TryClipOutside(pl4, AABB2<Point2>(Point2(50.0f, 25.0f), Point2(100.0f, 100.0f)));

  // Test Clip with all vertices right of AABB
  Polyline2<Point2> pl5(pts);
  TryClipOutside(pl5, AABB2<Point2>(Point2(0.0f, 25.0f), Point2(10.0f, 100.0f)));

  // Test Clip with all vertices below bottom of AABB
  Polyline2<Point2> pl6(pts);
  TryClipOutside(pl6, AABB2<Point2>(Point2(25.0f, 100.0f), Point2(50.0f, 200.0f)));
}

void TryClippedPolyline(Polyline2<Point2>& pl, const AABB2<Point2>& a, const uint32_t exp) {
  Polyline2<Point2> pl2 = pl.ClippedPolyline(a);
  uint32_t x = pl2.pts().size();
  if (x != 2)
    throw runtime_error("ClippedPolyline test failed: count not correct");

  if ((pl2.pts().at(0) != Point2(25.0f, 25.0f)) || (pl2.pts().at(1) != Point2(50.0f, 50.0f))) {
    throw runtime_error("ClippedPolyline test failed: clipped points not correct");
  }
}

void TestClippedPolyline() {
  std::vector<Point2> pts = {Point2(25.0f, 25.0f), Point2(50.0f, 50.0f), Point2(25.0f, 75.0f),
                             Point2(50.0f, 100.0f)};
  Polyline2<Point2> pl(pts);
  TryClippedPolyline(pl, AABB2<Point2>(Point2(0.0f, 0.0f), Point2(75.0f, 50.0f)), 2);
}

} // namespace

int main() {
  test::suite suite("polyline2");

  // Test Generalize
  suite.test(TEST_CASE(TestGeneralizeAndLength));

  // Test Generalize for which points are removed
  suite.test(TEST_CASE(TestGeneralizeSimplification));

  // Test distance of a point to a line segment
  suite.test(TEST_CASE(TestClosestPoint));

  // Test clip
  suite.test(TEST_CASE(TestClip));

  // Test clipped polyline
  suite.test(TEST_CASE(TestClippedPolyline));

  return suite.tear_down();
}
