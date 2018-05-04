#include "midgard/obb2.h"

#include "test.h"

#include <vector>

using namespace std;
using namespace valhalla::midgard;

namespace {

// Test if 2 oriented bounding boxes overlap
void TryOverlap(const OBB2<Point2>& a, const OBB2<Point2>& b, const bool expected) {
  if (a.Overlap(b) != expected) {
    throw runtime_error("OBB<Point2> Overlap test failed: expected: " + std::to_string(expected));
  }
}

// Test if 2 lat,lng oriented bounding boxes overlap
void TryOverlapLL(const OBB2<PointLL>& a, const OBB2<PointLL>& b, const bool expected) {
  if (a.Overlap(b) != expected) {
    throw runtime_error("OBB<PointLL> Overlap test failed: expected: " + std::to_string(expected));
  }
}

void TestOverlap() {
  // Test cases in x,y
  OBB2<Point2> a({1, 1}, {2, -1}, {6, 1}, {5, 3});
  OBB2<Point2> b({-1, 3}, {-2, 2}, {-1, 1}, {0, 2});
  OBB2<Point2> c({1, 4}, {0, 3}, {4, -1}, {5, 0});
  TryOverlap(a, b, false);
  TryOverlap(a, c, true);
  TryOverlap(b, c, false);

  // Same test cases with lat,lng
  OBB2<PointLL> d({1, 1}, {2, -1}, {6, 1}, {5, 3});
  OBB2<PointLL> e({-1, 3}, {-2, 2}, {-1, 1}, {0, 2});
  OBB2<PointLL> f({1, 4}, {0, 3}, {4, -1}, {5, 0});
  TryOverlapLL(d, e, false);
  TryOverlapLL(d, f, true);
  TryOverlapLL(e, f, false);
}

} // namespace

int main() {
  test::suite suite("obb2");

  // Tests if another bounding box overlaps this bounding box
  suite.test(TEST_CASE(TestOverlap));

  return suite.tear_down();
}
