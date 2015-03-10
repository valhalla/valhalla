#include "valhalla/midgard/obb2.h"

#include "test.h"

#include <vector>

#include "valhalla/midgard/point2.h"
#include "valhalla/midgard/vector2.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

void TryOverlap(const OBB2& a, const OBB2& b, const bool expected) {
  if (a.Overlap(b) != expected) {
    throw runtime_error("OBB Overlap test failed: expected: " +
                        std::to_string(expected));
  }
}

void TestOverlap() {
  OBB2 a({1,1}, {2,-1}, {6,1}, {5,3});
  OBB2 b({-1,3}, {-2,2}, {-1,1}, {0,2});
  OBB2 c({1,4}, {0,3}, {4,-1}, {5,0});

  TryOverlap(a, b, false);
  TryOverlap(a, c, true);
  TryOverlap(b, c, false);
}

}

int main() {
  test::suite suite("obb2");

  // Tests if another bounding box overlaps this bounding box
  suite.test(TEST_CASE(TestOverlap));

  return suite.tear_down();
}
