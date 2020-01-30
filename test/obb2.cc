#include "midgard/obb2.h"

#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

TEST(OBB2, TestOverlap) {
  // Test cases in x,y
  OBB2<Point2> a({1, 1}, {2, -1}, {6, 1}, {5, 3});
  OBB2<Point2> b({-1, 3}, {-2, 2}, {-1, 1}, {0, 2});
  OBB2<Point2> c({1, 4}, {0, 3}, {4, -1}, {5, 0});
  EXPECT_FALSE(a.Overlap(b));
  EXPECT_TRUE(a.Overlap(c));
  EXPECT_FALSE(b.Overlap(c));

  // Same test cases with lat,lng
  OBB2<PointLL> d({1, 1}, {2, -1}, {6, 1}, {5, 3});
  OBB2<PointLL> e({-1, 3}, {-2, 2}, {-1, 1}, {0, 2});
  OBB2<PointLL> f({1, 4}, {0, 3}, {4, -1}, {5, 0});
  EXPECT_FALSE(d.Overlap(e));
  EXPECT_TRUE(d.Overlap(f));
  EXPECT_FALSE(e.Overlap(f));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}