#include "thor/triplegbuilder.h"

#include <gtest/gtest.h>

using namespace valhalla::midgard;
using namespace valhalla::thor;

namespace {
// Fixes case in https://github.com/valhalla/valhalla/issues/2201#issuecomment-582656499
TEST(TrimShape, test_case_asan) {
  float start = 42.2698097;
  float end = 47;
  PointLL start_vertex{8.54709148, 47.3651924};
  PointLL end_vertex{8.54711914, 47.3651543};
  std::vector<PointLL> shape = {
      PointLL{8.5468483, 47.3655319},
      PointLL{8.54691314, 47.365448},
      PointLL{8.54711914, 47.3651543},
  };
  TrimShape(start, start_vertex, end, end_vertex, shape);
  ASSERT_EQ(shape.size(), 2);
  ASSERT_FLOAT_EQ(shape.at(0).lat(), 47.3652);
  ASSERT_FLOAT_EQ(shape.at(0).lng(), 8.54709);
  ASSERT_FLOAT_EQ(shape.at(1).lat(), 47.365154);
  ASSERT_FLOAT_EQ(shape.at(1).lng(), 8.54712);
}

// Check for empty shape
TEST(TrimShape, test_case_empty) {
  PointLL start_vertex;
  PointLL end_vertex;
  std::vector<PointLL> shape;
  TrimShape(0, start_vertex, 0, end_vertex, shape);
  ASSERT_EQ(shape.size(), 0);
}
} // anonymous namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
