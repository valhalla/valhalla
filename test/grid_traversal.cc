#include <vector>

#include "midgard/linesegment2.h"
#include "midgard/point2.h"

#include "meili/grid_traversal.h"

#include "test.h"

namespace {

using namespace valhalla;

std::string result_to_str(const std::vector<std::pair<int, int>>& result) {
  std::string result_str;
  result_str = "(";
  for (const auto& p : result) {
    result_str += std::to_string(p.first);
    result_str += " ";
    result_str += std::to_string(p.second);
    result_str += ", ";
  }
  if (1 < result_str.size()) {
    result_str.erase(result_str.size() - 2);
  }
  result_str += ")";
  return result_str;
}

void assert_equal_squares(const std::vector<std::pair<int, int>>& got,
                          const std::vector<std::pair<int, int>>& expected,
                          const std::string& /*msg*/) {

  ASSERT_EQ(got, expected) << "got: " << result_to_str(got)
                           << " expected: " << result_to_str(expected);

  ASSERT_EQ(got.size(), expected.size());
  for (decltype(got.size()) i = 0; i < got.size(); i++) {
    ASSERT_EQ(got[i].first, expected[i].first);
    ASSERT_EQ(got[i].second, expected[i].second);
  }
}

TEST(GridTraversal, TestGridTraversal) {
  meili::GridTraversal<midgard::Point2> grid(0.1, 0.1, 0.3, 0.3, 3, 3);
  assert_equal_squares(grid.Traverse({0, 0}, {0.9, 0.9}),
                       std::vector<std::pair<int, int>>{{0, 0}, {1, 1}, {2, 2}},
                       "It should intersect squares in diagonal");

  assert_equal_squares(grid.Traverse({0, 0}, {0.5, 0.5}),
                       std::vector<std::pair<int, int>>{{0, 0}, {1, 1}},
                       "It should intersect squares in diagonal");

  assert_equal_squares(grid.Traverse({0.2, 0.25}, {0.89, 0.32}),
                       std::vector<std::pair<int, int>>{{0, 0}, {1, 0}, {2, 0}},
                       "It should intersect bottom squares");

  assert_equal_squares(grid.Traverse({0.2, 0.25}, {0.34, 0.83}),
                       std::vector<std::pair<int, int>>{{0, 0}, {0, 1}, {0, 2}},
                       "It should intersect leftmost squares");

  assert_equal_squares(
      grid.Traverse({0.3, 0.25}, {0.98, 0.83}),
      std::vector<std::pair<int, int>>{{0, 0}, {1, 0}, {1, 1}, {2, 1}, {2, 2}},
      "It should intersect squares from SQUARE(0 0) to SQUARE(2 2) in the lower side way");

  assert_equal_squares(grid.Traverse({-1, 0.1}, {2, 0.1}),
                       std::vector<std::pair<int, int>>{{0, 0}, {1, 0}, {2, 0}},
                       "It should intersect bottom grid line");

  assert_equal_squares(grid.Traverse({0.1, -1}, {0.1, 2}),
                       std::vector<std::pair<int, int>>{{0, 0}, {0, 1}, {0, 2}},
                       "It should intersect leftmost grid line");

  assert_equal_squares(grid.Traverse({0.1, 0.1}, {0.1, 0.1}),
                       std::vector<std::pair<int, int>>{{0, 0}},
                       "Single point at left bottom corner should intersect SQUARE(0 0)");

  assert_equal_squares(grid.Traverse({0.5, 0.5}, {0.5, 0.5}),
                       std::vector<std::pair<int, int>>{{1, 1}},
                       "Single point should intersect SQUARE(1 1)");

  assert_equal_squares(grid.Traverse({0, 0}, {0, 0}), std::vector<std::pair<int, int>>{},
                       "Single point outside grid should not intersect");

  assert_equal_squares(grid.Traverse({0, 0}, {0.1, 0.2}), std::vector<std::pair<int, int>>{{0, 0}},
                       "It should intersect SQUARE(0 0)");
  assert_equal_squares(grid.Traverse({0, 0}, {0.2, 0.1}), std::vector<std::pair<int, int>>{{0, 0}},
                       "It should intersect SQUARE(0 0)");

  assert_equal_squares(grid.Traverse({2, 1}, {0.5, 0.5}),
                       std::vector<std::pair<int, int>>{{2, 1}, {1, 1}},
                       "It should partially intersect from right side");

  assert_equal_squares(grid.Traverse({0, 0}, {0.07, 0.08}), std::vector<std::pair<int, int>>{},
                       "A line segment completely outside the grid should not intersect");

  // Test a grid that can be precisely expressed by floating-point
  // number
  meili::GridTraversal<midgard::Point2> perfect_grid(0.5, 0.5, 0.25, 0.25, 3, 3);

  assert_equal_squares(perfect_grid.Traverse({1.25, 1.25}, {1.25, 1.25}),
                       std::vector<std::pair<int, int>>{},
                       "Sit on the rightmost grid line but it is actually outside the grid");

  assert_equal_squares(perfect_grid.Traverse({1, 1}, {1, 1}),
                       std::vector<std::pair<int, int>>{{2, 2}}, "Sit on the corner of SQUARE(2 2)");

  assert_equal_squares(perfect_grid.Traverse({1, 0}, {1, 2}),
                       std::vector<std::pair<int, int>>{{2, 0}, {2, 1}, {2, 2}},
                       "Parallel to vertical grid lines");
  assert_equal_squares(perfect_grid.Traverse({1.05, 0}, {1.05, 2}),
                       std::vector<std::pair<int, int>>{{2, 0}, {2, 1}, {2, 2}},
                       "Parallel to vertical grid lines");
  assert_equal_squares(perfect_grid.Traverse({1.25, 0}, {1.25, 2}),
                       std::vector<std::pair<int, int>>{}, "Parallel to vertical grid lines");

  assert_equal_squares(perfect_grid.Traverse({0, 1}, {2, 1}),
                       std::vector<std::pair<int, int>>{{0, 2}, {1, 2}, {2, 2}},
                       "Parallel to horizontal grid lines");
  assert_equal_squares(perfect_grid.Traverse({0, 1.05}, {2, 1.05}),
                       std::vector<std::pair<int, int>>{{0, 2}, {1, 2}, {2, 2}},
                       "Parallel to horizontal grid lines");
  assert_equal_squares(perfect_grid.Traverse({0, 1.25}, {2, 1.25}),
                       std::vector<std::pair<int, int>>{}, "Parallel to horizontal grid lines");

  assert_equal_squares(perfect_grid.Traverse({1.25, 1}, {1.25, 1}),
                       std::vector<std::pair<int, int>>{}, "Single point sit on rightmost grid line");

  // Test a grid from real world example
  meili::GridTraversal<midgard::Point2> real_grid(127.0, 36.75, 0.0005, 0.0005, 500, 500);

  assert_equal_squares(real_grid.Traverse({127.176254, 36.962307}, {127.177925, 36.9602432}),
                       std::vector<std::pair<int, int>>{{352, 424},
                                                        {353, 424},
                                                        {353, 423},
                                                        {353, 422},
                                                        {354, 422},
                                                        {354, 421},
                                                        {355, 421},
                                                        {355, 420}},
                       "Real world example of segment intersecting grid");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}