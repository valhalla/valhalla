#include <iostream>
#include <tuple>
#include <vector>

#include "midgard/linesegment2.h"
#include "midgard/point2.h"

#include "meili/grid_traversal.h"
#include "test.h"

using namespace valhalla;

bool equal_squares(const std::vector<std::pair<int, int>>& s1,
                   const std::vector<std::pair<int, int>>& s2) {
  bool ok = s1.size() == s2.size();
  if (!ok)
    return false;

  for (decltype(s1.size()) i = 0; i < s1.size(); i++) {
    ok = s1[i].first == s2[i].first && s1[i].second == s2[i].second;
    if (!ok)
      return false;
  }

  return true;
}

void assert_equal_squares(const std::vector<std::pair<int, int>>& got,
                          const std::vector<std::pair<int, int>>& expected,
                          const std::string& msg) {
  std::string got_ss;
  got_ss = "(";
  for (const auto& p : got) {
    got_ss += std::to_string(p.first);
    got_ss += " ";
    got_ss += std::to_string(p.second);
    got_ss += ", ";
  }
  if (1 < got_ss.size()) {
    got_ss.erase(got_ss.size() - 2);
  }
  got_ss += ")";

  std::string expected_ss;
  expected_ss = "(";
  for (const auto& p : expected) {
    expected_ss += std::to_string(p.first);
    expected_ss += " ";
    expected_ss += std::to_string(p.second);
    expected_ss += ", ";
  }
  if (1 < expected_ss.size()) {
    expected_ss.erase(expected_ss.size() - 2);
  }
  expected_ss += ")";

  test::assert_bool(equal_squares(got, expected),
                    msg + ". Expected " + expected_ss + ", but got " + got_ss);
}

void TestGridTraversal() {
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
      "It should intersect sqaures from SQUARE(0 0) to SQUARE(2 2) in the lower side way");

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

int main(int argc, char* argv[]) {
  test::suite suite("grid traversal");

  suite.test(TEST_CASE(TestGridTraversal));

  return suite.tear_down();
}
