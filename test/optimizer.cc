#include "thor/optimizer.h"
#include "config.h"
#include "test.h"
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

using namespace std;
using namespace valhalla::thor;

namespace {

void TryOptimizer(const uint32_t nlocs,
                  const std::vector<float>& costs,
                  const std::vector<uint32_t>& expected_order) {
  Optimizer optimizer;
  optimizer.Seed(111111);
  auto order = optimizer.Solve(nlocs, costs);
  if (order != expected_order) {
    throw runtime_error("TryOptimizer: expected order failed");
  }
}

void TestOptimizer() {
  std::vector<float> costs = {0,    3036, 707,  956,  318,  1934, 355,  1170, 1286, 3171, 2133,
                              2978, 0,    2664, 3613, 3102, 2011, 3139, 3846, 1764, 2050, 1143,
                              638,  2638, 0,    1295, 763,  1536, 800,  1528, 888,  2773, 1735,
                              940,  3457, 1281, 0,    582,  2450, 630,  655,  1796, 3681, 2643,
                              357,  3037, 708,  637,  0,    1935, 47,   851,  1286, 3171, 2133,
                              1839, 2004, 1525, 2480, 1963, 0,    2000, 2713, 690,  2578, 1100,
                              387,  3066, 737,  715,  77,   1964, 0,    928,  1316, 3201, 2163,
                              1129, 3803, 1537, 682,  769,  2707, 819,  0,    2052, 3230, 2899,
                              1214, 1750, 900,  1849, 1338, 634,  1375, 2082, 0,    1907, 846,
                              3128, 2036, 2814, 3763, 3252, 2549, 3290, 3228, 1914, 0,    2010,
                              2068, 1133, 1754, 2704, 2193, 1102, 2230, 2937, 854,  2000, 0};
  std::vector<uint32_t> expected_order = {0, 3, 7, 4, 6, 2, 8, 5, 9, 1, 10};
  TryOptimizer(11, costs, expected_order);
}

} // namespace

int main() {
  test::suite suite("optimizer");

  suite.test(TEST_CASE(TestOptimizer));

  return suite.tear_down();
}
