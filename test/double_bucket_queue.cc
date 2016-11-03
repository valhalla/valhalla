#include "test.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include "config.h"
#include "baldr/double_bucket_queue.h"

using namespace std;
using namespace valhalla::baldr;

namespace {

void TryAddRemove(const std::vector<uint32_t>& costs,
                  const std::vector<uint32_t>& expectedorder) {
  std::vector<float> edgelabels;

  const auto edgecost = [&edgelabels](const uint32_t label) {
    return edgelabels[label];
  };

  uint32_t i = 0;
  DoubleBucketQueue adjlist(0, 10000, 5, edgecost);
  for (auto cost : costs) {
    edgelabels.emplace_back(cost);
    adjlist.add(i, cost);
    i++;
  }
  for (auto expected : expectedorder) {
    uint32_t labelindex = adjlist.pop();
    if (edgelabels[labelindex] != expected) {
      throw runtime_error("TryAddRemove: expected order test failed");
    }
  }
}

void TestAddRemove() {
  std::vector<uint32_t> costs = { 67, 325, 25, 466, 1000, 100005, 758, 167,
            258, 16442, 278, 111111000 };
  std::vector<uint32_t> expectedorder = costs;
  std::sort(expectedorder.begin(), expectedorder.end());
  TryAddRemove(costs, expectedorder);
}

void TryClear(const std::vector<uint32_t>& costs) {
  uint32_t i = 0;
  std::vector<float> edgelabels;

  const auto edgecost = [&edgelabels](const uint32_t label) {
    return edgelabels[label];
  };
  DoubleBucketQueue adjlist(0, 10000, 50, edgecost);
  for (auto cost : costs) {
    edgelabels.emplace_back(cost);
    adjlist.add(i, cost);
    i++;
  }
  adjlist.clear();
  uint32_t idx = adjlist.pop();
  if (idx != kInvalidLabel)
    throw runtime_error("TryClear: failed to return invalid edge index after Clear");
}

void TestClear() {
  std::vector<uint32_t> costs = { 67, 325, 25, 466, 1000, 100005, 758, 167,
            258, 16442, 278, 111111000 };
  TryClear(costs);
}

/**
void TestDecreseCost() {
  std::vector<uint32_t> costs = { 67, 325, 25, 466, 1000, 100005, 758, 167,
            258, 16442, 278, 111111000 };
  std::vector<uint32_t> expectedorder = costs;
  std::sort(costs);
  TryAddRemove(costs, expectedorder);

}
*/

}

int main() {
  test::suite suite("adjacencylist");

  suite.test(TEST_CASE(TestAddRemove));

  suite.test(TEST_CASE(TestClear));

//  suite.test(TEST_CASE(TestDecreaseCost));

  return suite.tear_down();
}
