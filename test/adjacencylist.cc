#include "test.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include "config.h"
#include <valhalla/sif/edgelabel.h>
#include "thor/adjacencylist.h"

using namespace std;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

void TryAddRemove(const std::vector<uint32_t>& costs,
                  const std::vector<uint32_t>& expectedorder) {
  uint32_t i = 0;
  std::vector<EdgeLabel> edgelabels;
  AdjacencyList adjlist(0, 10000, 5);
  for (auto cost : costs) {
    EdgeLabel label;
    label.SetSortCost(cost);
    edgelabels.emplace_back(label);
    adjlist.Add(i, cost);
    i++;
  }
  for (auto expected : expectedorder) {
    uint32_t labelindex = adjlist.Remove(edgelabels);
    if (edgelabels[labelindex].sortcost() != expected) {
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
  std::vector<EdgeLabel> edgelabels;
  AdjacencyList adjlist(0, 10000, 50);
  for (auto cost : costs) {
    EdgeLabel label;
    label.SetSortCost(cost);
    edgelabels.emplace_back(label);
    adjlist.Add(i, cost);
    i++;
  }
  adjlist.Clear();
  uint32_t idx = adjlist.Remove(edgelabels);
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
