#include "test.h"

#include <vector>
#include "config.h"
#include "thor/edgelabel.h"
#include "thor/adjacencylist.h"

using namespace std;
using namespace valhalla::thor;

namespace {

void TryAddRemove(const std::vector<unsigned int>& costs,
                  const std::vector<unsigned int>& expectedorder) {
  AdjacencyList adjlist(0, 10000, 5);
  for (auto cost : costs) {
    EdgeLabel* label = new EdgeLabel();
    label->SetSortCost(cost);
    adjlist.Add(label);
  }
  for (auto expected : expectedorder) {
    EdgeLabel* label = adjlist.Remove();
    if (label->sortcost() != expected)
      throw runtime_error("TryAddRemove: expected order test failed");
    delete label;
  }
}

void TestAddRemove() {
  std::vector<unsigned int> costs = { 67, 325, 25, 466, 1000, 100005, 758, 167,
            258, 16442, 278, 111111000 };
  std::vector<unsigned int> expectedorder = costs;
  std::sort(expectedorder.begin(), expectedorder.end());
  TryAddRemove(costs, expectedorder);
}

void TryClear(const std::vector<unsigned int>& costs) {
  AdjacencyList adjlist(0, 10000, 50);
  for (auto cost : costs) {
    EdgeLabel* label = new EdgeLabel();
    label->SetSortCost(cost);
    adjlist.Add(label);
  }
  adjlist.Clear();
  EdgeLabel* label = adjlist.Remove();
  if (label != nullptr)
    throw runtime_error("TryClear: failed to return nullptr afer Clear");
}

void TestClear() {
  std::vector<unsigned int> costs = { 67, 325, 25, 466, 1000, 100005, 758, 167,
            258, 16442, 278, 111111000 };
  TryClear(costs);
}

/**
void TryMinLng(const AABBLL& a, float res) {
  if (fabs(a.minlng() - res) > kEpsilon)
    throw runtime_error("TestMinLng test failed");
}

void TestDecreseCost() {
  std::vector<unsigned int> costs = { 67, 325, 25, 466, 1000, 100005, 758, 167,
            258, 16442, 278, 111111000 };
  std::vector<unsigned int> expectedorder = costs;
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
