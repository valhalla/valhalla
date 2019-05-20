#include "baldr/double_bucket_queue.h"
#include "config.h"
#include "midgard/util.h"
#include "test.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

using namespace std;
using namespace valhalla;
using namespace valhalla::baldr;

namespace {

void TryAddRemove(const std::vector<uint32_t>& costs, const std::vector<uint32_t>& expectedorder) {
  std::vector<float> edgelabels;

  const auto edgecost = [&edgelabels](const uint32_t label) { return edgelabels[label]; };

  uint32_t i = 0;
  DoubleBucketQueue adjlist(0, 10000, 5, edgecost);
  for (auto cost : costs) {
    edgelabels.emplace_back(cost);
    adjlist.add(i);
    i++;
  }
  for (auto expected : expectedorder) {
    uint32_t labelindex = adjlist.pop();
    if (edgelabels[labelindex] != expected) {
      throw runtime_error("TryAddRemove: expected order test failed");
    }
  }
}

void TestInvalidConstruction() {
  std::vector<float> edgelabels;
  const auto edgecost = [&edgelabels](const uint32_t label) { return edgelabels[label]; };
  try {
    // Test invalid bucket size
    DoubleBucketQueue adjlist(0, 10000, 0, edgecost);
    throw runtime_error("Invalid bucket size not caught");
  } catch (...) {}
  try {
    // Test invalid range
    DoubleBucketQueue adjlist(0, 0.0f, 1, edgecost);
    throw runtime_error("Invalid cost range not caught");
  } catch (...) {}
}

void TestAddRemove() {
  std::vector<uint32_t> costs = {67,  325, 25,  466,   1000, 100005,
                                 758, 167, 258, 16442, 278,  111111000};
  std::vector<uint32_t> expectedorder = costs;
  std::sort(expectedorder.begin(), expectedorder.end());
  TryAddRemove(costs, expectedorder);
}

void TryClear(const std::vector<uint32_t>& costs) {
  uint32_t i = 0;
  std::vector<float> edgelabels;

  const auto edgecost = [&edgelabels](const uint32_t label) { return edgelabels[label]; };
  DoubleBucketQueue adjlist(0, 10000, 50, edgecost);
  for (auto cost : costs) {
    edgelabels.emplace_back(cost);
    adjlist.add(i);
    i++;
  }
  adjlist.clear();
  uint32_t idx = adjlist.pop();
  if (idx != kInvalidLabel)
    throw runtime_error("TryClear: failed to return invalid edge index after Clear");
}

void TestClear() {
  std::vector<uint32_t> costs = {67,  325, 25,  466,   1000, 100005,
                                 758, 167, 258, 16442, 278,  111111000};
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

void TryRemove(DoubleBucketQueue& dbqueue, size_t num_to_remove, const std::vector<float>& costs) {
  auto previous_cost = -std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < num_to_remove; ++i) {
    const auto top = dbqueue.pop();
    test::assert_bool(top != kInvalidLabel, "TryAddRemove: expected " +
                                                std::to_string(num_to_remove) + " labels to remove");
    const auto cost = costs[top];
    test::assert_bool(previous_cost <= cost, "TryAddRemove: expected order test failed");
    previous_cost = cost;
  }

  {
    const auto top = dbqueue.pop();
    test::assert_bool(top == kInvalidLabel, "Simulation: expect list to be empty");
  }
}

void TrySimulation(DoubleBucketQueue& dbqueue,
                   std::vector<float>& costs,
                   size_t loop_count,
                   size_t expansion_size,
                   size_t max_increment_cost) {
  // Track all label indexes in the dbqueue
  std::unordered_set<uint32_t> addedLabels;

  const uint32_t idx = costs.size();
  costs.push_back(10.f);
  dbqueue.add(idx);
  std::random_device rd;
  std::mt19937 gen(rd());
  for (size_t i = 0; i < loop_count; i++) {
    const auto key = dbqueue.pop();
    if (key == kInvalidLabel) {
      break;
    }

    const auto min_cost = costs[key];
    // Must be the minimal one among the tracked labels
    for (auto k : addedLabels) {
      test::assert_bool(min_cost <= costs[k], "Simulation: minimal cost expected");
    }
    addedLabels.erase(key);

    for (size_t i = 0; i < expansion_size; i++) {
      const auto newcost = std::floor(min_cost + 1 + test::rand01(gen) * max_increment_cost);
      if (i % 2 == 0 && !addedLabels.empty()) {
        // Decrease cost
        const auto idx = *std::next(addedLabels.begin(), test::rand01(gen) * addedLabels.size());
        if (newcost < costs[idx]) {
          dbqueue.decrease(idx, newcost);
          costs[idx] = newcost;
          // test::assert_bool(dbqueue.cost(idx) == newcost, "failed to decrease cost");
        } else {
          // // Assert that it must fail to decrease since costs[idx] <= newcost
          // test::assert_throw<std::runtime_error>([&dbqueue, idx, newcost](){
          //     dbqueue.decrease(idx, newcost);
          //   }, "decreasing a non-less cost must fail");
        }
      } else {
        // Add new label
        const uint32_t idx = costs.size();
        costs.push_back(newcost);
        dbqueue.add(idx);
        addedLabels.insert(idx);
      }
    }
  }

  TryRemove(dbqueue, addedLabels.size(), costs);
}

void TestSimulation() {
  {
    std::vector<float> costs;
    DoubleBucketQueue dbqueue1(0, 1, 100000, [&costs](const uint32_t label) { return costs[label]; });
    TrySimulation(dbqueue1, costs, 1000, 10, 1000);
  }

  {
    std::vector<float> costs;
    DoubleBucketQueue dbqueue2(0, 1, 100000, [&costs](const uint32_t label) { return costs[label]; });
    TrySimulation(dbqueue2, costs, 222, 40, 100);
  }

  {
    std::vector<float> costs;
    DoubleBucketQueue dbqueue3(0, 1, 100000, [&costs](const uint32_t label) { return costs[label]; });
    TrySimulation(dbqueue3, costs, 333, 60, 100);
  }

  {
    std::vector<float> costs;
    DoubleBucketQueue dbqueue4(0, 1, 1000, [&costs](const uint32_t label) { return costs[label]; });
    TrySimulation(dbqueue4, costs, 333, 60, 100);
  }
}

} // namespace

int main() {
  test::suite suite("double_bucket_queue");

  suite.test(TEST_CASE(TestInvalidConstruction));

  suite.test(TEST_CASE(TestAddRemove));

  suite.test(TEST_CASE(TestClear));

  //  suite.test(TEST_CASE(TestDecreaseCost));

  suite.test(TEST_CASE(TestSimulation));

  return suite.tear_down();
}
