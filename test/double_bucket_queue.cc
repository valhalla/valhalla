#include "baldr/double_bucket_queue.h"
#include "config.h"
#include "midgard/util.h"
#include "sif/edgelabel.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "test.h"

using namespace std;
using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::sif;

constexpr size_t kEdgeLabelExpectedSize = 40;
constexpr size_t kPathEdgeLabelExpectedSize = 48;
constexpr size_t kBDEdgeLabelExpectedSize = 64;
constexpr size_t kMMEdgeLabelExpectedSize = 72;

namespace {

struct simple_label {
  float c;
  float sortcost() const {
    return c;
  }
};

void TryAddRemove(const std::vector<uint32_t>& costs, const std::vector<uint32_t>& expectedorder) {
  std::vector<simple_label> edgelabels;

  uint32_t i = 0;
  DoubleBucketQueue<simple_label> adjlist(0, 10000, 1, &edgelabels);
  for (auto cost : costs) {
    edgelabels.emplace_back(simple_label{static_cast<float>(cost)});
    adjlist.add(i);
    ++i;
  }
  for (auto expected : expectedorder) {
    uint32_t labelindex = adjlist.pop();
    simple_label edgelabel{};
    if (labelindex != baldr::kInvalidLabel) {
      edgelabel = edgelabels[labelindex];
    }
    // Do the same transform that's done in `edgecost()`
    EXPECT_EQ(edgelabel.sortcost(), (float)expected) << "TryAddRemove: expected order test failed";
  }
}

TEST(DoubleBucketQueue, TestInvalidConstruction) {
  std::vector<simple_label> edgelabels;
  EXPECT_THROW(DoubleBucketQueue<simple_label> adjlist(0, 10000, 0, &edgelabels), runtime_error)
      << "Invalid bucket size not caught";
  EXPECT_THROW(DoubleBucketQueue<simple_label> adjlist(0, 0.0f, 1, &edgelabels), runtime_error)
      << "Invalid cost range not caught";
}

TEST(DoubleBucketQueue, TestAddRemove) {
  std::vector<uint32_t> costs = {67,  325, 25,  466,   1000, 100005,
                                 758, 167, 258, 16442, 278,  111111000};
  std::vector<uint32_t> expectedorder = costs;
  std::sort(expectedorder.begin(), expectedorder.end());
  TryAddRemove(costs, expectedorder);
}

void TryClear(const std::vector<uint32_t>& costs) {
  uint32_t i = 0;
  std::vector<simple_label> edgelabels;
  DoubleBucketQueue<simple_label> adjlist(0, 10000, 50, &edgelabels);
  for (auto cost : costs) {
    edgelabels.emplace_back(simple_label{static_cast<float>(cost)});
    adjlist.add(i);
    i++;
  }
  adjlist.clear();
  uint32_t idx = adjlist.pop();
  EXPECT_EQ(idx, baldr::kInvalidLabel) << "TryClear: failed to return invalid edge index after Clear";
}

TEST(DoubleBucketQueue, TestClear) {
  std::vector<uint32_t> costs = {67,  325, 25,  466,   1000, 100005,
                                 758, 167, 258, 16442, 278,  111111000};
  TryClear(costs);
}

TEST(DoubleBucketQueue, RC4FloatPrecisionErrors) {
  // Tests what happens when the internal floats in DoubleBucketQueue loses
  // precision

  std::vector<uint32_t> costs = {1320209856};
  std::vector<uint32_t> expectedorder = costs;
  std::sort(expectedorder.begin(), expectedorder.end());
  TryAddRemove(costs, expectedorder);
}

/**
   void TestDecreaseCost() {
   std::vector<uint32_t> costs = { 67, 325, 25, 466, 1000, 100005, 758, 167,
   258, 16442, 278, 111111000 };
   std::vector<uint32_t> expectedorder = costs;
   std::sort(costs);
   TryAddRemove(costs, expectedorder);

   }
*/

void TryRemove(DoubleBucketQueue<simple_label>& dbqueue,
               size_t num_to_remove,
               const std::vector<simple_label>& costs) {
  auto previous_cost = -std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < num_to_remove; ++i) {
    const auto top = dbqueue.pop();
    EXPECT_NE(top, baldr::kInvalidLabel)
        << "TryAddRemove: expected " + std::to_string(num_to_remove) + " labels to remove";
    const auto cost = costs[top].sortcost();
    EXPECT_LE(previous_cost, cost) << "TryAddRemove: expected order test failed";
    previous_cost = cost;
  }

  {
    const auto top = dbqueue.pop();
    EXPECT_EQ(top, baldr::kInvalidLabel) << "Simulation: expect list to be empty";
  }
}

void TrySimulation(DoubleBucketQueue<simple_label>& dbqueue,
                   std::vector<simple_label>& costs,
                   size_t loop_count,
                   size_t expansion_size,
                   size_t max_increment_cost) {
  // Track all label indexes in the dbqueue
  std::unordered_set<uint32_t> addedLabels;

  const uint32_t idx = costs.size();
  costs.push_back({10.f});
  dbqueue.add(idx);
  std::random_device rd;
  std::mt19937 gen(rd());
  for (size_t i = 0; i < loop_count; i++) {
    const auto key = dbqueue.pop();
    if (key == baldr::kInvalidLabel) {
      break;
    }

    const auto min_cost = costs[key].sortcost();
    // Must be the minimal one among the tracked labels
    for (auto k : addedLabels) {
      EXPECT_LE(min_cost, costs[k].sortcost()) << "Simulation: minimal cost expected";
    }
    addedLabels.erase(key);

    for (size_t i = 0; i < expansion_size; i++) {
      const auto newcost = std::floor(min_cost + 1 + test::rand01(gen) * max_increment_cost);
      if (i % 2 == 0 && !addedLabels.empty()) {
        // Decrease cost
        const auto idx = *std::next(addedLabels.begin(), test::rand01(gen) * (addedLabels.size()));
        if (newcost < costs[idx].sortcost()) {
          dbqueue.decrease(idx, newcost);
          costs[idx] = {newcost};
          // todo: why commented??
          // EXPECT_EQ(dbqueue.cost(idx), newcost) << "failed to decrease cost";
        } else {
          // Assert that it must fail to decrease since costs[idx] <= newcost
          // todo: why commented??
          // EXPECT_THROW(dbqueue.decrease(idx, newcost), std::runtime_error);
          // test::assert_throw<std::runtime_error>([&dbqueue, idx, newcost](){
          //     dbqueue.decrease(idx, newcost);
          //   }, "decreasing a non-less cost must fail");
        }
      } else {
        // Add new label
        const uint32_t idx = costs.size();
        costs.push_back({newcost});
        dbqueue.add(idx);
        addedLabels.insert(idx);
      }
    }
  }

  TryRemove(dbqueue, addedLabels.size(), costs);
}

TEST(DoubleBucketQueue, TestSimulation) {
  {
    std::vector<simple_label> costs;
    DoubleBucketQueue<simple_label> dbqueue1(0, 1, 100000, &costs);
    TrySimulation(dbqueue1, costs, 1000, 10, 1000);
  }

  {
    std::vector<simple_label> costs;
    DoubleBucketQueue<simple_label> dbqueue2(0, 1, 100000, &costs);
    TrySimulation(dbqueue2, costs, 222, 40, 100);
  }

  {
    std::vector<simple_label> costs;
    DoubleBucketQueue<simple_label> dbqueue3(0, 1, 100000, &costs);
    TrySimulation(dbqueue3, costs, 333, 60, 100);
  }

  {
    std::vector<simple_label> costs;
    DoubleBucketQueue<simple_label> dbqueue4(0, 1, 1000, &costs);
    TrySimulation(dbqueue4, costs, 333, 60, 100);
  }
}

// Test EdgeLabel size
TEST(EdgeLabel, test_sizeof) {
  EXPECT_EQ(sizeof(EdgeLabel), kEdgeLabelExpectedSize);
  EXPECT_EQ(sizeof(PathEdgeLabel), kPathEdgeLabelExpectedSize);
  EXPECT_EQ(sizeof(BDEdgeLabel), kBDEdgeLabelExpectedSize);
  EXPECT_EQ(sizeof(MMEdgeLabel), kMMEdgeLabelExpectedSize);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
