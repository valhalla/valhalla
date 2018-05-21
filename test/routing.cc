#include <cstdint>
// -*- mode: c++ -*-
#include "meili/routing.h"
#include "test.h"

using namespace valhalla;
using namespace valhalla::meili;

void Add(baldr::DoubleBucketQueue& adjlist, const std::vector<float>& costs) {
  uint32_t idx = 0;
  for (const auto cost : costs) {
    adjlist.add(idx++);
  }
}

void TryRemove(baldr::DoubleBucketQueue& adjlist,
               size_t num_to_remove,
               const std::vector<float>& costs) {
  auto previous_cost = -std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < num_to_remove; ++i) {
    const auto top = adjlist.pop();
    const auto cost = costs[top];
    test::assert_bool(previous_cost <= cost, "TryAddRemove: expected order test failed");
    previous_cost = cost;
  }
}

void TestAddRemove() {
  // Test add and remove
  std::vector<float> costs = {67, 325, 25, 466, 1000, 10000, 758, 167, 258, 16442, 278};

  const auto labelcost = [&costs](const uint32_t label) { return costs[label]; };

  baldr::DoubleBucketQueue adjlist(0, 100000, 1, labelcost);

  Add(adjlist, costs);
  TryRemove(adjlist, costs.size(), costs);
  test::assert_bool(adjlist.pop() == baldr::kInvalidLabel, "TestAddRemove: expect list to be empty");

  // Test add randomly and remove
  costs.clear();
  std::random_device rd;
  std::mt19937 gen(rd());
  for (size_t i = 0; i < 10000; i++) {
    const auto cost = std::floor(test::rand01(gen) * 100000);
    costs.push_back(cost);
  }

  baldr::DoubleBucketQueue adjlist2(0, 10000, 1, labelcost);
  Add(adjlist2, costs);
  TryRemove(adjlist2, costs.size(), costs);
  test::assert_bool(adjlist.pop() == baldr::kInvalidLabel, "TestAddRemove: expect list to be empty");

  // Construct a new adjlist
  costs.resize(5);
  costs[0] = 1000;
  costs[1] = 100;
  costs[2] = 10;
  costs[3] = 9;
  costs[4] = 5;
  baldr::DoubleBucketQueue adjlist3(0, 10, 1, labelcost);

  adjlist3.add(0);
  adjlist3.add(1);
  adjlist3.add(2);
  adjlist3.add(3);
  adjlist3.add(4);

  // Decrease cost of label 3 to 3 - pop the lowest cost element - it should be label 3
  adjlist3.decrease(3, 3);
  costs[3] = 3;
  uint32_t lab = adjlist3.pop();
  test::assert_bool(lab == 3,
                    "TestAddRemove: After decrease the lowest cost label must be label 3, top = " +
                        std::to_string(lab));
}

void TrySimulation(size_t loop_count, size_t expansion_size, size_t max_increment_cost) {
  std::vector<float> costs;
  // Track all label indexes in the adjlist
  std::unordered_set<uint32_t> track;

  const auto labelcost = [&costs](const uint32_t label) { return costs[label]; };

  baldr::DoubleBucketQueue adjlist(0, 100000, 1, labelcost);

  const uint32_t idx = costs.size();
  costs.push_back(10.f);
  adjlist.add(idx);
  track.insert(idx);

  std::random_device rd;
  std::mt19937 gen(rd());
  for (size_t i = 0; i < loop_count; i++) {
    const auto key = adjlist.pop();
    const auto min_cost = costs[key];
    // Must be the minimal one among the tracked labels
    for (auto k : track) {
      test::assert_bool(min_cost <= costs[k], "Simulation: minimal cost expected");
    }
    track.erase(key);

    for (size_t i = 0; i < expansion_size; i++) {
      const auto newcost = std::floor(min_cost + 1 + test::rand01(gen) * max_increment_cost);
      if (i % 2 == 0 && !track.empty()) {
        // Decrease cost
        const auto idx = *std::next(track.begin(), test::rand01(gen) * track.size());
        if (newcost < costs[idx]) {
          adjlist.decrease(idx, newcost);
          costs[idx] = newcost;
          test::assert_bool(labelcost(idx) == newcost, "failed to decrease cost");
        } /*else {
          // Assert that it must fail to decrease since costs[idx] <= newcost
          test::assert_throw<std::runtime_error>([&adjlist, idx, newcost](){
              adjlist.decrease(idx, newcost);
            }, "decreasing a non-less cost must fail");
        } */
      } else {
        // Add new label
        const uint32_t idx = costs.size();
        costs.push_back(newcost);
        adjlist.add(idx);
        track.insert(idx);
      }
    }
  }

  TryRemove(adjlist, track.size(), costs);
  test::assert_bool(adjlist.pop() == baldr::kInvalidLabel, "Simulation: expect list to be empty");
}

void TestSimulation() {
  TrySimulation(1000, 40, 100);
  TrySimulation(222, 40, 100);
  TrySimulation(333, 60, 100);
  TrySimulation(333, 60, 100);
}

void Benchmark() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::vector<float> costs;
  size_t N = 1000000;
  for (size_t i = 0; i < N; ++i) {
    costs.push_back(std::floor(test::rand01(gen) * N));
  }

  const auto labelcost = [&costs](const uint32_t label) { return costs[label]; };

  std::clock_t start = std::clock();
  baldr::DoubleBucketQueue adjlist5(0, N, 1, labelcost);
  Add(adjlist5, costs);
  TryRemove(adjlist5, costs.size(), costs);
  uint32_t ms = (std::clock() - start) / static_cast<double>(CLOCKS_PER_SEC / 1000);
  std::cout << ms << std::endl;
}

void TestRoutePathIterator() {
  meili::LabelSet labelset(100);
  // Travel mode is insignificant in the tests
  sif::TravelMode travelmode = static_cast<sif::TravelMode>(0);

  // Create a dummy DirectedEdge for use in LabelSet
  baldr::DirectedEdge de;

  // Construct two poor trees:
  //  0         1
  //         3     4
  //        5
  labelset.put(0, travelmode, nullptr);
  labelset.put(1, travelmode, nullptr);
  labelset.put(2, travelmode, nullptr);
  labelset.put(3, baldr::GraphId(), 0.f, 1.f, {0.f, 0.0f}, 0.f, 0.f, 1, &de, travelmode);
  labelset.put(4, baldr::GraphId(), 0.f, 1.f, {0.f, 0.0f}, 0.f, 0.f, 1, &de, travelmode);
  labelset.put(5, baldr::GraphId(), 0.f, 1.f, {0.f, 0.0f}, 0.f, 0.f, 3, &de, travelmode);
  labelset.put(6, baldr::GraphId(), 0.f, 1.f, {0.f, 0.0f}, 0.f, 0.f, 3, &de, travelmode);

  meili::RoutePathIterator the_end(&labelset, baldr::kInvalidLabel), it0(&labelset, 0),
      it1(&labelset, 1), it2(&labelset, 2), it3(&labelset, 3), it4(&labelset, 4), it5(&labelset, 5),
      it6(&labelset, 6);

  test::assert_bool(it0 != the_end, "TestRoutePathIterator: wrong equality testing");

  test::assert_bool(&(*it0) == &labelset.label(0), "TestRoutePathIterator: wrong dereferencing");

  test::assert_bool(it0->predecessor() == baldr::kInvalidLabel,
                    "TestRoutePathIterator: wrong dereferencing pointer");

  test::assert_bool(++it0 == the_end, "TestRoutePathIterator: wrong prefix increment");

  test::assert_bool(std::next(it3) == it1, "TestRoutePathIterator: wrong forwarding");

  test::assert_bool(std::next(it3, 2) == the_end, "TestRoutePathIterator: wrong forwarding 2");

  test::assert_bool(std::next(it4) == it1, "TestRoutePathIterator: wrong forwarding 3");

  test::assert_bool(std::next(it4, 2) == the_end, "TestRoutePathIterator: wrong forwarding 4");

  test::assert_bool(it4->predecessor() == 1, "TestRoutePathIterator: wrong dereferencing pointer 2");

  test::assert_bool((it5++)->predecessor() == 3, "TestRoutePathIterator: wrong postfix increment");

  test::assert_bool(it5 == it3, "TestRoutePathIterator: wrong after postfix increment");

  test::assert_bool(++it5 == it1, "TestRoutePathIterator: wrong prefix increment");

  test::assert_bool(it5 == it1, "TestRoutePathIterator: wrong after prefix increment");

  std::advance(it5, 1);
  test::assert_bool(it5 == the_end, "TestRoutePathIterator: wrong advance");
}

int main(int argc, char* argv[]) {
  test::suite suite("routing");

  suite.test(TEST_CASE(TestAddRemove));

  suite.test(TEST_CASE(TestSimulation));

  suite.test(TEST_CASE(Benchmark));

  suite.test(TEST_CASE(TestRoutePathIterator));

  return suite.tear_down();
}
