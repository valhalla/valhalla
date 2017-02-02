// -*- mode: c++ -*-
#include "test.h"
#include "meili/routing.h"

using namespace valhalla;
using namespace valhalla::meili;

constexpr uint32_t kInvalidKey = std::numeric_limits<uint16_t>::max();

using AdjacencyList = meili::BucketQueue<uint32_t, kInvalidKey>;


void Add(AdjacencyList &adjlist, const std::vector<float>& costs)
{
  uint32_t idx = 0;
  for (const auto cost : costs) {
    adjlist.add(idx++, cost);
  }
}


void TryRemove(AdjacencyList &adjlist, size_t num_to_remove, const std::vector<float>& costs)
{
  auto previous_cost = -std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < num_to_remove && !adjlist.empty(); ++i) {
    const auto top = adjlist.pop();
    const auto cost = costs[top];
    test::assert_bool(previous_cost <= cost, "TryAddRemove: expected order test failed");
    previous_cost = cost;
  }
}


void TestAddRemove()
{
  // Test add and remove
  AdjacencyList adjlist(100000);
  std::vector<float> costs = { 67, 325, 25, 466, 1000, 10000, 758, 167,
                               258, 16442, 278 };
  Add(adjlist, costs);
  TryRemove(adjlist, costs.size(), costs);
  test::assert_bool(adjlist.empty(), "TestAddRemove: expect list to be empty");

  // Test add randomly and remove
  costs.clear();
  for (size_t i = 0; i < 10000; i++) {
    const auto cost = std::floor(rand01() * 100000);
    costs.push_back(cost);
  }
  AdjacencyList adjlist2(10000);
  Add(adjlist2, costs);
  TryRemove(adjlist2, costs.size(), costs);
  test::assert_bool(adjlist2.empty(), "TestAddRemove: expect list to be empty");

  AdjacencyList adjlist3(10);
  adjlist3.add(1, 100);
  test::assert_bool(adjlist3.empty() && adjlist3.cost(1) < 0.f,
                    "TestAddRemove: 100 should not be added since it is out of the queue range [0, 10)");
  adjlist3.add(1, 10);
  test::assert_bool(adjlist3.empty() && adjlist3.cost(1) < 0.f,
                    "TestAddRemove: 10 should not be added since it is out of the queue range [0, 10)");
  adjlist3.add(1, 9);
  test::assert_bool(!adjlist3.empty() && adjlist3.cost(1) == 9,
                    "TestAddRemove: 9 should be added since it is within the queue range [0, 10)");
  adjlist3.decrease(1, 3);
  test::assert_bool(!adjlist3.empty() && adjlist3.cost(1) == 3,
                    "TestAddRemove: 3 should be decreased since it is less than the last cost 9");

  {
    test::assert_throw<std::runtime_error>([&adjlist3](){
        adjlist3.decrease(1, 5);
      },
      "TestAddRemove: it should fail to decrease cost to 5 since it is larger than last cost 3");
    test::assert_bool(!adjlist3.empty() && adjlist3.cost(1) == 3,
                      "TestAddRemove: the cost should still be 3 since it was failed to decrease to 5");
  }
}


void TrySimulation(AdjacencyList& adjlist, size_t loop_count, size_t expansion_size, size_t max_increment_cost)
{
  std::vector<float> costs;
  // Track all label indexes in the adjlist
  std::unordered_set<uint32_t> track;

  const uint32_t idx = costs.size();
  costs.push_back(10.f);
  bool inserted = adjlist.add(idx, 10.f);
  if (inserted) {
    track.insert(idx);
  }

  for (size_t i = 0; i < loop_count && !adjlist.empty(); i++) {
    const auto key = adjlist.pop();
    const auto min_cost = costs[key];
    // Must be the minimal one among the tracked labels
    for (auto k : track) {
      test::assert_bool(min_cost <= costs[k], "Simulation: minimal cost expected");
    }
    track.erase(key);

    for (size_t i = 0; i < expansion_size; i++) {
      const auto newcost = std::floor(min_cost + 1 + rand01() * max_increment_cost);
      if (i % 2 == 0 && !track.empty()) {
        // Decrease cost
        const auto idx = *std::next(track.begin(), rand01() * track.size());
        if (newcost < costs[idx]) {
          adjlist.decrease(idx, newcost);
          costs[idx] = newcost;
          test::assert_bool(adjlist.cost(idx) == newcost,
                            "failed to decrease cost");
        } else {
          // Assert that it must fail to decrease since costs[idx] <= newcost
          test::assert_throw<std::runtime_error>([&adjlist, idx, newcost](){
              adjlist.decrease(idx, newcost);
            }, "decreasing a non-less cost must fail");
        }
      } else {
        // Add new label
        const uint32_t idx = costs.size();
        costs.push_back(newcost);
        const bool inserted = adjlist.add(idx, newcost);
        if (inserted) {
          track.insert(idx);
        }
      }
    }
  }

  TryRemove(adjlist, track.size(), costs);
  test::assert_bool(adjlist.empty(), "Simulation: expect list to be empty");
}


void TestSimulation()
{
  AdjacencyList adjlist1(100000);
  TrySimulation(adjlist1, 1000, 40, 100);

  AdjacencyList adjlist2(100000);
  TrySimulation(adjlist2, 222, 40, 100);

  AdjacencyList adjlist3(100000);
  TrySimulation(adjlist3, 333, 60, 100);

  AdjacencyList adjlist4(1000);
  TrySimulation(adjlist4, 333, 60, 100);
}


void Benchmark()
{
  std::vector<float> costs;
  size_t N = 1000000;
  for (size_t i = 0; i < N; ++i) {
    costs.push_back(std::floor(rand01() * N));
  }

  std::clock_t start = std::clock();
  AdjacencyList adjlist5(N);
  Add(adjlist5, costs);
  TryRemove(adjlist5, costs.size(), costs);
  uint32_t ms = (std::clock() - start) / static_cast<double>(CLOCKS_PER_SEC / 1000);
  std::cout << ms << std::endl;
}


void TestRoutePathIterator()
{
  meili::LabelSet labelset(100);
  // Travel mode is insignificant in the tests
  sif::TravelMode travelmode = static_cast<sif::TravelMode>(0);

  // Construct two poor trees:
  //  0         1
  //         3     4
  //        5
  labelset.put(0, travelmode, nullptr);
  labelset.put(1, travelmode, nullptr);
  labelset.put(2, travelmode, nullptr);
  labelset.put(3, baldr::GraphId(),
               0.f, 1.f,
               0.f, 0.f, 0.f,
               1, nullptr, travelmode, nullptr);
  labelset.put(4, baldr::GraphId(),
               0.f, 1.f,
               0.f, 0.f, 0.f,
               1, nullptr, travelmode, nullptr);
  labelset.put(5, baldr::GraphId(),
               0.f, 1.f,
               0.f, 0.f, 0.f,
               3, nullptr, travelmode, nullptr);
  labelset.put(6, baldr::GraphId(),
               0.f, 1.f,
               0.f, 0.f, 0.f,
               3, nullptr, travelmode, nullptr);

  meili::RoutePathIterator the_end(&labelset, meili::kInvalidLabelIndex),
      it0(&labelset, 0),
      it1(&labelset, 1),
      it2(&labelset, 2),
      it3(&labelset, 3),
      it4(&labelset, 4),
      it5(&labelset, 5),
      it6(&labelset, 6);

  test::assert_bool(it0 != the_end,
                    "TestRoutePathIterator: wrong equality testing");

  test::assert_bool(&(*it0) == &labelset.label(0),
                    "TestRoutePathIterator: wrong dereferencing");

  test::assert_bool(it0->predecessor == meili::kInvalidLabelIndex,
                    "TestRoutePathIterator: wrong dereferencing pointer");

  test::assert_bool(++it0 == the_end,
                    "TestRoutePathIterator: wrong prefix increment");

  test::assert_bool(std::next(it3) == it1,
                    "TestRoutePathIterator: wrong forwarding");

  test::assert_bool(std::next(it3, 2) == the_end,
                    "TestRoutePathIterator: wrong forwarding 2");

  test::assert_bool(std::next(it4) == it1,
                    "TestRoutePathIterator: wrong forwarding 3");

  test::assert_bool(std::next(it4, 2) == the_end,
                    "TestRoutePathIterator: wrong forwarding 4");

  test::assert_bool(it4->predecessor == 1,
                    "TestRoutePathIterator: wrong dereferencing pointer 2");

  test::assert_bool((it5++)->predecessor == 3,
                    "TestRoutePathIterator: wrong postfix increment");

  test::assert_bool(it5 == it3,
                    "TestRoutePathIterator: wrong after postfix increment");

  test::assert_bool(++it5 == it1,
                    "TestRoutePathIterator: wrong prefix increment");

  test::assert_bool(it5 == it1,
                    "TestRoutePathIterator: wrong after prefix increment");

  std::advance(it5, 1);
  test::assert_bool(it5 == the_end,
                    "TestRoutePathIterator: wrong advance");
}


int main(int argc, char *argv[])
{
  test::suite suite("routing");

  suite.test(TEST_CASE(TestAddRemove));

  suite.test(TEST_CASE(TestSimulation));

  suite.test(TEST_CASE(Benchmark));

  suite.test(TEST_CASE(TestRoutePathIterator));

  return suite.tear_down();
}
