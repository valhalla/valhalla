#include "sp.h"

constexpr uint32_t kInvalidKey = std::numeric_limits<uint16_t>::max();

using AdjacencyList = BucketQueue<uint32_t, kInvalidKey>;


void Add(AdjacencyList &adjlist, const std::vector<float>& costs)
{
  uint32_t idx = 0;
  for (auto cost : costs) {
    adjlist.add(idx++, cost);
  }
}


void TryRemove(AdjacencyList &adjlist, size_t num_to_remove, const std::vector<float>& costs)
{
  auto previous_cost = -std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < num_to_remove && !adjlist.empty(); ++i) {
    auto top = adjlist.pop();
    auto cost = costs[top];
    if (!(previous_cost <= cost)) {
      throw std::runtime_error("TryAddRemove: expected order test failed");
    }
    previous_cost = cost;
  }
}


void TestAddRemove()
{
  AdjacencyList adjlist(100000);
  std::vector<float> costs = { 67, 325, 25, 466, 1000, 10000, 758, 167,
                               258, 16442, 278 };
  Add(adjlist, costs);
  TryRemove(adjlist, costs.size(), costs);
  if (!adjlist.empty()) {
    throw std::runtime_error("TestAddRemove: expect list to be empty");
  }

  costs.clear();
  for (size_t i = 0; i < 10000; i++) {
    auto cost = std::floor(rand01() * 100000);
    costs.push_back(cost);
  }
  AdjacencyList adjlist2(10000);
  Add(adjlist2, costs);
  TryRemove(adjlist2, costs.size(), costs);
  if (!adjlist2.empty()) {
    throw std::runtime_error("TestAddRemove: expect list to be empty");
  }

  AdjacencyList adjlist3(10);
  adjlist3.add(1, 100);
  if (!adjlist3.empty()) {
    throw std::runtime_error("TestAddRemove: 100 should be added");
  }
  adjlist3.add(1, 10);
  if (!adjlist3.empty()) {
    throw std::runtime_error("TestAddRemove: 10 shouldn't be added");
  }
  adjlist3.add(1, 9.99);
  if (adjlist3.empty()) {
    throw std::runtime_error("TestAddRemove: 9.99 should be added");
  }
  adjlist3.add(1, 9);
  if (adjlist3.empty()) {
    throw std::runtime_error("TestAddRemove: 9 should be added");
  }
}


void TrySimulation(AdjacencyList& adjlist, size_t loop_count, size_t expansion_size, size_t max_increment_cost)
{
  std::vector<float> costs;
  // Track all label indexes in the adjlist
  std::unordered_set<uint32_t> track;

  uint32_t idx = costs.size();
  costs.push_back(10.f);
  bool inserted = adjlist.add(idx, 10.f);
  if (inserted) {
    track.insert(idx);
  }

  for (size_t i = 0; i < loop_count && !adjlist.empty(); i++) {
    auto key = adjlist.pop();
    auto min_cost = costs[key];
    // Must be the minimal one among the tracked labels
    for (auto k : track) {
      if (costs[k] < min_cost) {
        throw std::runtime_error("Simulation: minimal cost expected");
      }
    }
    track.erase(key);

    for (size_t i = 0; i < expansion_size; i++) {
      auto newcost = std::floor(min_cost + 1 + rand01() * max_increment_cost);
      if (i % 2 == 0 && !track.empty()) {
        // Decrease cost
        auto idx = *std::next(track.begin(), rand01() * track.size());
        bool updated = adjlist.add(idx, newcost);
        if (updated) {
          if (newcost > costs[idx]) {
            throw std::runtime_error("Simulation: wrong");
          }
          costs[idx] = newcost;
        }
      } else {
        // Add new label
        uint32_t idx = costs.size();
        costs.push_back(newcost);
        bool inserted = adjlist.add(idx, newcost);
        if (inserted) {
          track.insert(idx);
        }
      }
    }
  }

  TryRemove(adjlist, track.size(), costs);
  if (!adjlist.empty()) {
    std::cout << adjlist.size() << std::endl;
    throw std::runtime_error("Simulation: expect list to be empty");
  }
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
  LabelSet labelset(100);
  // Construct two poor trees:
  //  0         1
  //         3     4
  //        5
  labelset.put(0);
  labelset.put(1);
  labelset.put(2);
  labelset.put(3, GraphId(), 0.f, 1.f, 0.f, 1);
  labelset.put(4, GraphId(), 0.f, 1.f, 0.f, 1);
  labelset.put(5, GraphId(), 0.f, 1.f, 0.f, 3);
  labelset.put(6, GraphId(), 0.f, 1.f, 0.f, 3);

  RoutePathIterator the_end(&labelset, kInvalidLabelIndex),
      it0(&labelset, 0),
      it1(&labelset, 1),
      it2(&labelset, 2),
      it3(&labelset, 3),
      it4(&labelset, 4),
      it5(&labelset, 5),
      it6(&labelset, 6);

  if (it0 == the_end) {
    throw std::runtime_error("TestRoutePathIterator: wrong equality testing");
  }

  if (&(*it0) != &labelset.label(0)) {
    throw std::runtime_error("TestRoutePathIterator: wrong dereferencing");
  }

  if (it0->predecessor != kInvalidLabelIndex) {
    throw std::runtime_error("TestRoutePathIterator: wrong dereferencing pointer");
  }

  if (++it0 != the_end) {
    throw std::runtime_error("TestRoutePathIterator: wrong prefix increment");
  }

  if (std::next(it3) != it1) {
    throw std::runtime_error("TestRoutePathIterator: wrong forwarding");
  }

  if (std::next(it3, 2) != the_end) {
    throw std::runtime_error("TestRoutePathIterator: wrong forwarding 2");
  }

  if (std::next(it4) != it1) {
    throw std::runtime_error("TestRoutePathIterator: wrong forwarding 3");
  }

  if (std::next(it4, 2) != the_end) {
    throw std::runtime_error("TestRoutePathIterator: wrong forwarding 4");
  }

  if (it4->predecessor != 1) {
    throw std::runtime_error("TestRoutePathIterator: wrong dereferencing pointer 2");
  }

  if ((it5++)->predecessor != 3) {
    throw std::runtime_error("TestRoutePathIterator: wrong postfix increment");
  }

  if (it5 != it3) {
    throw std::runtime_error("TestRoutePathIterator: wrong after postfix increment");
  }

  if (++it5 != it1) {
    throw std::runtime_error("TestRoutePathIterator: wrong prefix increment");
  }

  if (it5 != it1) {
    throw std::runtime_error("TestRoutePathIterator: wrong after prefix increment");
  }

  std::advance(it5, 1);
  if (it5 != the_end) {
    throw std::runtime_error("TestRoutePathIterator: wrong advance");
  }
}


int main(int argc, char *argv[])
{
  TestAddRemove();

  TestSimulation();

  Benchmark();

  TestRoutePathIterator();

  return 0;
}
