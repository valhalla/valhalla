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


int main(int argc, char *argv[])
{
  TestAddRemove();

  TestSimulation();

  Benchmark();

  return 0;
}
