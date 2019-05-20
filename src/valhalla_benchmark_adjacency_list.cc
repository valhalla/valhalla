#include <boost/program_options.hpp>
#include <cstdint>
#include <iostream>
#include <queue>
#include <random>
#include <string>
#include <vector>

#include "config.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "sif/edgelabel.h"

#include "baldr/double_bucket_queue.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace bpo = boost::program_options;

/**
 * Benchmark of adjacency list. Constructs a large number of random numbers,
 * adds EdgeLabels to the AdjacencyList with those as the sortcost. Then
 * removes them from the list. This compares performance of an STL
 * priority_queue with the custom approximate double bucket sorting used
 * in adjacencylist.cc.
 */
int Benchmark(const uint32_t n, const float maxcost, const float bucketsize) {
  // Create a set of random costs
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 1);
  std::vector<uint32_t> costs(n);
  for (uint32_t i = 0; i < n; i++) {
    costs[i] = static_cast<uint32_t>(dis(gen) * maxcost);
  }

  // Test performance of STL priority queue. Construct labels
  // and add to the priority queue
  std::clock_t start = std::clock();
  std::priority_queue<EdgeLabel> pqueue;
  for (uint32_t i = 0; i < n; i++) {
    EdgeLabel el;
    el.SetSortCost(costs[i]);
    pqueue.push(std::move(el));
  }

  uint32_t count = 0;
  std::vector<uint32_t> ordered_cost1;
  while (!pqueue.empty()) {
    EdgeLabel el = pqueue.top();
    ordered_cost1.push_back(el.sortcost());
    pqueue.pop();
    count++;
  }
  uint32_t ms = (std::clock() - start) / static_cast<double>(CLOCKS_PER_SEC / 1000);
  LOG_INFO("Priority Queue: Added and removed " + std::to_string(count) + " edgelabels in " +
           std::to_string(ms) + " ms");

  // Didn't alter the sort order of priority queue, so reverse the vector
  std::reverse(ordered_cost1.begin(), ordered_cost1.end());

  // Test performance of double bucket adjacency list. Set the bucket maxcost
  // such that EmptyOverflow is called once
  std::vector<EdgeLabel> edgelabels;
  // Set up lambda to get sort costs
  const auto edgecost = [&edgelabels](const uint32_t label) { return edgelabels[label].sortcost(); };
  start = std::clock();
  DoubleBucketQueue adjlist(0, maxcost / 2, bucketsize, edgecost);

  // Construct EdgeLabels and add to adjacency list

  for (uint32_t i = 0; i < n; i++) {
    EdgeLabel el;
    el.SetSortCost(costs[i]);
    edgelabels.push_back(std::move(el));
    adjlist.add(i);
  }

  // Get edge label indexes from the adj list. Accumulate total cost to make
  // sure compiler doesn't optimize too much.
  count = 0;
  std::vector<uint32_t> ordered_cost2;
  while (true) {
    uint32_t idx = adjlist.pop();
    if (idx == kInvalidLabel) {
      break;
    }

    // Copy the edge label - simulates what is done in PathAlgorithm
    EdgeLabel el = edgelabels[idx];
    ordered_cost2.push_back(el.sortcost());
    count++;
  }
  ms = (std::clock() - start) / static_cast<double>(CLOCKS_PER_SEC / 1000);
  LOG_INFO("Bucketed Adj. List: Added and removed " + std::to_string(count) + " edgelabels in " +
           std::to_string(ms) + " ms");

  // Verify order
  for (uint32_t i = 0; i < count; i++) {
    if (ordered_cost1[i] != ordered_cost2[i]) {
      LOG_INFO("Costs: " + std::to_string(ordered_cost1[i]) + "," + std::to_string(ordered_cost2[i]));
    }
  }
  return 0;
}

int main(int argc, char* argv[]) {

  bpo::options_description options(
      "valhalla " VALHALLA_VERSION "\n"
      "\n"
      " Usage: adjlistbenchmark [options]\n"
      "\n"
      "adjlistbenchmark is benchmark comparing performance of an STL priority_queue"
      "to the approximate double bucket adjacency list class supplied with Valhalla."
      "\n"
      "\n");

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.");

  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).run(), vm);
    bpo::notify(vm);

  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }

  if (vm.count("version")) {
    std::cout << "AdjacencyListBenchmark " << VALHALLA_VERSION << "\n";
    return EXIT_SUCCESS;
  }

  // Benchmark with count, maxcost, and bucketsize
  Benchmark(1000000, 50000, 1);
  LOG_INFO("Done Benchmark!");

  return EXIT_SUCCESS;
}
