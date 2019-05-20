#include <chrono>
#include <cstdlib>
#include <fstream>
#include <list>
#include <thread>
#include <utility>
#include <vector>

#include "midgard/logging.h"
#include "skadi/sample.h"

void get_samples(const valhalla::skadi::sample& sample,
                 const std::list<std::pair<double, double>>& postings,
                 size_t id) {
  LOG_INFO("Thread" + std::to_string(id) + " sampling " + std::to_string(postings.size()) +
           " postings");
  auto values = sample.get_all(postings);
  size_t no_data_value = 0;
  for (auto v : values) {
    no_data_value += v == valhalla::skadi::sample::get_no_data_value();
  }
  LOG_INFO("Thread" + std::to_string(id) + " finished with " + std::to_string(no_data_value) +
           " no data values");
}

int main(int argc, char** argv) {

  // check args
  if (argc < 2) {
    throw std::runtime_error("No data source specified");
  }
  if (argc < 3) {
    throw std::runtime_error("No coordinate postings provided");
  }
  size_t thread_count;
  if (argc > 3) {
    thread_count = std::stoul(argv[3]);
  } else {
    thread_count = std::max<size_t>(std::thread::hardware_concurrency(), 1);
  }

  LOG_INFO("Loading elevation data");
  valhalla::skadi::sample sample(argv[1]);

  LOG_INFO("Loading coordinate postings");
  std::vector<std::list<std::pair<double, double>>> postings(thread_count);
  auto posting = postings.end() - 1;
  size_t posting_count = 0;
  std::ifstream file(argv[2]);
  do {
    ++posting;
    if (posting == postings.end()) {
      posting = postings.begin();
    }
    posting->emplace_back();
    ++posting_count;
  } while (file >> posting->back().first >> posting->back().second);
  posting->pop_back();
  --posting_count;

  // run the threads
  auto start = std::chrono::system_clock::now();
  std::list<std::thread> threads;
  size_t id = 0;
  for (const auto& p : postings) {
    threads.emplace_back(get_samples, std::cref(sample), std::cref(p), id++);
  }
  for (auto& t : threads) {
    t.join();
  }
  std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - start;
  LOG_INFO(std::to_string(posting_count / elapsed.count()) + " postings per second");

  return EXIT_SUCCESS;
}
