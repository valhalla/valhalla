#include <algorithm>
#include <atomic>
#include <fstream>
#include <future>
#include <list>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/rapidjson_utils.h"
#include "filesystem.h"
#include "loki/search.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "sif/costfactory.h"
#include "worker.h"

#include "argparse_utils.h"

std::string costing_str;

using job_t = std::vector<valhalla::baldr::Location>;
std::vector<job_t> jobs;
std::atomic<size_t> job_index(0);
std::string geojson(const job_t& job) {
  std::string json = "{";
  for (const auto& l : job) {
    json.push_back('{');
    json.append(std::to_string(l.latlng_.lng()));
    json.push_back(',');
    json.append(std::to_string(l.latlng_.lat()));
    json.push_back('}');
    if (&l != &job.back()) {
      json.push_back(',');
    }
  }
  json.push_back('}');
  return json;
}

struct result_t {
  std::chrono::milliseconds time;
  bool pass;
  job_t job;
  bool cached;
  bool operator<(const result_t& other) const {
    if (cached < other.cached) {
      return true;
    }
    if (time < other.time) {
      return true;
    }
    if (pass < other.pass) {
      return true;
    }
    if (job.size() < other.job.size()) {
      return true;
    }
    return job.front().latlng_ < other.job.front().latlng_;
  }
  bool operator==(const result_t& other) const {
    return cached == other.cached && time == other.time && pass == other.pass && job == other.job;
  }
};
using results_t = std::set<result_t>;

valhalla::sif::cost_ptr_t create_costing() {
  valhalla::Options options;
  valhalla::Costing::Type costing;
  if (valhalla::Costing_Enum_Parse(costing_str, &costing)) {
    options.set_costing_type(costing);
  } else {
    options.set_costing_type(valhalla::Costing::none_);
  }
  (*options.mutable_costings())[costing];
  return valhalla::sif::CostFactory{}.Create(options);
}

void work(const boost::property_tree::ptree& config, std::promise<results_t>& promise) {
  // lambda to do the current job
  auto costing = create_costing();
  valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));
  auto search = [&reader, &costing](const job_t& job) {
    // so that we dont benefit from cache coherency
    reader.Clear();
    std::pair<result_t, result_t> result;
    bool cached = false;
    for (auto* r : {&result.first, &result.second}) {
      auto start = std::chrono::high_resolution_clock::now();
      try {
        // TODO: actually save the result
        auto result = valhalla::loki::Search(job, reader, costing);
        auto end = std::chrono::high_resolution_clock::now();
        (*r) = result_t{std::chrono::duration_cast<std::chrono::milliseconds>(end - start), true, job,
                        cached};
      } catch (...) {
        auto end = std::chrono::high_resolution_clock::now();
        (*r) = result_t{std::chrono::duration_cast<std::chrono::milliseconds>(end - start), false,
                        job, cached};
      }
      cached = true;
    }
    return result;
  };

  // pull work off and do it
  results_t results;
  size_t i;
  while ((i = job_index.fetch_add(1)) < jobs.size()) {
    auto result = search(jobs[i]);
    results.emplace(std::move(result.first));
    results.emplace(std::move(result.second));
  }

  // return the statistics
  promise.set_value(std::move(results));
}

int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  size_t batch, isolated, radius;
  bool extrema = false;
  std::vector<std::string> input_files;
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_VERSION + "\n\n"
      "a program that does location searches on tiled route data.\n"
      "To run it use a valid config file to let it know where the tiled route data \n"
      "is. The input is simply a text file of one location per line\n\n");

    std::string search_type;
    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("j,concurrency", "Number of threads to use. Defaults to all threads.", cxxopts::value<uint32_t>())
      ("b,batch", "Number of locations to group together per search", cxxopts::value<size_t>(batch)->default_value("1"))
      ("e,extrema", "Show the input locations of the extrema for a given statistic", cxxopts::value<bool>(extrema)->default_value("false"))
      ("i,reach", "How many edges need to be reachable before considering it as connected to the larger network", cxxopts::value<size_t>(isolated))
      ("r,radius", "How many meters to search away from the input location", cxxopts::value<size_t>(radius)->default_value("0"))
      ("costing", "Which costing model to use.", cxxopts::value<std::string>(costing_str)->default_value("auto"))
      ("input_files", "positional arguments", cxxopts::value<std::vector<std::string>>(input_files));
    // clang-format on

    options.parse_positional({"input_files"});
    options.positional_help("LOCATIONS.TXT");
    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, config, "loki.logging"))
      return EXIT_SUCCESS;

    if (!result.count("input_files")) {
      throw cxxopts::exceptions::exception("Input file is required\n\n" + options.help());
    }
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // fill up the queue with work
  job_t job;
  for (const auto& file : input_files) {
    std::ifstream stream(file);
    std::string line;
    while (std::getline(stream, line)) {
      // Parse line to get lat,lon
      std::stringstream ss(line);
      std::string item;
      std::vector<std::string> parts;
      while (std::getline(ss, item, ' ')) {
        parts.push_back(std::move(item));
      }
      float lat = std::stof(parts[0]);
      if (lat < -90.0f || lat > 90.0f) {
        throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
      }
      float lon = valhalla::midgard::circular_range_clamp<float>(std::stof(parts[1]), -180, 180);
      valhalla::midgard::PointLL ll(lat, lon);
      valhalla::baldr::Location loc(ll);
      loc.min_inbound_reach_ = loc.min_outbound_reach_ = isolated;
      loc.radius_ = radius;
      job.emplace_back(std::move(loc));
      if (job.size() == batch) {
        jobs.emplace_back(std::move(job));
        job.clear();
      }
      line.clear();
    }
  }
  if (!job.empty()) {
    jobs.emplace_back(std::move(job));
  }

  // start up the threads
  std::list<std::thread> pool;
  const auto num_threads = config.get<uint32_t>("mjolnir.concurrency");
  std::vector<std::promise<results_t>> pool_results(num_threads);
  for (size_t i = 0; i < num_threads; ++i) {
    pool.emplace_back(work, std::cref(config), std::ref(pool_results[i]));
  }

  // let the threads finish up
  for (auto& thread : pool) {
    thread.join();
  }

  // grab all the results
  results_t results;
  for (auto& thread_results : pool_results) {
    try {
      auto result = thread_results.get_future().get();
      std::move(std::begin(result), std::end(result), std::inserter(results, results.begin()));
    } // rethrow anything that happened in a thread
    catch (std::exception& e) {
      throw e;
    }
  }

  // do some statistics,
  const std::vector<std::tuple<std::string, bool, bool>> stat_types =
      {std::make_tuple("Succeeded Searches on Uncached Tiles", true, false),
       std::make_tuple("Failed Searches on Uncached Tiles", false, false),
       std::make_tuple("Succeeded Searches on Cached Tiles", true, true),
       std::make_tuple("Failed Searches on Cached Tiles", false, true)};
  for (const auto& stat_type : stat_types) {
    // grab the averages and the best and worst cases
    size_t count = 0;
    std::chrono::duration<double, std::milli> time(0);
    result_t first, last;
    for (const auto& result : results) {
      // are we interested in this result
      if (std::get<1>(stat_type) == result.pass && std::get<2>(stat_type) == result.cached) {
        time += result.time;
        if (count == 0) {
          first = result;
          last = result;
        } else {
          if (result.time < first.time) {
            first = result;
          }
          if (result.time > last.time) {
            last = result;
          }
        }
        count++;
      }
    }
    double median = time.count() / static_cast<double>(count);
    // grab the std deviation
    double sum_squared_difference = 0;
    for (const auto& result : results) {
      // are we interested in this result
      if (std::get<1>(stat_type) == result.pass && std::get<2>(stat_type) == result.cached) {
        auto ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(result.time)
                      .count();
        auto diff = ms - median;
        sum_squared_difference += diff * diff;
      }
    }
    auto variance = sum_squared_difference / static_cast<double>(count);
    auto std_deviation = sqrt(variance);
    // stats about outliers
    size_t faster = 0, slower = 0;
    for (const auto& result : results) {
      // are we interested in this result
      if (std::get<1>(stat_type) == result.pass && std::get<2>(stat_type) == result.cached) {
        auto ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(result.time)
                      .count();
        if (ms < median - std_deviation) {
          faster++;
        } else if (ms > median + std_deviation) {
          slower++;
        }
      }
    }

    LOG_INFO(std::get<0>(stat_type));
    LOG_INFO("--------------------------------");
    if (count) {
      LOG_INFO("Total: " + std::to_string(count));
      auto fast =
          std::chrono::duration_cast<std::chrono::duration<int, std::milli>>(first.time).count();
      auto fast_per = static_cast<double>(fast) / first.job.size();
      LOG_INFO("Fastest: " + std::to_string(fast) + "ms (" + std::to_string(fast_per) + "ms per)" +
               (extrema ? geojson(first.job) : ""));
      auto slow =
          std::chrono::duration_cast<std::chrono::duration<int, std::milli>>(last.time).count();
      auto slow_per = static_cast<double>(slow) / last.job.size();
      LOG_INFO("Slowest: " + std::to_string(slow) + "ms (" + std::to_string(slow_per) + "ms per)" +
               (extrema ? geojson(last.job) : ""));
      LOG_INFO("Median: " + std::to_string(median) + "ms");
      LOG_INFO("Standard Deviation: " + std::to_string(std_deviation) + "ms");
      LOG_INFO("Faster Than 1 Standard Deviation: " + std::to_string(faster));
      LOG_INFO("Slower Than 1 Standard Deviation: " + std::to_string(slower));
    } else {
      LOG_INFO("No results");
    }
    LOG_INFO("--------------------------------\n\n");
  }

  return EXIT_SUCCESS;
}
