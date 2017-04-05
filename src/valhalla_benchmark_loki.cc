#include "config.h"

#include "loki/search.h"
#include "baldr/graphfsreader.h"
#include "midgard/logging.h"

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lockfree/queue.hpp>
#include <fstream>
#include <string>
#include <thread>
#include <future>
#include <vector>
#include <list>
#include <set>
#include <tuple>
#include <algorithm>

namespace bpo = boost::program_options;

boost::filesystem::path config_file_path;
size_t threads = std::max(static_cast<size_t>(std::thread::hardware_concurrency()), static_cast<size_t>(1));
std::vector<std::string> input_files;
std::atomic<bool> done(false);

struct job_t{
  float lng, lat;
};
boost::lockfree::queue<job_t> jobs(1024);
struct result_t {
  std::chrono::milliseconds time;
  bool pass;
  job_t job;
  bool cached;
  bool operator<(const result_t& other) const {
    if(cached < other.cached)
      return true;
    if(time < other.time)
      return true;
    if(pass < other.pass)
      return true;
    if(job.lng == other.job.lng)
      return job.lat < other.job.lat;
    return job.lng < other.job.lng;
  }
  bool operator==(const result_t& other) const {
    return cached == other.cached && time == other.time && pass == other.pass && job.lng == other.job.lng && job.lat == other.job.lat;
  }
};
using results_t = std::set<result_t>;

bool ParseArguments(int argc, char *argv[]) {

  bpo::options_description options(
    "search " VERSION "\n"
    "\n"
    " Usage: loki_benchmark [options] <location_input_file> ...\n"
    "\n"
    "loki_benchmark is a program that does location searches on tiled route data. "
    "To run it use the conf file in conf/valhalla.json to let it know where the "
    "tiled route data is. The input is simply a text file of one location per line"
    "\n"
    "\n");

  std::string search_type;
  options.add_options()
      ("help,h", "Print this help message.")
      ("version,v", "Print the version of this software.")
      ("config,c",
        boost::program_options::value<boost::filesystem::path>(&config_file_path),
        "Path to the json configuration file.")
      ("threads,t",
        boost::program_options::value<size_t>(&threads),
        "Concurrency to use.")
      //positional arguments
      ("input_files", boost::program_options::value<std::vector<std::string> >(&input_files)->multitoken());

  bpo::positional_options_description pos_options;
  pos_options.add("input_files", 16);

  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(), vm);
    bpo::notify(vm);

  } catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what()
      << "\n" << "This is a bug, please report it at " PACKAGE_BUGREPORT
      << "\n";
    return false;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return true;
  }

  if (vm.count("version")) {
    std::cout << "loki_benchmark " << VERSION << "\n";
    return true;
  }

  // argument checking and verification
  for (auto arg : std::vector<std::string> { "config", "input_files" }) {
    if (vm.count(arg) == 0) {
      std::cerr << "The <" << arg << "> argument was not provided, but is mandatory\n\n";
      std::cerr << options << "\n";
      return false;
    }
  }

  //TODO: complain when no input files

  return true;
}

void work(const boost::property_tree::ptree& config, std::promise<results_t>& promise) {
  //lambda to do the current job
  auto search = [&config] (const job_t job) {
    //so that we dont benefit from cache coherency we always make a new reader
    valhalla::baldr::GraphFsReader reader(config.get_child("mjolnir"));
    auto location = valhalla::baldr::Location({job.lng, job.lat});
    std::pair<result_t, result_t> result;
    bool cached = false;
    for(auto r : {&result.first, &result.second}) {
      auto start = std::chrono::high_resolution_clock::now();
      try {
        //TODO: actually save the result
        auto result = valhalla::loki::Search({location}, reader, valhalla::loki::PassThroughEdgeFilter);
        const auto& c = result.at(location);
        auto end = std::chrono::high_resolution_clock::now();
        (*r) = result_t{std::chrono::duration_cast<std::chrono::milliseconds>(end - start), true, job, cached};
      }
      catch(...) {
        auto end = std::chrono::high_resolution_clock::now();
        (*r) = result_t{std::chrono::duration_cast<std::chrono::milliseconds>(end - start), false, job, cached};
      }
      cached = true;
    }
    return result;
  };

  //pull work off and do it
  job_t job;
  results_t results;
  while(!done){
    while(jobs.pop(job)) {
      auto result = search(job);
      results.emplace(std::move(result.first));
      results.emplace(std::move(result.second));
    }
  }
  while(jobs.pop(job)) {
    auto result = search(job);
    results.emplace(std::move(result.first));
    results.emplace(std::move(result.second));
  }

  //return the statistics
  promise.set_value(std::move(results));
}

int main(int argc, char** argv) {

  if (!ParseArguments(argc, argv))
    return EXIT_FAILURE;

  //check what type of input we are getting
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config_file_path.c_str(), pt);

  //configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree = pt.get_child_optional("loki.logging");
  if(logging_subtree) {
    auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&,
      std::unordered_map<std::string, std::string> >(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  //start up the threads
  std::list<std::thread> pool;
  std::vector<std::promise<results_t> > pool_results(threads);
  for(size_t i = 0; i < threads; ++i) {
    pool.emplace_back(work, std::cref(pt), std::ref(pool_results[i]));
  }

  //let the main thread rip through the file
  for(const auto& file : input_files) {
    std::ifstream stream(file);
    std::string line;
    while(std::getline(stream, line)) {
      auto location = valhalla::baldr::Location::FromCsv(line);
      jobs.push(job_t{location.latlng_.lng(), location.latlng_.lat()});
      line.clear();
    }
  }
  done = true;

  //let the threads finish up
  for(auto& thread : pool) {
    thread.join();
  }

  //grab all the results
  results_t results;
  for(auto& thread_results : pool_results) {
    try {
      auto result = thread_results.get_future().get();
      std::move(std::begin(result), std::end(result), std::inserter(results, results.begin()));
    }//rethrow anything that happened in a thread
    catch(std::exception& e) {
      throw e;
    }
  }

  //do some statistics,
  const std::vector<std::tuple<std::string, bool, bool> > stat_types =
    {
      std::make_tuple("Succeeded Searches on Uncached Tiles", true, false),
      std::make_tuple("Failed Searches on Uncached Tiles", false, false),
      std::make_tuple("Succeeded Searches on Cached Tiles", true, true),
      std::make_tuple("Failed Searches on Cached Tiles", false, true)
    };
  for(const auto& stat_type : stat_types) {
    //grab the averages and the best and worst cases
    size_t count = 0;
    std::chrono::duration<double, std::milli> time(0);
    result_t first, last;
    for(const auto& result : results) {
      //are we interested in this result
      if(std::get<1>(stat_type) == result.pass && std::get<2>(stat_type) == result.cached) {
        time += result.time;
        if(count == 0) {
          first = result;
          last = result;
        }
        else {
          if(result.time < first.time)
            first = result;
          if(result.time > last.time)
            last = result;
        }
        count++;
      }
    }
    double median = time.count() / static_cast<double>(count);
    //grab the std deviation
    double sum_squared_difference = 0;
    for(const auto& result : results) {
      //are we interested in this result
      if(std::get<1>(stat_type) == result.pass && std::get<2>(stat_type) == result.cached) {
        auto ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(result.time).count();
        auto diff = ms - median;
        sum_squared_difference += diff * diff;
      }
    }
    auto variance = sum_squared_difference / static_cast<double>(count);
    auto std_deviation = sqrt(variance);
    //stats about outliers
    size_t faster = 0, slower = 0;
    for(const auto& result : results) {
      //are we interested in this result
      if(std::get<1>(stat_type) == result.pass && std::get<2>(stat_type) == result.cached) {
        auto ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(result.time).count();
        if(ms < median - std_deviation)
          faster++;
        else if(ms > median + std_deviation)
          slower++;
      }
    }

    LOG_INFO(std::get<0>(stat_type));
    LOG_INFO("--------------------------------");
    if(count) {
      LOG_INFO("Total: " + std::to_string(count));
      LOG_INFO("Fastest: " + std::to_string(first.job.lat) + "," + std::to_string(first.job.lng) + " @ " +
        std::to_string(std::chrono::duration_cast<std::chrono::duration<int, std::milli> >(first.time).count()) + "ms");
      LOG_INFO("Slowest: " + std::to_string(last.job.lat) + "," + std::to_string(last.job.lng) + " @ " +
        std::to_string(std::chrono::duration_cast<std::chrono::duration<int, std::milli> >(last.time).count()) + "ms");
      LOG_INFO("Median: " + std::to_string(median) + "ms");
      LOG_INFO("Standard Deviation: " + std::to_string(std_deviation) + "ms");
      LOG_INFO("Faster Than 1 Standard Deviation: " + std::to_string(faster));
      LOG_INFO("Slower Than 1 Standard Deviation: " + std::to_string(slower));
    }
    else {
      LOG_INFO("No results");
    }
    LOG_INFO("--------------------------------\n\n");
  }

  return EXIT_SUCCESS;
}

