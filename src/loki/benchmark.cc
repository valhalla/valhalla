#include "config.h"

#include "loki/search.h"
#include <valhalla/midgard/logging.h>

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
#include <algorithm>
#include <tuple>

namespace bpo = boost::program_options;

boost::filesystem::path config_file_path;
size_t threads = std::max(static_cast<size_t>(std::thread::hardware_concurrency()), static_cast<size_t>(1));
std::vector<std::string> input_files;
std::atomic<bool> done(false);

struct ll{
  float lng, lat;
};
boost::lockfree::queue<ll> jobs(512);

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

void work(const boost::property_tree::ptree& config, std::promise<std::tuple<size_t, size_t, std::chrono::milliseconds> >& result) {
  //things we will need for each job we do
  valhalla::baldr::GraphReader reader(config.get_child("mjolnir.hierarchy"));
  ll job;
  std::tuple<size_t, size_t, std::chrono::milliseconds> stats;

  //lambda to do the current job
  auto search = [&reader, &job, &stats] () {
    try {
      auto location = valhalla::baldr::Location({job.lng, job.lat});
      auto correlated = valhalla::loki::Search(location, reader, valhalla::loki::PathThroughFilter);
      std::get<0>(stats) = std::get<0>(stats) + 1;
    }
    catch(...) {
      std::get<1>(stats) = std::get<1>(stats) + 1;
    }
  };

  //pull work off and do it
  auto start = std::chrono::high_resolution_clock::now();
  while(!done){
    while(jobs.pop(job)) {
      search();
    }
  }
  while(jobs.pop(job)) {
    search();
  }
  auto end = std::chrono::high_resolution_clock::now();

  //return the statistics
  std::get<2>(stats) = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  result.set_value(stats);
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

  std::vector<std::thread> pool;
  std::vector<std::promise<std::tuple<size_t, size_t, std::chrono::milliseconds> > > results(threads);
  for(size_t i = 0; i < threads; ++i) {
    pool.emplace_back(work, pt, std::ref(results[i]));
  }

  //let the main thread rip through the file
  for(const auto& file : input_files) {
    std::ifstream stream(file);
    std::string line;
    while(std::getline(stream, line)) {
      auto location = valhalla::baldr::Location::FromCsv(line);
      jobs.push(ll{location.latlng_.lng(), location.latlng_.lat()});
      line.clear();
    }
  }
  done = true;

  //let the threads finish up
  for(auto& thread : pool) {
    thread.join();
  }

  // Check all of the outcomes
  size_t succeeded = 0;
  size_t failed = 0;
  std::chrono::duration<double, std::milli> duration(0);
  for(auto& result : results) {
    try {
      auto stats = result.get_future().get();
      if(std::get<0>(stats) > 0 || std::get<1>(stats) > 0) {
        succeeded += std::get<0>(stats);
        failed += std::get<1>(stats);
        duration += std::get<2>(stats);
      }
    }//rethrow anything that happened in a thread
    catch(std::exception& e) {
      throw e;
    }
  }

  LOG_INFO("Succeeded: " + std::to_string(succeeded));
  LOG_INFO("Failed: " + std::to_string(failed));
  LOG_INFO("Avg Search: " + std::to_string(duration.count()/static_cast<double>(succeeded + failed)) + "ms");

  return EXIT_SUCCESS;
}

