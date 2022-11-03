#include "baldr/rapidjson_utils.h"
#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cxxopts.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "config.h"

#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "odin/directionsbuilder.h"
#include "odin/util.h"
#include "sif/costfactory.h"
#include "thor/costmatrix.h"
#include "thor/optimizer.h"
#include "thor/timedistancematrix.h"
#include "worker.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::sif;
using namespace valhalla::thor;

// Format the time string
std::string GetFormattedTime(uint32_t secs) {
  if (secs == 0) {
    return "0";
  }
  uint32_t hours = secs / 3600;
  uint32_t minutes = (secs / 60) % 60;
  uint32_t seconds = secs % 60;
  return std::to_string(hours) + ":" + std::to_string(minutes) + ":" + std::to_string(seconds);
}

// Log results
void LogResults(const bool optimize,
                const valhalla::Options& options,
                const std::vector<TimeDistance>& res) {
  LOG_INFO("Results:");
  uint32_t idx1 = 0;
  uint32_t idx2 = 0;
  uint32_t nlocs = options.sources_size();
  for (auto& td : res) {
    LOG_INFO(std::to_string(idx1) + "," + std::to_string(idx2) +
             ": Distance= " + std::to_string(td.dist) + " Time= " + GetFormattedTime(td.time) +
             " secs = " + std::to_string(td.time));
    idx2++;
    if (idx2 == nlocs) {
      idx2 = 0;
      idx1++;
    }
  }
  if (optimize) {
    // Optimize the path
    auto t10 = std::chrono::high_resolution_clock::now();
    std::vector<float> costs;
    costs.reserve(res.size());
    for (auto& td : res) {
      costs.push_back(static_cast<float>(td.time));
    }

    Optimizer opt;
    auto tour = opt.Solve(nlocs, costs);
    LOG_INFO("Optimal Tour:");
    for (auto& loc : tour) {
      LOG_INFO("   : " + std::to_string(loc));
    }
    auto t11 = std::chrono::high_resolution_clock::now();
    uint32_t ms1 = std::chrono::duration_cast<std::chrono::milliseconds>(t11 - t10).count();
    LOG_INFO("Optimization took " + std::to_string(ms1) + " ms");
  }
}

// Main method for testing time and distance matrix methods
int main(int argc, char* argv[]) {
  // args
  std::string json_str, config;
  uint32_t iterations;

  try {
    // clang-format off
    cxxopts::Options options(
      "valhalla_run_matrix",
      "valhalla_run_matrix " VALHALLA_VERSION "\n\n"
      "valhalla_run_matrix is a command line test tool for time+distance matrix routing.\n"
      "Use the -j option for specifying source to target locations.");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("j,json", "JSON Example: "
        "'{\"locations\":[{\"lat\":40.748174,\"lon\":-73.984984,\"type\":\"break\",\"heading\":200,"
        "\"name\":\"Empire State Building\",\"street\":\"350 5th Avenue\",\"city\":\"New "
        "York\",\"state\":\"NY\",\"postal_code\":\"10118-0110\",\"country\":\"US\"},{\"lat\":40."
        "749231,\"lon\":-73.968703,\"type\":\"break\",\"name\":\"United Nations "
        "Headquarters\",\"street\":\"405 East 42nd Street\",\"city\":\"New "
        "York\",\"state\":\"NY\",\"postal_code\":\"10017-3507\",\"country\":\"US\"}],\"costing\":"
        "\"auto\",\"directions_options\":{\"units\":\"miles\"}}'", cxxopts::value<std::string>())
      ("m,multi-run", "Generate the route N additional times before exiting.", cxxopts::value<uint32_t>()->default_value("1"))
      ("c,config", "Valhalla configuration file", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("version")) {
      std::cout << "valhalla_run_matrix " << VALHALLA_VERSION << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("config") &&
        filesystem::is_regular_file(filesystem::path(result["config"].as<std::string>()))) {
      config = result["config"].as<std::string>();
    } else {
      std::cerr << "Configuration file is required\n\n" << options.help() << "\n\n";
      return EXIT_FAILURE;
    }

    if (!result.count("json")) {
      std::cerr << "A JSON format request must be present."
                << "\n";
      return EXIT_FAILURE;
    }
    json_str = result["json"].as<std::string>();

    iterations = result["multi-run"].as<uint32_t>();
  } catch (const cxxopts::OptionException& e) {
    std::cout << "Unable to parse command line options because: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  Api request;
  ParseApi(json_str, valhalla::Options::sources_to_targets, request);
  const auto& options = request.options();

  // parse the config
  boost::property_tree::ptree pt;
  rapidjson::read_json(config.c_str(), pt);

  // configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      pt.get_child_optional("thor.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  // Get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir"));

  // Construct costing
  CostFactory factory;

  // Get type of route - this provides the costing method to use.
  const std::string& routetype = valhalla::Costing_Enum_Name(options.costing_type());
  LOG_INFO("routetype: " + routetype);

  // Get the costing method - pass the JSON configuration
  sif::TravelMode mode;
  auto mode_costing = factory.CreateModeCosting(options, mode);

  // Find path locations (loki) for sources and targets
  auto t0 = std::chrono::high_resolution_clock::now();
  loki_worker_t lw(pt);
  lw.matrix(request);
  auto t1 = std::chrono::high_resolution_clock::now();
  uint32_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  LOG_INFO("Location Processing took " + std::to_string(ms) + " ms");

  // Get the max matrix distances for construction of the CostMatrix and TimeDistanceMatrix classes
  std::unordered_map<std::string, float> max_matrix_distance;
  for (const auto& kv : pt.get_child("service_limits")) {
    // Skip over any service limits that are not for a costing method
    if (kv.first == "max_exclude_locations" || kv.first == "max_reachability" ||
        kv.first == "max_radius" || kv.first == "max_timedep_distance" || kv.first == "skadi" ||
        kv.first == "trace" || kv.first == "isochrone" || kv.first == "centroid" ||
        kv.first == "max_alternates" || kv.first == "max_exclude_polygons_length" ||
        kv.first == "status") {
      continue;
    }
    max_matrix_distance.emplace(kv.first,
                                pt.get<float>("service_limits." + kv.first + ".max_matrix_distance"));
  }

  if (max_matrix_distance.empty()) {
    throw std::runtime_error("Missing max_matrix_distance configuration");
  }
  auto m = max_matrix_distance.find(routetype);
  float max_distance;
  if (m == max_matrix_distance.end()) {
    LOG_ERROR("Could not find max_matrix_distance for " + routetype);
    max_distance = 4000000.0f;
  } else {
    max_distance = m->second;
  }

  // If the sources and targets are equal we can run optimize
  bool optimize = true;
  if (options.sources_size() == options.targets_size()) {
    for (uint32_t i = 0; i < options.sources_size(); ++i) {
      if (options.sources(i).ll().lat() != options.targets(i).ll().lat() ||
          options.sources(i).ll().lng() != options.targets(i).ll().lng()) {
        optimize = false;
        break;
      }
    }
  } else {
    optimize = false;
  }
  if (optimize) {
    LOG_INFO("Find the optimal path");
  }

  // Timing with CostMatrix
  std::vector<TimeDistance> res;
  CostMatrix matrix;
  t0 = std::chrono::high_resolution_clock::now();
  for (uint32_t n = 0; n < iterations; n++) {
    res.clear();
    res = matrix.SourceToTarget(options.sources(), options.targets(), reader, mode_costing, mode,
                                max_distance);
    matrix.clear();
  }
  t1 = std::chrono::high_resolution_clock::now();
  ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  float avg = (static_cast<float>(ms) / static_cast<float>(iterations)) * 0.001f;
  LOG_INFO("CostMatrix average time to compute: " + std::to_string(avg) + " sec");
  LogResults(optimize, options, res);

  // Run with TimeDistanceMatrix
  TimeDistanceMatrix tdm;
  for (uint32_t n = 0; n < iterations; n++) {
    res.clear();
    res = tdm.SourceToTarget(options.sources(), options.targets(), reader, mode_costing, mode,
                             max_distance);
    tdm.clear();
  }
  t1 = std::chrono::high_resolution_clock::now();
  ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  avg = (static_cast<float>(ms) / static_cast<float>(iterations)) * 0.001f;
  LOG_INFO("TimeDistanceMatrix average time to compute: " + std::to_string(avg) + " sec");
  LogResults(optimize, options, res);

  // Shutdown protocol buffer library
  google::protobuf::ShutdownProtobufLibrary();

  return EXIT_SUCCESS;
}
