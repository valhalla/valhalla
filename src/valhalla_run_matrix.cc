#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "config.h"

#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "loki/search.h"
#include "midgard/logging.h"
#include "odin/directionsbuilder.h"
#include "odin/util.h"
#include "sif/costfactory.h"
#include "thor/costmatrix.h"
#include "thor/optimizer.h"
#include "thor/timedistancematrix.h"
#include "worker.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace bpo = boost::program_options;

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

float random_unit_float() {
  // Create a random integer between -1000 - +1000,
  // then scale to be between -1 and 1
  int r = rand() % 2000 - 1000;
  return static_cast<float>(r) * 0.001f;
}

// Jitter lat,lng by up to about 1km
PointLL JitterLatLng(const PointLL& latlng, const float delta) {
  return {latlng.lng() + random_unit_float() * delta, latlng.lat() + random_unit_float() * delta};
}

// Log results
void LogResults(const std::string& matrixtype,
                const std::vector<PathLocation>& path_locations,
                const std::vector<TimeDistance>& res) {
  LOG_INFO("Results:");
  if (matrixtype == "many_to_many") {
    uint32_t idx1 = 0;
    uint32_t idx2 = 0;
    uint32_t nlocs = path_locations.size();
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

  } else {
    uint32_t idx = 0;
    for (auto& td : res) {
      LOG_INFO(std::to_string(idx) + ": Distance= " + std::to_string(td.dist) +
               " Time= " + GetFormattedTime(td.time) + " secs = " + std::to_string(td.time));
      idx++;
    }
  }
}

// Main method for testing time and distance matrix methods
int main(int argc, char* argv[]) {
  bpo::options_description options(
      "timedistance_test " VALHALLA_VERSION "\n"
      "\n"
      " Usage: timedistance_test [options]\n"
      "\n"
      "timedistance_test is a command line test tool for time+distance matrix routing. "
      "\n"
      "Use the -j option for specifying the locations. "
      "\n"
      "\n");

  std::string routetype, json, config;
  std::string matrixtype = "one_to_many";
  uint32_t iterations = 1;

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "type,t", boost::program_options::value<std::string>(&routetype),
      "Route Type: auto|bicycle|pedestrian|auto-shorter")(
      "matrixtype,m", boost::program_options::value<std::string>(&matrixtype),
      "Matrix Type: one_to_many|many_to_many|many_to_one")(
      "json,j", boost::program_options::value<std::string>(&json),
      "JSON Example: "
      "'{\"locations\":[{\"lat\":40.748174,\"lon\":-73.984984,\"type\":\"break\",\"heading\":200,"
      "\"name\":\"Empire State Building\",\"street\":\"350 5th Avenue\",\"city\":\"New "
      "York\",\"state\":\"NY\",\"postal_code\":\"10118-0110\",\"country\":\"US\"},{\"lat\":40."
      "749231,\"lon\":-73.968703,\"type\":\"break\",\"name\":\"United Nations "
      "Headquarters\",\"street\":\"405 East 42nd Street\",\"city\":\"New "
      "York\",\"state\":\"NY\",\"postal_code\":\"10017-3507\",\"country\":\"US\"}],\"costing\":"
      "\"auto\",\"directions_options\":{\"units\":\"miles\"}}'")(
      "multi-run", bpo::value<uint32_t>(&iterations),
      "Generate the route N additional times before exiting.")
      // positional arguments
      ("config", bpo::value<std::string>(&config), "Valhalla configuration file");

  bpo::positional_options_description pos_options;
  pos_options.add("config", 1);
  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(),
               vm);
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
    std::cout << "timedistance_test " << VALHALLA_VERSION << "\n";
    return EXIT_SUCCESS;
  }

  valhalla::valhalla_request_t request;
  request.parse(json, valhalla::odin::DirectionsOptions::sources_to_targets);

  auto locations = PathLocation::fromPBF(request.options.locations());
  if (locations.size() == 0) {
    throw std::runtime_error("Request requires 1 or more locations");
  }

  // We require JSON input of locations (unlike pathtest). The first location
  // is the origin.
  // std::stringstream stream;
  // stream << json;
  // boost::property_tree::ptree json_ptree;
  // boost::property_tree::read_json(stream, json_ptree);

  // Get type of route - this provides the costing method to use
  try {
    routetype = valhalla::odin::Costing_Name(request.options.costing());
  } catch (...) { throw std::runtime_error("No edge/node costing provided"); }

  // parse the config
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config.c_str(), pt);

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
  CostFactory<DynamicCost> factory;
  factory.RegisterStandardCostingModels();

  // Figure out the route type
  for (auto& c : routetype) {
    c = std::tolower(c);
  }
  LOG_INFO("routetype: " + routetype);

  // Get the costing method - pass the JSON configuration
  TravelMode mode;
  std::shared_ptr<DynamicCost> mode_costing[4];
  if (routetype == "multimodal") {
    // Create array of costing methods per mode and set initial mode to
    // pedestrian
    mode_costing[0] = factory.Create(valhalla::odin::Costing::auto_, request.options);
    mode_costing[1] = factory.Create(valhalla::odin::Costing::pedestrian, request.options);
    mode_costing[2] = factory.Create(valhalla::odin::Costing::bicycle, request.options);
    mode_costing[3] = factory.Create(valhalla::odin::Costing::transit, request.options);
    mode = TravelMode::kPedestrian;
  } else {
    // Assign costing method
    std::shared_ptr<DynamicCost> cost = factory.Create(request.options.costing(), request.options);
    mode = cost->travel_mode();
    mode_costing[static_cast<uint32_t>(mode)] = cost;
  }

  // If only one location is provided we create a set of random locations
  // around this location
  if (locations.size() == 1) {
    LOG_INFO("Create random locations");
    PointLL ll = locations.front().latlng_;
    LOG_INFO("Location 0 = " + std::to_string(ll.lat()) + "," + std::to_string(ll.lng()));
    uint32_t n = 50;
    float delta = 0.15f; // Should keep all locations inside a 35 mile radius
    for (uint32_t i = 0; i < n; i++) {
      PointLL ll2 = JitterLatLng(ll, delta);
      locations.push_back(Location(ll2));
      LOG_INFO("Location " + std::to_string(i + 1) + " = " + std::to_string(ll2.lat()) + "," +
               std::to_string(ll2.lng()));
    }
  } else if (locations.size() == 0) {
    LOG_ERROR("No locations provided");
    exit(EXIT_FAILURE);
  }

  // Get path locations (Loki) for all locations.
  auto t0 = std::chrono::high_resolution_clock::now();
  std::shared_ptr<DynamicCost> cost = mode_costing[static_cast<uint32_t>(mode)];
  const auto projections = Search(locations, reader, cost->GetEdgeFilter(), cost->GetNodeFilter());
  std::vector<PathLocation> path_locations;
  valhalla::odin::DirectionsOptions directions_options;
  for (auto& loc : locations) {
    try {
      path_locations.push_back(projections.at(loc));
      PathLocation::toPBF(path_locations.back(), directions_options.mutable_locations()->Add(),
                          reader);
    } catch (...) { exit(EXIT_FAILURE); }
  }
  if (matrixtype == "one_to_many") {
    directions_options.mutable_sources()->Add()->CopyFrom(*directions_options.locations().begin());
    directions_options.mutable_targets()->CopyFrom(directions_options.locations());
  } else if (matrixtype == "many_to_many") {
    directions_options.mutable_sources()->CopyFrom(directions_options.locations());
    directions_options.mutable_targets()->CopyFrom(directions_options.locations());
  } else {
    directions_options.mutable_sources()->CopyFrom(directions_options.locations());
    directions_options.mutable_targets()->Add()->CopyFrom(*directions_options.locations().rbegin());
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  uint32_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  LOG_INFO("Location Processing took " + std::to_string(ms) + " ms");

  // Get the max matrix distances for construction of the CostMatrix and TimeDistanceMatrix classes
  std::unordered_map<std::string, float> max_matrix_distance;
  for (const auto& kv : pt.get_child("service_limits")) {
    if (kv.first == "max_avoid_locations" || kv.first == "max_reachability" ||
        kv.first == "max_radius") {
      continue;
    }
    if (kv.first != "skadi" && kv.first != "trace" && kv.first != "isochrone") {
      max_matrix_distance.emplace(kv.first, pt.get<float>("service_limits." + kv.first +
                                                          ".max_matrix_distance"));
    }
  }

  if (max_matrix_distance.empty()) {
    throw std::runtime_error("Missing max_matrix_distance configuration");
  }

  // Compute the cost matrix
  t0 = std::chrono::high_resolution_clock::now();

  // Timing with CostMatrix
  std::vector<TimeDistance> res;
  for (uint32_t n = 0; n < iterations; n++) {
    res.clear();
    CostMatrix matrix;
    res = matrix.SourceToTarget(directions_options.sources(), directions_options.targets(), reader,
                                mode_costing, mode, max_matrix_distance.find(routetype)->second);
  }
  t1 = std::chrono::high_resolution_clock::now();
  ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  float avg = (static_cast<float>(ms) / static_cast<float>(iterations)) * 0.001f;
  LOG_INFO("CostMatrix average time to compute: " + std::to_string(avg) + " sec");
  LogResults(matrixtype, path_locations, res);

  // Run with TimeDistanceMatrix
  for (uint32_t n = 0; n < iterations; n++) {
    res.clear();
    TimeDistanceMatrix tdm;
    if (matrixtype == "one_to_many") {
      res = tdm.OneToMany(*directions_options.locations().begin(), directions_options.locations(),
                          reader, mode_costing, mode, max_matrix_distance.find(routetype)->second);
    } else if (matrixtype == "many_to_many") {
      res = tdm.ManyToOne(*directions_options.locations().rbegin(), directions_options.locations(),
                          reader, mode_costing, mode, max_matrix_distance.find(routetype)->second);
    } else {
      res = tdm.ManyToMany(directions_options.locations(), reader, mode_costing, mode,
                           max_matrix_distance.find(routetype)->second);
    }
  }
  t1 = std::chrono::high_resolution_clock::now();
  ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  avg = (static_cast<float>(ms) / static_cast<float>(iterations)) * 0.001f;
  LOG_INFO("TimeDistanceMatrix average time to compute: " + std::to_string(avg) + " sec");
  LogResults(matrixtype, path_locations, res);

  return EXIT_SUCCESS;
}
