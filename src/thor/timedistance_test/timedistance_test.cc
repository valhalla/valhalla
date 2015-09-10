#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <tuple>
#include <cmath>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>
#include <boost/format.hpp>

#include "config.h"

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/loki/search.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/odin/directionsbuilder.h>
#include <valhalla/odin/util.h>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/distanceapproximator.h>

#include "thor/timedistancematrix.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace bpo = boost::program_options;


std::string GetFormattedTime(uint32_t seconds) {
  if (seconds == 0) {
    return "0";
  }
  uint32_t hours = (uint32_t) seconds / 3600;
  uint32_t minutes = ((uint32_t) (seconds / 60)) % 60;
  std::string formattedTime = "";
  // Hours
  if (hours > 0) {
    formattedTime += std::to_string(hours);
    formattedTime += (hours == 1) ? " hour" : " hours";
    if (minutes > 0) {
      formattedTime += ", ";
    }
  }
  // Minutes
  if (minutes > 0) {
    formattedTime += std::to_string(minutes);
    formattedTime += (minutes == 1) ? " minute" : " minutes";
  }
  return formattedTime;
}

// Main method for testing time and distance matrix methods
int main(int argc, char *argv[]) {
  bpo::options_description options("timedistance_test " VERSION "\n"
  "\n"
  " Usage: timedistance_test [options]\n"
  "\n"
  "timedistance_test is a simple command line test tool for shortest path routing. "
  "\n"
  "Use the -o and -d options OR the -j option for specifying the locations. "
  "\n"
  "\n");

  std::string origin, destination, routetype, json, config;
  bool multi_run = false;
  uint32_t iterations;

  options.add_options()("help,h", "Print this help message.")(
      "version,v", "Print the version of this software.")(
      "origin,o",
      boost::program_options::value<std::string>(&origin),
      "Origin: lat,lng,[through|stop],[name],[street],[city/town/village],[state/province/canton/district/region/department...],[zip code],[country].")(
      "destination,d",
      boost::program_options::value<std::string>(&destination),
      "Destination: lat,lng,[through|stop],[name],[street],[city/town/village],[state/province/canton/district/region/department...],[zip code],[country].")(
      "type,t", boost::program_options::value<std::string>(&routetype),
      "Route Type: auto|bicycle|pedestrian|auto-shorter")(
      "json,j",
      boost::program_options::value<std::string>(&json),
      "JSON Example: '{\"locations\":[{\"lat\":40.748174,\"lon\":-73.984984,\"type\":\"break\",\"heading\":200,\"name\":\"Empire State Building\",\"street\":\"350 5th Avenue\",\"city\":\"New York\",\"state\":\"NY\",\"postal_code\":\"10118-0110\",\"country\":\"US\"},{\"lat\":40.749231,\"lon\":-73.968703,\"type\":\"break\",\"name\":\"United Nations Headquarters\",\"street\":\"405 East 42nd Street\",\"city\":\"New York\",\"state\":\"NY\",\"postal_code\":\"10017-3507\",\"country\":\"US\"}],\"costing\":\"auto\",\"directions_options\":{\"units\":\"miles\"}}'")
      ("multi-run", bpo::value<uint32_t>(&iterations), "Generate the route N additional times before exiting.")
      // positional arguments
      ("config", bpo::value<std::string>(&config), "Valhalla configuration file");


  bpo::positional_options_description pos_options;
  pos_options.add("config", 1);

  bpo::variables_map vm;

  try {
    bpo::store(
        bpo::command_line_parser(argc, argv).options(options).positional(
            pos_options).run(),
        vm);
    bpo::notify(vm);

  } catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what()
              << "\n" << "This is a bug, please report it at " PACKAGE_BUGREPORT
              << "\n";
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }

  if (vm.count("version")) {
    std::cout << "timedistance_test " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  if (vm.count("multi-run")) {
    multi_run = true;
  }

  // We require JSON input of locations (unlike pathtest). The first location
  // is the origin.
  std::stringstream stream;
  stream << json;
  boost::property_tree::ptree json_ptree;
  boost::property_tree::read_json(stream, json_ptree);
  std::vector<Location> locations;
  try {
    for (const auto& location : json_ptree.get_child("locations"))
      locations.emplace_back(std::move(Location::FromPtree(location.second)));
    if (locations.size() < 2)
      throw;
  } catch (...) {
    throw std::runtime_error(
        "insufficiently specified required parameter 'locations'");
  }

  // Parse out the type of route - this provides the costing method to use
  std::string costing;
  try {
    routetype = json_ptree.get<std::string>("costing");
  } catch (...) {
    throw std::runtime_error("No edge/node costing provided");
  }

  // parse the config
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config.c_str(), pt);

  // configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree = pt
      .get_child_optional("thor.logging");
  if (logging_subtree) {
    auto logging_config = valhalla::midgard::ToMap<
        const boost::property_tree::ptree&,
        std::unordered_map<std::string, std::string> >(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  // Get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir.hierarchy"));

  // Construct costing
  CostFactory<DynamicCost> factory;
  factory.Register("auto", CreateAutoCost);
  factory.Register("auto_shorter", CreateAutoShorterCost);
  factory.Register("bus", CreateBusCost);
  factory.Register("bicycle", CreateBicycleCost);
  factory.Register("pedestrian", CreatePedestrianCost);
  factory.Register("transit", CreateTransitCost);

  // Figure out the route type
  for (auto & c : routetype)
    c = std::tolower(c);
  LOG_INFO("routetype: " + routetype);

  // Get the costing method - pass the JSON configuration
  TravelMode mode;
  std::shared_ptr<DynamicCost> mode_costing[4];
  if (routetype == "multimodal") {
    // Create array of costing methods per mode and set initial mode to
    // pedestrian
    mode_costing[0] = factory.Create("auto",
                        pt.get_child("costing_options.auto"));
    mode_costing[1] = factory.Create("pedestrian",
                        pt.get_child("costing_options.pedestrian"));
    mode_costing[2] = factory.Create("bicycle",
                        pt.get_child("costing_options.bicycle"));
    mode_costing[3] = factory.Create("transit",
                        pt.get_child("costing_options.transit"));
    mode = TravelMode::kPedestrian;
  } else {
    // Assign costing method
    std::shared_ptr<DynamicCost> cost = factory.Create(
        routetype, pt.get_child("costing_options." + routetype));
    mode = cost->travelmode();
    mode_costing[static_cast<uint32_t>(mode)] = cost;
  }

  // lambda for getting path location (Loki search)
  std::shared_ptr<DynamicCost> cost = mode_costing[static_cast<uint32_t>(mode)];
  auto getPathLoc = [&reader, &cost] (Location& loc) {
    try {
      return Search(loc, reader, cost->GetFilter());
    } catch (...) {
      exit(EXIT_FAILURE);
    }
  };

  // Get path locations (Loki) for all locations. The first location in the
  // incoming list is the origin, the rest are destinations.
  auto t0 = std::chrono::high_resolution_clock::now();
  std::vector<PathLocation> path_locations;
  for (auto& loc : locations) {
    path_locations.push_back(getPathLoc(loc));
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  uint32_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
  LOG_INFO("Location Processing took " + std::to_string(ms) + " ms");

  // Compute the time+distance matrix (for now one to many).
  // TODO - add ManyToOne and some way of computing ManyToMany
  t0 = std::chrono::high_resolution_clock::now();
  TimeDistanceMatrix tdm;
  std::vector<TimeDistance> res = tdm.OneToMany(0, path_locations, reader,
                                               mode_costing, mode);
  t1 = std::chrono::high_resolution_clock::now();
  ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
  LOG_INFO("TDMatrix Processing took " + std::to_string(ms) + " ms");

  uint32_t idx = 0;
  for (auto& td : res) {
    LOG_INFO(std::to_string(idx) + ": Distance= " + std::to_string(td.dist) +
        " Time= " + GetFormattedTime(td.time) + " secs = " + std::to_string(td.time));
    idx++;
  }

  return EXIT_SUCCESS;
}

