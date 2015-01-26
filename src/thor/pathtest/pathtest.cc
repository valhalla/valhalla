#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>

#include "config.h"

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/loki/search.h>
#include <valhalla/odin/directionsbuilder.h>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/midgard/logging.h>
#include "thor/pathalgorithm.h"
#include "thor/costfactory.h"
#include "thor/trippathbuilder.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::thor;

namespace bpo = boost::program_options;

/**
 * TODO: add locations to TripPath
 */
TripPath PathTest(GraphReader& reader, const PathLocation& origin,
                  const PathLocation& dest, std::string routetype) {
  // Register costing methods
  CostFactory<DynamicCost> factory;
  factory.Register("auto", CreateAutoCost);
  factory.Register("bicycle", CreateBicycleCost);
  factory.Register("pedestrian", CreatePedestrianCost);

  for (auto & c : routetype)
    c = std::tolower(c);
  std::shared_ptr<DynamicCost> cost = factory.Create(routetype);

  LOG_INFO("routetype: " + routetype);

  std::clock_t start = std::clock();
  PathAlgorithm pathalgorithm;
  uint32_t msecs = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
  LOG_INFO("PathAlgorithm Construction took " + std::to_string(msecs) + " ms");
  start = std::clock();
  std::vector<GraphId> pathedges;
  pathedges = pathalgorithm.GetBestPath(origin, dest, reader, cost);
  msecs = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
  LOG_INFO("PathAlgorithm GetBestPath took " + std::to_string(msecs) + " ms");

  // Form output information based on pathedges
  start = std::clock();
  TripPath trip_path = TripPathBuilder::Build(reader, pathedges);

  // TODO - perhaps walk the edges to find total length?
  //LOG_INFO("Trip length is: " + std::to_string(trip_path.length) + " km");
  msecs = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
  LOG_INFO("TripPathBuilder took " + std::to_string(msecs) + " ms");

  start = std::clock();
  pathalgorithm.Clear();
  msecs = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
  LOG_INFO("PathAlgorithm Clear took " + std::to_string(msecs) + " ms");
  return trip_path;
}

TripDirections DirectionsTest(TripPath& trip_path) {
  DirectionsBuilder directions;
  TripDirections trip_directions = directions.Build(trip_path);
  float totalDistance = 0.0f;
  int m = 1;
  for (const auto& maneuver : trip_directions.maneuver()) {
    LOG_INFO("----------------------------------------------");
    LOG_INFO(std::to_string(m++) + ": " + maneuver.text_instruction() + " | " + std::to_string(maneuver.length()) + " km");
    totalDistance += maneuver.length();
  }
  LOG_INFO("==============================================");
  LOG_INFO("Total distance: " + std::to_string(totalDistance) + " km");

  return trip_directions;
}

// Main method for testing a single path
int main(int argc, char *argv[]) {
  bpo::options_description options("pathtest " VERSION "\n"
  "\n"
  " Usage: pathtest [options]\n"
  "\n"
  "pathtest is a simple command line test tool for shortest path routing. "
  "\n"
  "\n");

  std::string origin, destination, routetype, config;

  options.add_options()("help,h", "Print this help message.")(
      "version,v", "Print the version of this software.")(
      "origin,o",
      boost::program_options::value<std::string>(&origin)->required(),
      "Origin lat,lng.")(
      "destination,d",
      boost::program_options::value<std::string>(&destination)->required(),
      "Destination lat,lng.")(
      "route_type,t",
      boost::program_options::value<std::string>(&routetype)->required(),
      "Route Type: auto|bicycle|pedestrian")
  // positional arguments
  ("config", bpo::value<std::string>(&config)->required(),
   "String for the application to echo back");

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
    std::cout << "pathtest " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  // argument checking and verification
  for (auto arg : std::vector<std::string> { "origin", "destination",
      "route_type", "config" }) {
    if (vm.count(arg) == 0) {
      std::cerr << "The <" << arg
                << "> argument was not provided, but is mandatory\n\n";
      std::cerr << options << "\n";
      return EXIT_FAILURE;
    }
  }

  //parse the config
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config.c_str(), pt);

  //configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree = pt.get_child_optional("logging.thor");
  if(logging_subtree) {
    auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&, std::unordered_map<std::string, std::string> >(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  valhalla::baldr::GraphReader reader(pt);

  // Use Loki to get location information
  std::clock_t start = std::clock();

  // Origin
  Location originloc = Location::FromCsv(origin);
  PathLocation pathOrigin = valhalla::loki::Search(originloc, reader);

  // Destination
  Location destloc = Location::FromCsv(destination);
  PathLocation pathDest = valhalla::loki::Search(destloc, reader);

  uint32_t msecs = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
  LOG_INFO("Location Processing took " + std::to_string(msecs) + " ms");

  // TODO - set locations

  // Try the route
  TripPath trip_path = PathTest(reader, pathOrigin, pathDest, routetype);

  // Try the the directions
  TripDirections trip_directions = DirectionsTest(trip_path);

  return EXIT_SUCCESS;
}

