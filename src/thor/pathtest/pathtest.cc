#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "config.h"

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/loki/search.h>
#include <valhalla/odin/narrativebuilder.h>
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
int PathTest(GraphReader& reader, const PathLocation& origin,
             const PathLocation& dest, std::string routetype,
             TripPath& trip_path) {
  // Register costing methods
  CostFactory<DynamicCost> factory;
  factory.Register("auto", CreateAutoCost);
  factory.Register("bicycle", CreateBicycleCost);
  factory.Register("pedestrian", CreatePedestrianCost);

  for (auto & c : routetype)
    c = std::tolower(c);
  std::shared_ptr<DynamicCost> cost = factory.Create(routetype);

  std::cout << "routetype: " << routetype << std::endl;

  std::clock_t start = std::clock();
  PathAlgorithm pathalgorithm;
  uint32_t msecs = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
  std::cout << "PathAlgorithm Construction took " << msecs << " ms"
            << std::endl;
  start = std::clock();
  std::vector<GraphId> pathedges;
  pathedges = pathalgorithm.GetBestPath(origin, dest, reader, cost);
  msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  std::cout << "PathAlgorithm GetBestPath took " << msecs << " ms" << std::endl;

  // Form output information based on pathedges
  start = std::clock();
  TripPathBuilder trippathbuilder;

  trippathbuilder.Build(reader, pathedges, trip_path);

  // TODO - perhaps walk the edges to find total length?
//  std::cout << "Trip length is: " << trip_path.length << " km" << std::endl;
  msecs = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
  std::cout << "TripPathBuilder took " << msecs << " ms" << std::endl;

  start = std::clock();
  pathalgorithm.Clear();
  msecs = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
  std::cout << "PathAlgorithm Clear took " << msecs << " ms" << std::endl;
  return 0;
}

void NarrativeTest(TripPath& trip_path) {
  NarrativeBuilder nb(trip_path);
  nb.Build();
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
  std::cout << "Location Processing took " << msecs << " ms" << std::endl;

  TripPath trip_path;
  // TODO - set locations

  // Try the route
  PathTest(reader, pathOrigin, pathDest, routetype, trip_path);

  // Try the the narrative
  NarrativeTest(trip_path);

  return EXIT_SUCCESS;
}

