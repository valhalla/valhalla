#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <boost/program_options.hpp>

#include "config.h"

#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "thor/edgecost.h"
#include "thor/pathalgorithm.h"
#include "thor/trippathbuilder.h"

using namespace valhalla::geo;
using namespace valhalla::baldr;
using namespace valhalla::thor;

namespace bpo = boost::program_options;

/**
 *
 */
int PathTest(const std::string& datapath, const PathLocation& origin,
             const PathLocation& dest) {
  // Use Loki to get location information
  std::clock_t start = std::clock();

  unsigned int msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
//  std::cout << "Location Processing took " << msecs << " ms" << std::endl;

  start = std::clock();
  PathAlgorithm pathalgorithm;
  GraphReader graphreader(datapath);
  EdgeCost* edgecost = new EdgeCost;
  std::vector<GraphId> pathedges;
  pathedges = pathalgorithm.GetBestPath(origin, dest, graphreader, edgecost);
  msecs = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
  std::cout << "PathAlgorithm GetBestPath took " << msecs << " ms" << std::endl;

  // Form output information based on pathedges
  start = std::clock();
  TripPathBuilder trippath;
  float length = trippath.Build(graphreader, pathedges);
  std::cout << "Trip length is: " << length << " km" << std::endl;
  std::cout << "TripPathBuilder took " << msecs << " ms" << std::endl;

  start = std::clock();
  pathalgorithm.Clear();
  std::cout << "PathAlgorithm Clear took " << msecs << " ms" << std::endl;
  return 0;
}

// Main method for testing a single path
int main(int argc, char *argv[]) {

  bpo::options_description options(
  "pathtest " VERSION "\n"
  "\n"
  " Usage: pathtest [options]\n"
  "\n"
  "pathtest is a simple command line test tool for shortest path routing. "
  "\n"
  "\n");

  std::string echo;

  options.add_options()
    ("help,h", "Print this help message.")
    ("version,v", "Print the version of this software.")
    ("origin,o", "Origin lat,lng.")
    // positional arguments
    ("echo", bpo::value<std::string>(&echo), "String for the application to echo back")
    ;


  bpo::positional_options_description pos_options;
  pos_options.add("echo", 1);

  bpo::variables_map vm;

  try {
    bpo::store(bpo::command_line_parser(argc,argv)
      .options(options)
      .positional(pos_options)
      .run(),
      vm);
    bpo::notify(vm);

  } catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n" << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
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
  for (auto arg : {"echo"}) {
    if (vm.count(arg) == 0) {
      std::cerr << "The <" << arg << "> argument was not provided, but is mandatory\n\n";
      std::cerr << options << "\n";
      return EXIT_FAILURE;
    }
  }

  //echo it back
  std::cout << echo << "\n";

  // Test path
  std::string datapath = "/data/valhalla/";

  // Origin
  Location originloc(PointLL(40.042587f, -76.299248f), Location::StopType::BREAK);
  PathLocation origin(originloc);
  PathLocation::Edge originedge;
  originedge.dist_ = 0.0f;
  originedge.id_ = GraphId(749214, 2, 23460);
  origin.edges_.push_back(originedge);

  // Destination
  Location destloc(PointLL(40.039505f, -76.305596f), Location::StopType::BREAK);
  PathLocation dest(destloc);
  PathLocation::Edge destedge;
  destedge.dist_ = 0.0f;
  destedge.id_ = GraphId(749214, 2, 157);
  dest.edges_.push_back(destedge);

  PathTest(datapath, origin, dest);

  return EXIT_SUCCESS;
}

