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
#include <valhalla/thor/pathalgorithm.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/trippathbuilder.h>
#include <valhalla/thor/isochrone.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::odin;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace bpo = boost::program_options;

// Returns the costing method (created from the dynamic cost factory).
// Get the costing options. Get the base options from the config and the
// options for the specified costing method. Merge in any request costing
// options that override those in the config.
valhalla::sif::cost_ptr_t get_costing(CostFactory<DynamicCost> factory,
                                      boost::property_tree::ptree& config,
                                      boost::property_tree::ptree& request,
                                      const std::string& costing) {
 std::string method_options = "costing_options." + costing;
 auto config_costing = config.get_child_optional(method_options);
 if (!config_costing)
   throw std::runtime_error("No costing method found for '" + costing + "'");
 auto request_costing = request.get_child_optional(method_options);
 if (request_costing) {
   // If the request has any options for this costing type, merge the 2
   // costing options - override any config options that are in the request.
   // and add any request options not in the config.
   for (const auto& r : *request_costing) {
     config_costing->put_child(r.first, r.second);
   }
 }
 return factory.Create(costing, *config_costing);
}

// Main method for testing a single path
int main(int argc, char *argv[]) {
  bpo::options_description options("valhalla_run_isochrone " VERSION "\n"
  "\n"
  " Usage: valhalla_run_isochrone [options]\n"
  "\n"
  "valhalla_run_isochrone is a simple command line test tool for generating an isochrone. "
  "\n"
  "Use the -o option OR the -j option for specifying the origin location. "
  "\n"
  "\n");

  std::string origin, routetype, json, config;
  options.add_options()("help,h", "Print this help message.")(
      "version,v", "Print the version of this software.")(
      "origin,o",boost::program_options::value<std::string>(&origin),
      "Origin: lat,lng,[through|stop],[name],[street],[city/town/village],[state/province/canton/district/region/department...],[zip code],[country].")(
      "type,t", boost::program_options::value<std::string>(&routetype),
      "Route Type: auto|bicycle|pedestrian|auto-shorter")(
      "json,j",
      boost::program_options::value<std::string>(&json),
      "JSON Example: '{\"locations\":[{\"lat\":40.748174,\"lon\":-73.984984,\"type\":\"break\",\"heading\":200,\"name\":\"Empire State Building\",\"street\":\"350 5th Avenue\",\"city\":\"New York\",\"state\":\"NY\",\"postal_code\":\"10118-0110\",\"country\":\"US\"},{\"lat\":40.749231,\"lon\":-73.968703,\"type\":\"break\",\"name\":\"United Nations Headquarters\",\"street\":\"405 East 42nd Street\",\"city\":\"New York\",\"state\":\"NY\",\"postal_code\":\"10017-3507\",\"country\":\"US\"}],\"costing\":\"auto\",\"directions_options\":{\"units\":\"miles\"}}'")
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
    std::cout << "valhalla_run_isochrone " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  // Locations
  std::vector<Location> locations;

  // argument checking and verification
  boost::property_tree::ptree json_ptree;
  if (vm.count("json") == 0) {
    for (auto arg : std::vector<std::string> { "origin", "type", "config" }) {
      if (vm.count(arg) == 0) {
        std::cerr
            << "The <"
            << arg
            << "> argument was not provided, but is mandatory when json is not provided\n\n";
        std::cerr << options << "\n";
        return EXIT_FAILURE;
      }
    }
    locations.push_back(Location::FromCsv(origin));
  }
  ////////////////////////////////////////////////////////////////////////////
  // Process json input
  else {
    std::stringstream stream;
    stream << json;
    boost::property_tree::read_json(stream, json_ptree);

    try {
      for (const auto& location : json_ptree.get_child("locations"))
        locations.emplace_back(std::move(Location::FromPtree(location.second)));
      if (locations.size() != 1)
        throw;
    } catch (...) {
      throw std::runtime_error("Requires a single location");
    }

    // Parse out the type of route - this provides the costing method to use
    std::string costing;
    try {
      routetype = json_ptree.get<std::string>("costing");
    } catch (...) {
      throw std::runtime_error("No edge/node costing provided");
    }
  }

  //parse the config
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config.c_str(), pt);

  //configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree = pt
      .get_child_optional("thor.logging");
  if (logging_subtree) {
    auto logging_config = valhalla::midgard::ToMap<
        const boost::property_tree::ptree&,
        std::unordered_map<std::string, std::string> >(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  // Get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir"));

  // Construct costing
  CostFactory<DynamicCost> factory;
  factory.Register("auto", CreateAutoCost);
  factory.Register("auto_shorter", CreateAutoShorterCost);
  factory.Register("bus", CreateBusCost);
  factory.Register("bicycle", CreateBicycleCost);
  factory.Register("pedestrian", CreatePedestrianCost);
  factory.Register("truck", CreateTruckCost);
  factory.Register("transit", CreateTransitCost);

  // Figure out the route type
  for (auto & c : routetype)
    c = std::tolower(c);
  LOG_INFO("routetype: " + routetype);

  // Get the costing method - pass the JSON configuration
  TripPath trip_path;
  TravelMode mode;
  std::shared_ptr<DynamicCost> mode_costing[4];
  if (routetype == "multimodal") {
    // Create array of costing methods per mode and set initial mode to
    // pedestrian
    mode_costing[0] = get_costing(factory, pt, json_ptree, "auto");
    mode_costing[1] = get_costing(factory, pt, json_ptree, "pedestrian");
    mode_costing[2] = get_costing(factory, pt, json_ptree, "bicycle");
    mode_costing[3] = get_costing(factory, pt, json_ptree, "transit");
    mode = TravelMode::kPedestrian;
  } else {
    // Assign costing method, override any config options that are in the
    // json request
    std::shared_ptr<DynamicCost> cost = get_costing(factory, pt,
                              json_ptree, routetype);
    mode = cost->travelmode();
    mode_costing[static_cast<uint32_t>(mode)] = cost;
  }

  // Find locations
  std::shared_ptr<DynamicCost> cost = mode_costing[static_cast<uint32_t>(mode)];
  auto getPathLoc = [&reader, &cost] (Location& loc, std::string status) {
    try {
      return Search(loc, reader, cost->GetEdgeFilter(), cost->GetNodeFilter());
    } catch (...) {
      exit(EXIT_FAILURE);
    }
  };
  std::vector<PathLocation> path_location;
  for (auto loc : locations) {
    path_location.push_back(getPathLoc(loc, "fail_invalid_origin"));
  }

  // Test - compute an isochrone. 2 hours
  uint32_t max_seconds = 3600;
  auto t1 = std::chrono::high_resolution_clock::now();
  Isochrone isochrone;
  auto isotile = isochrone.Compute(path_location[0], max_seconds, reader, mode_costing, mode);
  auto geojson = isotile->GenerateContourGeoJson({300, 600, 900, 1200, 1500, 1800, 2100, 2400, 2700, 3000, 3300, 3600, 3900, 4200, 4500, 4800, 5100, 5400});
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("Isochrone took " + std::to_string(msecs) + " ms");

  int nv = 0;
  auto& secs = isotile->data();
  for (auto sec : secs) {
    if (sec < max_seconds) {
      nv++;
    }
  }
  LOG_INFO("Marked " + std::to_string(nv) + " cells in the isotile" + " size= " + std::to_string(secs.size()));

  LOG_INFO("Geojson: " + geojson);
  return EXIT_SUCCESS;
}

