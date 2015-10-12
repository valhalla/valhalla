#include <iostream>
#include <string>
#include <vector>
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
#include <valhalla/midgard/logging.h>

#include "thor/timedistancematrix.h"
#include "thor/optimizer.h"

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
  uint32_t hours   =  secs / 3600;
  uint32_t minutes = (secs / 60) % 60;
  uint32_t seconds =  secs % 60;
  return std::to_string(hours) + ":" + std::to_string(minutes) + ":" + std::to_string(seconds);
}

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

// Main method for testing time and distance matrix methods
int main(int argc, char *argv[]) {
  bpo::options_description options("timedistance_test " VERSION "\n"
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

  options.add_options()("help,h", "Print this help message.")(
      "version,v", "Print the version of this software.")(
      "type,t", boost::program_options::value<std::string>(&routetype),
           "Route Type: auto|bicycle|pedestrian|auto-shorter")(
      "matrixtype,m", boost::program_options::value<std::string>(&matrixtype),
               "Matrix Type: one_to_many|many_to_many|many_to_one")(
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
            pos_options).run(), vm);
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
    mode_costing[0] = get_costing(factory, pt, json_ptree, "auto");
    mode_costing[1] = get_costing(factory, pt, json_ptree, "pedestrian");
    mode_costing[2] = get_costing(factory, pt, json_ptree, "bicycle");
    mode_costing[3] = get_costing(factory, pt, json_ptree, "transit");
    mode = TravelMode::kPedestrian;
  } else {
    // Assign costing method
    std::shared_ptr<DynamicCost> cost = get_costing(factory, pt,
                                  json_ptree, routetype);
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

  // Get path locations (Loki) for all locations.
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
  std::vector<TimeDistance> res;
  for (uint32_t n = 0; n < iterations; n++) {
    res.clear();
    if (matrixtype == "one_to_many") {
      // First location in the list is the origin
      res = tdm.OneToMany(0, path_locations, reader, mode_costing, mode);
    } else if (matrixtype == "many_to_many") {
      res = tdm.ManyToMany(path_locations, reader, mode_costing, mode);
    } else {
      // Last location in the list is the destination of many to one
      uint32_t idx = path_locations.size() - 1;
      res = tdm.ManyToOne(idx, path_locations, reader, mode_costing, mode);
    }
    tdm.Clear();
  }
  t1 = std::chrono::high_resolution_clock::now();
  ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
  float avg = (static_cast<float>(ms) / static_cast<float>(iterations)) * 0.001f;
  LOG_INFO("TDMatrix average time to compute: " + std::to_string(avg) + " sec");

  // Log results
  LOG_INFO("Results:");
  if (matrixtype == "many_to_many") {
    uint32_t idx1 = 0;
    uint32_t idx2 = 0;
    uint32_t nlocs = path_locations.size();
    for (auto& td : res) {
      LOG_INFO(std::to_string(idx1) + "," + std::to_string(idx2) +
          ": Distance= " + std::to_string(td.dist) +
          " Time= " + GetFormattedTime(td.time) + " secs = " + std::to_string(td.time));
      idx2++;
      if (idx2 == nlocs) {
        idx2 = 0;
        idx1++;
      }
    }

    // Optimize the path
    std::vector<float> costs;
    for (auto& td : res) {
      costs.push_back(static_cast<float>(td.time));
    }

    Optimizer opt;
    auto tour = opt.Solve(nlocs, costs);
    LOG_INFO("Optimal Tour:");
    for (auto& loc : tour) {
      LOG_INFO("   : " + std::to_string(loc));
    }
  } else {
    uint32_t idx = 0;
    for (auto& td : res) {
      LOG_INFO(std::to_string(idx) + ": Distance= " + std::to_string(td.dist) +
          " Time= " + GetFormattedTime(td.time) + " secs = " + std::to_string(td.time));
      idx++;
    }
  }
  return EXIT_SUCCESS;
}

