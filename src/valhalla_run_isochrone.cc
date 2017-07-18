#include <cstdint>
#include <iostream>
#include <fstream>
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

#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "baldr/geojson.h"
#include "loki/search.h"
#include "sif/costfactory.h"
#include "odin/directionsbuilder.h"
#include "odin/util.h"
#include "proto/trippath.pb.h"
#include "proto/tripdirections.pb.h"
#include "proto/directions_options.pb.h"
#include "midgard/logging.h"
#include "midgard/distanceapproximator.h"
#include "thor/pathalgorithm.h"
#include "thor/bidirectional_astar.h"
#include "thor/trippathbuilder.h"
#include "thor/isochrone.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace bpo = boost::program_options;

// Returns the costing method (created from the dynamic cost factory).
// Get the costing options. Merge in any request costing options that
// override those in the config.
valhalla::sif::cost_ptr_t get_costing(CostFactory<DynamicCost> factory,
                                      boost::property_tree::ptree& request,
                                      const std::string& costing) {
  std::string method_options = "costing_options." + costing;
  auto costing_options = request.get_child(method_options, {});
  return factory.Create(costing, costing_options);
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

  bool reverse = false, polygons = false, show_locations = false;
  size_t n_contours = 4;
  unsigned int max_minutes = 60;
  std::string origin, routetype, json, config, filename;
  float denoise = 1.f;
  float generalize = kOptimalGeneralization;
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
      ("reverse,r", bpo::value<bool>(&reverse), "Reverse direction.")
      ("ncontours,n", bpo::value<size_t>(&n_contours), "Number of contours.")
      ("minutes,m", bpo::value<unsigned int>(&max_minutes), "Maximum minutes.")
      ("config,c", bpo::value<std::string>(&config), "Valhalla configuration file")
      ("file,f", bpo::value<std::string>(&filename), "Geojson output file name.")
      ("polygons,p", bpo::value<bool>(&polygons), "Return as polygons or lines.")
      ("show_locations,l", bpo::value<bool>(&show_locations), "Include locations in the final geojson.")
      ("denoise,d", bpo::value<float>(&denoise), "Denoise value. Must be between 0 and 1.")
      ("generalize,g", bpo::value<float>(&generalize), "Generalize value.");

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
  std::vector<Location> avoid_locations;

  // Isochrone parameters
  std::unordered_map<float, std::string> colors {};
  std::vector<float> contour_times;

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
    for (size_t i = 1; i <= n_contours; i++) {
      contour_times.push_back((max_minutes * i) / n_contours);
    }
  }
  ////////////////////////////////////////////////////////////////////////////
  // Process json input
  else {
    std::stringstream stream;
    stream << json;
    boost::property_tree::read_json(stream, json_ptree);

    if (vm.count("minutes")) {
      LOG_WARN ("minutes parameter is being overwritten by JSON contours");
    }

    if (vm.count("ncontours")) {
      LOG_WARN ("ncontours parameter is being overwritten by JSON contours");
    }

    try {
      for (const auto& location : json_ptree.get_child("locations")) {
        locations.emplace_back(std::move(Location::FromPtree(location.second)));
      }
    } catch (...) {
      throw std::runtime_error("Requires a single location");
    }

    // Process avoid locations
    try {
      for (const auto& location : json_ptree.get_child("avoid_locations")) {
        avoid_locations.emplace_back(std::move(Location::FromPtree(location.second)));
      }
    } catch (...) {
      LOG_INFO("No avoid locations");
    }

    // Parse out the type of route - this provides the costing method to use
    std::string costing;
    try {
      routetype = json_ptree.get<std::string>("costing");
    } catch (...) {
      throw std::runtime_error("No edge/node costing provided");
    }

    // Get denoise parameter
    try {
      denoise = json_ptree.get<float>("denoise");
      if (vm.count("denoise")) {
        LOG_WARN ("denoise parameter is being overwritten by JSON denoise parameter");
      }
    } catch(...){}

    // Get generalize parameter
    try {
      generalize = json_ptree.get<float>("generalize");
      if (vm.count("generalize")) {
        LOG_WARN ("generalize parameter is being overwritten by JSON generalize parameter");
      }
    } catch (...) {}

    // Get polygons
    try {
      polygons = json_ptree.get<bool>("polygons");
      if (vm.count("polygons")) {
        LOG_WARN ("polygons parameter is being overwritten by JSON polygons parameter");
      }
    } catch (...) {}

    // Get show_locations
    try {
      show_locations = json_ptree.get<bool>("show_locations");
      if (vm.count("show_locations")) {
        LOG_WARN ("show_locations parameter is being overwritten by JSON show_locations parameter");
      }
    } catch (...) {}

    // Get Contours
    try {
      for (const auto& contour : json_ptree.get_child("contours")) {
        contour_times.push_back(contour.second.get<float>("time"));
        colors[contour_times.back()] = contour.second.get<std::string>("color", "");
      }
    } catch (...) {
      throw std::runtime_error("Contours failed to parse.");
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
  valhalla::odin::TripPath trip_path;
  TravelMode mode;
  std::shared_ptr<DynamicCost> mode_costing[4];
  if (routetype == "multimodal") {
    // Create array of costing methods per mode and set initial mode to
    // pedestrian
    mode_costing[0] = get_costing(factory, json_ptree, "auto");
    mode_costing[1] = get_costing(factory, json_ptree, "pedestrian");
    mode_costing[2] = get_costing(factory, json_ptree, "bicycle");
    mode_costing[3] = get_costing(factory, json_ptree, "transit");
    mode = TravelMode::kPedestrian;
  } else {
    // Assign costing method, override any config options that are in the
    // json request
    std::shared_ptr<DynamicCost> cost = get_costing(factory,
                              json_ptree, routetype);
    mode = cost->travel_mode();
    mode_costing[static_cast<uint32_t>(mode)] = cost;
  }

  // Find locations
  std::shared_ptr<DynamicCost> cost = mode_costing[static_cast<uint32_t>(mode)];
  const auto projections = Search(locations, reader, cost->GetEdgeFilter(), cost->GetNodeFilter());
  std::vector<PathLocation> path_location;
  for (auto loc : locations) {
    try {
      path_location.push_back(projections.at(loc));
    } catch (...) {
      exit(EXIT_FAILURE);
    }
  }

  // Find avoid locations
  std::vector<GraphId> avoid_edges;
  const auto avoids = Search(avoid_locations, reader, cost->GetEdgeFilter(), cost->GetNodeFilter());
  for (auto loc : avoid_locations) {
    for (auto& e : avoids.at(loc).edges) {
      avoid_edges.push_back(e.id);
    }
  }
  if (avoid_edges.size() > 0) {
    mode_costing[static_cast<uint32_t>(mode)]->AddUserAvoidEdges(avoid_edges);
  }

  // For multimodal - hack the date time for now!
  if (routetype == "multimodal") {
    path_location.front().date_time_ = "current";
  }

  // Compute the isotile
  auto t1 = std::chrono::high_resolution_clock::now();
  Isochrone isochrone;
  auto isotile = (routetype == "multimodal") ?
      isochrone.ComputeMultiModal(path_location, contour_times.back() + 10, reader, mode_costing, mode) :
      (reverse) ?
        isochrone.ComputeReverse(path_location, contour_times.back() + 10, reader, mode_costing, mode) :
        isochrone.Compute(path_location, contour_times.back() + 10, reader, mode_costing, mode);
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("Compute isotile took " + std::to_string(msecs) + " ms");

  // Evaluate the min, max rows and columns that are set
  int nv = 0;
  int32_t min_row = isotile->nrows();
  int32_t max_row = 0;
  int32_t min_col = isotile->ncolumns();
  int32_t max_col = 0;
  const auto& iso_data = isotile->data();
  for (int32_t row = 0; row < isotile->nrows(); row++) {
    for (int32_t col = 0; col < isotile->ncolumns(); col++) {
      int id = isotile->TileId(col, row);
      if (iso_data[id] < max_minutes + 5) {
        min_row = std::min(row, min_row);
        max_row = std::max(row, max_row);
        min_col = std::min(col, min_col);
        max_col = std::max(col, max_col);
        nv++;
      }
    }
  }
  LOG_INFO("Marked " + std::to_string(nv) + " cells in the isotile" + " size= " + std::to_string(iso_data.size()));
  LOG_INFO("Rows = " + std::to_string(isotile->nrows()) + " min = " + std::to_string(min_row) + " max = " + std::to_string(max_row));
  LOG_INFO("Cols = " + std::to_string(isotile->ncolumns()) + " min = " + std::to_string(min_col) + " max = " + std::to_string(max_col));

  if (denoise < 0.f || denoise > 1.f) {
    denoise = std::max(std::min(denoise, 1.f), 0.f);
    LOG_WARN ("denoise parameter was out of range. Being clamped to " + std::to_string(denoise));
  }
  auto contours = isotile->GenerateContours(contour_times, polygons, denoise,
                           generalize);
  auto geojson = (show_locations) ? json::to_geojson<PointLL>(contours, polygons, colors, path_location)
                                 : json::to_geojson<PointLL>(contours, polygons, colors);

  auto t3 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
  LOG_INFO("Contour Generation took " + std::to_string(msecs) + " ms");
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t1).count();
  LOG_INFO("Isochrone took " + std::to_string(msecs) + " ms");

  std::cout << std::endl;
  if (vm.count("file")) {
    std::ofstream geojsonOut (filename, std::ofstream::out);
    geojsonOut << *geojson;
    geojsonOut.close();
  } else {
    std::cout << *geojson << std::endl;
  }

  return EXIT_SUCCESS;
}

