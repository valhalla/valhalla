#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "loki/search.h"
#include "midgard/logging.h"
#include "sif/costconstants.h"
#include "sif/costfactory.h"
#include "thor/isochrone.h"
#include "tyr/serializers.h"
#include "worker.h"

#include <valhalla/proto/options.pb.h>

#include "config.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace bpo = boost::program_options;

// Main method for testing a single path
int main(int argc, char* argv[]) {
  bpo::options_description poptions(
      "valhalla_run_isochrone " VALHALLA_VERSION "\n"
      "\n"
      " Usage: valhalla_run_isochrone [options]\n"
      "\n"
      "valhalla_run_isochrone is a simple command line test tool for generating an isochrone. "
      "\n"
      "Use the -j option for specifying the location and isocrhone options "
      "\n"
      "\n");
  std::string json, config, filename;
  poptions.add_options()("help,h", "Print this help message.")("version,v",
                                                               "Print the version of this software.")(
      "json,j", boost::program_options::value<std::string>(&json),
      "JSON Example: "
      "'{\"locations\":[{\"lat\":40.748174,\"lon\":-73.984984}],\"costing\":"
      "\"auto\",\"contours\":[{\"time\":15,\"color\":\"ff0000\"}]}'")
      // positional arguments
      ("config", bpo::value<std::string>(&config),
       "Valhalla configuration file")("file,f", bpo::value<std::string>(&filename),
                                      "Geojson output file name.");

  bpo::positional_options_description pos_options;
  pos_options.add("config", 1);
  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(poptions).positional(pos_options).run(),
               vm);
    bpo::notify(vm);

  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // Verify args. Make sure JSON payload exists.
  if (vm.count("help")) {
    std::cout << poptions << "\n";
    return EXIT_SUCCESS;
  }
  if (vm.count("version")) {
    std::cout << "valhalla_run_isochrone " << VALHALLA_VERSION << "\n";
    return EXIT_SUCCESS;
  }
  if (vm.count("json") == 0) {
    std::cerr << "A JSON format request must be present."
              << "\n";
    return EXIT_FAILURE;
  }

  // Process json request
  Api request;
  ParseApi(json, valhalla::Options::isochrone, request);
  auto& options = *request.mutable_options();

  // Get the denoise parameter
  float denoise = options.denoise();
  if (denoise < 0.f || denoise > 1.f) {
    denoise = std::max(std::min(denoise, 1.f), 0.f);
    LOG_WARN("denoise parameter was out of range. Being clamped to " + std::to_string(denoise));
  }

  // Get generalize parameter
  float generalize = kOptimalGeneralization;
  if (options.has_generalize()) {
    generalize = options.generalize();
  }

  // Get the polygons parameters
  bool polygons = options.polygons();

  // Show locations
  bool show_locations = options.show_locations();

  // reverse (arrive-by) isochrone - trigger if date time type is arrive by
  // TODO - is this how we want to expose in the service? only support reverse
  // for time dependent isochrones? or do we also want a flag to support for
  // general case with no time?
  bool reverse = options.date_time_type() == valhalla::Options::arrive_by;

  // Get Contours
  std::unordered_map<float, std::string> colors{};
  std::vector<float> contour_times;
  if (options.contours_size() == 0) {
    throw std::runtime_error("Contours failed to parse. JSON requires a contours object");
  }
  for (const auto& contour : options.contours()) {
    contour_times.push_back(contour.time());
    colors[contour_times.back()] = contour.color();
  }

  // Process locations
  auto locations = PathLocation::fromPBF(options.locations());
  if (locations.size() != 1) {
    // TODO - for now just 1 location - maybe later allow multiple?
    throw std::runtime_error("Requires a single location");
  }

  // Process avoid locations
  auto avoid_locations = PathLocation::fromPBF(options.avoid_locations());
  if (avoid_locations.size() == 0) {
    LOG_INFO("No avoid locations");
  }

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
  CostFactory<DynamicCost> factory;
  factory.RegisterStandardCostingModels();

  // Get type of route - this provides the costing method to use.
  std::string routetype = valhalla::Costing_Enum_Name(options.costing());
  LOG_INFO("routetype: " + routetype);

  // Get the costing method - pass the JSON configuration
  valhalla::TripLeg trip_path;
  TravelMode mode;
  std::shared_ptr<DynamicCost> mode_costing[4];
  if (routetype == "multimodal") {
    // Create array of costing methods per mode and set initial mode to
    // pedestrian
    mode_costing[0] = factory.Create(valhalla::Costing::auto_, options);
    mode_costing[1] = factory.Create(valhalla::Costing::pedestrian, options);
    mode_costing[2] = factory.Create(valhalla::Costing::bicycle, options);
    mode_costing[3] = factory.Create(valhalla::Costing::transit, options);
    mode = TravelMode::kPedestrian;
  } else {
    // Assign costing method, override any config options that are in the
    // json request
    std::shared_ptr<DynamicCost> cost = factory.Create(options.costing(), options);
    mode = cost->travel_mode();
    mode_costing[static_cast<uint32_t>(mode)] = cost;
  }

  // Find locations
  std::shared_ptr<DynamicCost> cost = mode_costing[static_cast<uint32_t>(mode)];
  const auto projections = Search(locations, reader, cost.get());
  std::vector<PathLocation> path_location;
  for (const auto& loc : locations) {
    try {
      path_location.push_back(projections.at(loc));
    } catch (...) { exit(EXIT_FAILURE); }
  }

  // Find avoid locations
  std::vector<sif::AvoidEdge> avoid_edges;
  const auto avoids = Search(avoid_locations, reader, cost.get());
  for (const auto& loc : avoid_locations) {
    for (auto& e : avoids.at(loc).edges) {
      avoid_edges.push_back({e.id, e.percent_along});
    }
  }
  if (avoid_edges.size() > 0) {
    mode_costing[static_cast<uint32_t>(mode)]->AddUserAvoidEdges(avoid_edges);
  }

  // For multimodal - hack the date time for now!
  if (routetype == "multimodal") {
    path_location.front().date_time_ = "current";
  }
  // TODO: build real request from options above and call the functions like actor_t does
  options.mutable_locations()->Clear();
  for (const auto& pl : path_location) {
    valhalla::baldr::PathLocation::toPBF(pl, options.mutable_locations()->Add(), reader);
  }

  // Compute the isotile
  auto t1 = std::chrono::high_resolution_clock::now();
  Isochrone isochrone;
  auto isotile =
      (routetype == "multimodal")
          ? isochrone.ComputeMultiModal(*options.mutable_locations(), contour_times.back() + 10,
                                        reader, mode_costing, mode)
          : (reverse)
                ? isochrone.ComputeReverse(*options.mutable_locations(), contour_times.back() + 10,
                                           reader, mode_costing, mode)
                : isochrone.Compute(*options.mutable_locations(), contour_times.back() + 10, reader,
                                    mode_costing, mode);
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("Compute isotile took " + std::to_string(msecs) + " ms");

  // Evaluate the min, max rows and columns that are set
  int nv = 0;
  uint32_t max_minutes = contour_times.back();
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
  LOG_INFO("Marked " + std::to_string(nv) + " cells in the isotile" +
           " size= " + std::to_string(iso_data.size()));
  LOG_INFO("Rows = " + std::to_string(isotile->nrows()) + " min = " + std::to_string(min_row) +
           " max = " + std::to_string(max_row));
  LOG_INFO("Cols = " + std::to_string(isotile->ncolumns()) + " min = " + std::to_string(min_col) +
           " max = " + std::to_string(max_col));

  // Generate contours
  t2 = std::chrono::high_resolution_clock::now();
  auto contours = isotile->GenerateContours(contour_times, polygons, denoise, generalize);
  auto t3 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
  LOG_INFO("Contour Generation took " + std::to_string(msecs) + " ms");

  // Serialize to GeoJSON
  std::string geojson = valhalla::tyr::serializeIsochrones<PointLL>(request, contours, polygons,
                                                                    colors, show_locations);

  auto t4 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
  LOG_INFO("GeoJSON serialization took " + std::to_string(msecs) + " ms");
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t1).count();
  LOG_INFO("Isochrone took " + std::to_string(msecs) + " ms");

  std::cout << std::endl;
  if (vm.count("file")) {
    std::ofstream geojsonOut(filename, std::ofstream::out);
    geojsonOut << geojson;
    geojsonOut.close();
  } else {
    std::cout << geojson << std::endl;
  }

  // Shutdown protocol buffer library
  google::protobuf::ShutdownProtobufLibrary();

  return EXIT_SUCCESS;
}
