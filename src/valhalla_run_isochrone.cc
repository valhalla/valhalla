#include <cmath>
#include <cstdint>
#include <cxxopts.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "argparse_utils.h"
#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "loki/search.h"
#include "midgard/logging.h"
#include "proto/options.pb.h"
#include "sif/costconstants.h"
#include "sif/costfactory.h"
#include "thor/isochrone.h"
#include "tyr/serializers.h"
#include "worker.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::sif;
using namespace valhalla::thor;

// Main method for testing a single path
int main(int argc, char* argv[]) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  std::string json_str;
  std::string filename = "";
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_VERSION + "\n\n"
      "a simple command line test tool for generating an isochrone.\n"
      "Use the -j option for specifying the location and isocrhone options.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("j,json", "JSON Example: "
        "'{\"locations\":[{\"lat\":40.748174,\"lon\":-73.984984}],\"costing\":"
        "\"auto\",\"contours\":[{\"time\":15,\"color\":\"ff0000\"}]}'", cxxopts::value<std::string>())
      ("c,config", "Valhalla configuration file", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("f,file", "file name. If omitted program will print to stdout.", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, config, "mjolnir.logging"))
      return EXIT_SUCCESS;

    if (!result.count("json")) {
      throw cxxopts::exceptions::exception("A JSON format request must be present.\n\n" +
                                           options.help());
    }
    json_str = result["json"].as<std::string>();

    if (result.count("file")) {
      filename = result["file"].as<std::string>();
    }
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // Process json request
  Api request;
  ParseApi(json_str, valhalla::Options::isochrone, request);

  auto& options = *request.mutable_options();

  // Get the denoise parameter
  float denoise = options.denoise();
  if (denoise < 0.f || denoise > 1.f) {
    denoise = std::max(std::min(denoise, 1.f), 0.f);
    LOG_WARN("denoise parameter was out of range. Being clamped to " + std::to_string(denoise));
  }

  // Get generalize parameter
  float generalize = kOptimalGeneralization;
  if (options.has_generalize_case()) {
    generalize = options.generalize();
  }

  // Get the polygons parameters
  bool polygons = options.polygons();

  // Show locations
  bool show_locations = options.show_locations();

  // reverse or time dependent (arrive-by) isochrone
  bool reverse = options.reverse() || options.date_time_type() == valhalla::Options::arrive_by;

  // Get Contours
  std::vector<GriddedData<2>::contour_interval_t> contour_times;
  float max_minutes = std::numeric_limits<float>::min();
  for (const auto& contour : options.contours()) {
    if (contour.has_time_case()) {
      max_minutes = std::max(max_minutes, contour.time());
      contour_times.emplace_back(0, contour.time(), "time", contour.color());
    }
  }
  if (options.contours_size() == 0 || max_minutes == std::numeric_limits<float>::min()) {
    throw std::runtime_error("Contours failed to parse. JSON requires a contours object with time");
  }

  // Process locations
  auto locations = PathLocation::fromPBF(options.locations());
  if (locations.size() != 1) {
    // TODO - for now just 1 location - maybe later allow multiple?
    throw std::runtime_error("Requires a single location");
  }

  // Process avoid locations
  auto exclude_locations = PathLocation::fromPBF(options.exclude_locations());
  if (exclude_locations.size() == 0) {
    LOG_INFO("No avoid locations");
  }

  // Get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));

  // Construct costing
  CostFactory factory;

  // Get type of route - this provides the costing method to use.
  const std::string& routetype = valhalla::Costing_Enum_Name(options.costing_type());
  LOG_INFO("routetype: " + routetype);

  // Get the costing method - pass the JSON configuration
  valhalla::TripLeg trip_path;
  sif::TravelMode mode;
  auto mode_costing = factory.CreateModeCosting(options, mode);

  // Find locations
  std::shared_ptr<DynamicCost> cost = mode_costing[static_cast<uint32_t>(mode)];
  const auto projections = Search(locations, reader, cost);
  std::vector<PathLocation> path_location;
  for (const auto& loc : locations) {
    try {
      path_location.push_back(projections.at(loc));
    } catch (...) { exit(EXIT_FAILURE); }
  }

  // Find avoid locations
  std::vector<sif::AvoidEdge> avoid_edges;
  const auto avoids = Search(exclude_locations, reader, cost);
  for (const auto& loc : exclude_locations) {
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
  valhalla::thor::Isochrone isochrone;
  auto expansion_type = routetype == "multimodal"
                            ? ExpansionType::multimodal
                            : (reverse ? ExpansionType::reverse : ExpansionType::forward);
  auto isogrid = isochrone.Expand(expansion_type, request, reader, mode_costing, mode);

  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("Compute isotile took " + std::to_string(msecs) + " ms");

  // Generate contours
  t2 = std::chrono::high_resolution_clock::now();
  auto t3 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
  LOG_INFO("Contour Generation took " + std::to_string(msecs) + " ms");

  std::string res = valhalla::tyr::serializeIsochrones(request, contour_times, isogrid);
  auto t4 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
  LOG_INFO("Isochrone serialization took " + std::to_string(msecs) + " ms");
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t1).count();
  LOG_INFO("Isochrone took " + std::to_string(msecs) + " ms");

  if (!filename.empty()) {
    std::ofstream resOut(filename, std::ofstream::out);
    resOut << res;
    resOut.close();
  } else {
    std::cout << "\n" << res << std::endl;
  }

  // Shutdown protocol buffer library
  google::protobuf::ShutdownProtobufLibrary();

  return EXIT_SUCCESS;
}
