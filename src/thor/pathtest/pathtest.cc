#include <iostream>
#include <string>
#include <vector>
#include <queue>
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
#include "thor/pathalgorithm.h"
#include "thor/trippathbuilder.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::odin;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace bpo = boost::program_options;

/**
 * Test a single path from origin to destination.
 */
TripPath PathTest(GraphReader& reader, const PathLocation& origin,
                  const PathLocation& dest, std::shared_ptr<DynamicCost> cost) {
  auto t1 = std::chrono::high_resolution_clock::now();
  PathAlgorithm pathalgorithm;
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(
      t2 - t1).count();
  LOG_INFO("PathAlgorithm Construction took " + std::to_string(msecs) + " ms");
  t1 = std::chrono::high_resolution_clock::now();
  std::vector<PathInfo> pathedges;
  pathedges = pathalgorithm.GetBestPath(origin, dest, reader, cost);
  if (pathedges.size() == 0) {
    if (cost->AllowMultiPass()) {
      LOG_INFO("Try again with relaxed hierarchy limits");
      pathalgorithm.Clear();
      cost->RelaxHierarchyLimits(16.0f);
      pathedges = pathalgorithm.GetBestPath(origin, dest, reader, cost);
    }
  }
  if (pathedges.size() == 0) {
    cost->DisableHighwayTransitions();
    pathedges = pathalgorithm.GetBestPath(origin, dest, reader, cost);
    if (pathedges.size() == 0) {
      throw std::runtime_error("No path could be found for input");
    }
  }
  t2 = std::chrono::high_resolution_clock::now();
  msecs =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("PathAlgorithm GetBestPath took " + std::to_string(msecs) + " ms");

  // Form output information based on pathedges
  t1 = std::chrono::high_resolution_clock::now();
  TripPath trip_path = TripPathBuilder::Build(reader, pathedges, origin, dest);

  // TODO - perhaps walk the edges to find total length?
  //LOG_INFO("Trip length is: " + std::to_string(trip_path.length) + " mi");
  t2 = std::chrono::high_resolution_clock::now();
  msecs =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("TripPathBuilder took " + std::to_string(msecs) + " ms");

  t1 = std::chrono::high_resolution_clock::now();
  pathalgorithm.Clear();
  t2 = std::chrono::high_resolution_clock::now();
  msecs =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("PathAlgorithm Clear took " + std::to_string(msecs) + " ms");

  // Run again to see benefits of caching
  uint32_t totalms = 0;
  for (uint32_t i = 0; i < 10; i++) {
    t1 = std::chrono::high_resolution_clock::now();
    pathedges = pathalgorithm.GetBestPath(origin, dest, reader, cost);
    t2 = std::chrono::high_resolution_clock::now();
    totalms += std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();

    pathalgorithm.Clear();
  }
  msecs = totalms / 10;
  LOG_INFO("PathAlgorithm GetBestPath average: " + std::to_string(msecs) + " ms");
  return trip_path;
}

namespace std {

//TODO: maybe move this into location.h if its actually useful elsewhere than here?
std::string to_string(const valhalla::baldr::Location& l) {
  std::string s;
  for (auto address : { &l.name_, &l.street_, &l.city_, &l.state_, &l.zip_, &l
      .country_ }) {
    s.append(*address);
    s.push_back(',');
  }
  s.erase(s.end() - 1);
  return s;
}

//TODO: maybe move this into location.h if its actually useful elsewhere than here?
std::string to_json(const valhalla::baldr::Location& l) {
  std::string json = "{";
  json += "\"lat\":";
  json += std::to_string(l.latlng_.lat());

  json += ",\"lon\":";
  json += std::to_string(l.latlng_.lng());

  json += ",\"type\":\"";
  json +=
      (l.stoptype_ == valhalla::baldr::Location::StopType::THROUGH) ?
          "through" : "break";
  json += "\"";

  if (l.heading_) {
    json += ",\"heading\":";
    json += *l.heading_;
  }

  if (!l.name_.empty()) {
    json += ",\"name\":\"";
    json += l.name_;
    json += "\"";
  }

  if (!l.street_.empty()) {
    json += ",\"street\":\"";
    json += l.street_;
    json += "\"";
  }

  if (!l.city_.empty()) {
    json += ",\"city\":\"";
    json += l.city_;
    json += "\"";
  }

  if (!l.state_.empty()) {
    json += ",\"state\":\"";
    json += l.state_;
    json += "\"";
  }

  if (!l.zip_.empty()) {
    json += ",\"postal_code\":\"";
    json += l.zip_;
    json += "\"";
  }

  if (!l.country_.empty()) {
    json += ",\"country\":\"";
    json += l.country_;
    json += "\"";
  }

  json += "}";

  return json;
}

}

std::string GetFormattedTime(uint32_t seconds) {
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

TripDirections DirectionsTest(const DirectionsOptions& directions_options,
                              TripPath& trip_path, Location origin,
                              Location destination) {
  DirectionsBuilder directions;
  TripDirections trip_directions = directions.Build(directions_options,
                                                    trip_path);
  std::string units = (
      directions_options.units()
          == DirectionsOptions::Units::DirectionsOptions_Units_kKilometers ?
          "km" : "mi");
  int m = 1;
  valhalla::midgard::logging::Log("From: " + std::to_string(origin),
                                  " [NARRATIVE] ");
  valhalla::midgard::logging::Log("To: " + std::to_string(destination),
                                  " [NARRATIVE] ");
  valhalla::midgard::logging::Log(
      "==============================================", " [NARRATIVE] ");
  for (int i = 0; i < trip_directions.maneuver_size(); ++i) {
    const auto& maneuver = trip_directions.maneuver(i);
    valhalla::midgard::logging::Log(
        (boost::format("%d: %s | %.1f %s") % m++ % maneuver.text_instruction()
            % maneuver.length() % units).str(),
        " [NARRATIVE] ");
    if (i < trip_directions.maneuver_size() - 1)
      valhalla::midgard::logging::Log(
          "----------------------------------------------", " [NARRATIVE] ");
  }
  valhalla::midgard::logging::Log(
      "==============================================", " [NARRATIVE] ");
  valhalla::midgard::logging::Log(
      "Total time: " + GetFormattedTime(trip_directions.summary().time()),
      " [NARRATIVE] ");
  valhalla::midgard::logging::Log(
      (boost::format("Total length: %.1f %s")
          % trip_directions.summary().length() % units).str(),
      " [NARRATIVE] ");

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
  "Use the -o and -d options OR the -j option for specifying the locations. "
  "\n"
  "\n");

  std::string origin, destination, routetype, json, config;

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
    std::cout << "pathtest " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  // Directions options
  // TODO - read options?
  DirectionsOptions directions_options;
  directions_options.set_units(
      DirectionsOptions::Units::DirectionsOptions_Units_kMiles);
  directions_options.set_language("en_US");

  Location originloc(PointLL { 0, 0 });
  Location destloc(PointLL { 0, 0 });
  // argument checking and verification
  if (vm.count("json") == 0) {
    for (auto arg : std::vector<std::string> { "origin", "destination", "type",
        "config" }) {
      if (vm.count(arg) == 0) {
        std::cerr
            << "The <"
            << arg
            << "> argument was not provided, but is mandatory when json is not provided\n\n";
        std::cerr << options << "\n";
        return EXIT_FAILURE;
      }
    }
    originloc = Location::FromCsv(origin);
    destloc = Location::FromCsv(destination);
  } else {
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

    originloc = locations.at(0);
    destloc = locations.at(locations.size() - 1);

    // Parse out the type of route - this provides the costing method to use
    std::string costing;
    try {
      routetype = json_ptree.get<std::string>("costing");
    } catch (...) {
      throw std::runtime_error("No edge/node costing provided");
    }

    // Grab the directions options, if they exist
    auto directions_options_ptree_ptr = json_ptree.get_child_optional(
        "directions_options");
    if (directions_options_ptree_ptr) {
      directions_options = valhalla::odin::GetDirectionsOptions(
          *directions_options_ptree_ptr);
    }
  }

  // TODO: remove after input files are transformed
#ifdef LOGGING_LEVEL_DEBUG
  std::string json_input = "-j '{\"locations\":[";
  json_input += std::to_json(originloc);
  json_input += ",";
  json_input += std::to_json(destloc);
  json_input += "],\"costing\":\"auto\",";
  json_input += "\"directions_options\":{\"units\":\"miles\"}}'";
  json_input += " --config ../conf/valhalla.json";
  valhalla::midgard::logging::Log(json_input, " [JSON_INPUT] ");
#endif

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
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir.hierarchy"));

  auto tile_hierarchy = reader.GetTileHierarchy();
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  auto tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  uint32_t origin_tile = tiles.TileId(originloc.latlng_);
  uint32_t dest_tile = tiles.TileId(destloc.latlng_);
  auto t10 = std::chrono::high_resolution_clock::now();
  if (!reader.AreConnected({origin_tile, local_level, 0}, {dest_tile, local_level, 0})) {
    LOG_INFO("No tile connectivity between origin and destination.");
    return EXIT_SUCCESS;
  }
  auto t20 = std::chrono::high_resolution_clock::now();
  uint32_t msecs1 =
         std::chrono::duration_cast<std::chrono::milliseconds>(t20 - t10).count();
  LOG_INFO("Tile CoverageMap took " + std::to_string(msecs1) + " ms");

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

  // Get the costing method - pass the JSON configuration
  TripPath trip_path;
  if (routetype == "multimodal") {
    auto t1 = std::chrono::high_resolution_clock::now();
    // Create array of costing methods per mode
    std::shared_ptr<DynamicCost> mode_costing[4];
    mode_costing[0] = factory.Create("auto",
                         pt.get_child("costing_options.auto"));
    mode_costing[1] = factory.Create("pedestrian",
                         pt.get_child("costing_options.pedestrian"));
    mode_costing[2] = factory.Create("bicycle",
                         pt.get_child("costing_options.bicycle"));
    mode_costing[3] = factory.Create("transit",
                         pt.get_child("costing_options.transit"));

    // For now assume pedestrian mode at origin and destination
    PathLocation pathOrigin = Search(originloc, reader, mode_costing[1]->GetFilter());
    PathLocation pathDest = Search(destloc, reader, mode_costing[1]->GetFilter());
    PathAlgorithm pathalgorithm;

    // Have costing models use the per mode max distances
    for (auto& cost : mode_costing) {
      cost->UseMaxModeDistance();
    }

    // Check if there no possible path to destination based on mode
    // to the destination - for now assume pedestrian
    if (!pathalgorithm.CanReachDestination(pathDest, reader,
                         TravelMode::kPedestrian, mode_costing)) {
      LOG_INFO("Cannot reach destination - too far from a transit stop");
      return EXIT_SUCCESS;
    }

    std::vector<PathInfo> pathedges = pathalgorithm.GetBestPathMM(pathOrigin,
                   pathDest, reader, mode_costing);
    trip_path = TripPathBuilder::Build(reader, pathedges, pathOrigin, pathDest);

  } else {
    LOG_INFO("routetype: " + routetype);
    std::shared_ptr<DynamicCost> cost = factory.Create(
      routetype, pt.get_child("costing_options." + routetype));

    // Use Loki to get location information
    auto t1 = std::chrono::high_resolution_clock::now();
    PathLocation pathOrigin = Search(originloc, reader, cost->GetFilter());
    PathLocation pathDest = Search(destloc, reader, cost->GetFilter());
    auto t2 = std::chrono::high_resolution_clock::now();
    uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(
        t2 - t1).count();
    LOG_INFO("Location Processing took " + std::to_string(msecs) + " ms");

    // Get the route
    t1 = std::chrono::high_resolution_clock::now();
    trip_path = PathTest(reader, pathOrigin, pathDest, cost);
    t2 = std::chrono::high_resolution_clock::now();
    msecs =
        std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    LOG_INFO("PathTest took " + std::to_string(msecs) + " ms");
  }

  // Try the the directions
  auto t1 = std::chrono::high_resolution_clock::now();
  TripDirections trip_directions = DirectionsTest(directions_options, trip_path,
                                                  originloc, destloc);
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("TripDirections took " + std::to_string(msecs) + " ms");

  return EXIT_SUCCESS;
}

