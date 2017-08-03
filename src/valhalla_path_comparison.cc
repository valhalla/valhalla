#include <cstdint>
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

#include "midgard/encoded.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "baldr/pathlocation.h"
#include "baldr/connectivity_map.h"
#include "loki/search.h"
#include "sif/costfactory.h"
#include "odin/directionsbuilder.h"
#include "odin/util.h"
#include "proto/trippath.pb.h"
#include "proto/tripdirections.pb.h"
#include "proto/directions_options.pb.h"
#include "midgard/logging.h"
#include "midgard/distanceapproximator.h"
#include "thor/astar.h"
#include "thor/bidirectional_astar.h"
#include "thor/multimodal.h"
#include "thor/trippathbuilder.h"
#include "thor/attributes_controller.h"
#include "thor/route_matcher.h"
#include "meili/map_matcher.h"
#include "meili/map_matcher_factory.h"
#include "meili/match_result.h"

//using namespace valhalla::midgard;
//using namespace valhalla::baldr;
//using namespace valhalla::loki;
//using namespace valhalla::odin;
using namespace valhalla::sif;
//using namespace valhalla::thor;
using namespace valhalla::meili;

namespace bpo = boost::program_options;
/*
namespace {
  class PathStatistics {
    std::pair<float, float> origin;
    std::pair<float, float> destination;
    std::string success;
    uint32_t passes;
    uint32_t runtime;
    uint32_t trip_time;
    float trip_dist;
    float arc_dist;
    uint32_t manuevers;

  public:
    PathStatistics (std::pair<float, float> p1, std::pair<float, float> p2)
      : origin(p1), destination(p2), success("false"),
        passes(0), runtime(), trip_time(),
        trip_dist(), arc_dist(), manuevers() { }

    void setSuccess(std::string s) { success = s; }
    void incPasses(void) { ++passes; }
    void addRuntime(uint32_t msec) { runtime += msec; }
    void setTripTime(uint32_t t) { trip_time = t; }
    void setTripDist(float d) { trip_dist = d; }
    void setArcDist(float d) { arc_dist = d; }
    void setManuevers(uint32_t n) { manuevers = n; }
    void log() {
      valhalla::midgard::logging::Log(
        (boost::format("%f,%f,%f,%f,%s,%d,%d,%d,%f,%f,%d")
          % origin.first % origin.second % destination.first % destination.second
          % success % passes % runtime % trip_time % trip_dist % arc_dist % manuevers).str(),
        " [STATISTICS] ");
    }
  };
}*/

/**
 * Test a single path from origin to destination.
 */
/*
TripPath PathTest(GraphReader& reader, PathLocation& origin,
                  PathLocation& dest, PathAlgorithm* pathalgorithm,
                  const std::shared_ptr<DynamicCost>* mode_costing,
                  const TravelMode mode, PathStatistics& data,
                  bool multi_run, uint32_t iterations,
                  bool using_astar, bool match_test) {
  auto t1 = std::chrono::high_resolution_clock::now();
  std::vector<PathInfo> pathedges;
  pathedges = pathalgorithm->GetBestPath(origin, dest, reader, mode_costing, mode);
  cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];
  data.incPasses();
  if (pathedges.size() == 0) {
    if (cost->AllowMultiPass()) {
      LOG_INFO("Try again with relaxed hierarchy limits");
      pathalgorithm->Clear();
      float relax_factor = (using_astar) ? 16.0f : 8.0f;
      float expansion_within_factor = (using_astar) ? 4.0f : 2.0f;
      cost->RelaxHierarchyLimits(using_astar, expansion_within_factor);
      pathedges = pathalgorithm->GetBestPath(origin, dest, reader, mode_costing, mode);
      data.incPasses();
    }
  }
  if (pathedges.size() == 0) {
    // Return an empty trip path
    return TripPath();
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("PathAlgorithm GetBestPath took " + std::to_string(msecs) + " ms");

  // Form trip path
  t1 = std::chrono::high_resolution_clock::now();
  AttributesController controller;
  TripPath trip_path = TripPathBuilder::Build(controller, reader, mode_costing,
                                              pathedges, origin, dest,
                                              std::list<PathLocation>{});
  t2 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("TripPathBuilder took " + std::to_string(msecs) + " ms");

  // Time how long it takes to clear the path
  t1 = std::chrono::high_resolution_clock::now();
  pathalgorithm->Clear();
  t2 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("PathAlgorithm Clear took " + std::to_string(msecs) + " ms");

  // Test RouteMatcher
  if (match_test) {
    LOG_INFO("Testing RouteMatcher");

    // Get shape
    std::vector<PointLL> shape = decode<std::vector<PointLL>>(trip_path.shape());
    std::vector<Measurement> trace; trace.reserve(shape.size());
    std::transform(shape.begin(), shape.end(), std::back_inserter(trace), [](const PointLL& p){ return Measurement{p, 0, 0}; });

    // Use the shape to form a single edge correlation at the start and end of
    // the shape (using heading).
    std::vector<valhalla::baldr::Location> locations{shape.front(), shape.back()};
    locations.front().heading_ = std::round(PointLL::HeadingAlongPolyline(shape, 30.f));
    locations.back().heading_ = std::round(PointLL::HeadingAtEndOfPolyline(shape, 30.f));

    std::shared_ptr<DynamicCost> cost = mode_costing[static_cast<uint32_t>(mode)];
    const auto projections = Search(locations, reader, cost->GetEdgeFilter(), cost->GetNodeFilter());
    std::vector<PathLocation> path_location;
    for (auto loc : locations) {
      path_location.push_back(projections.at(loc));
    }
    std::vector<PathInfo> path;
    bool ret = RouteMatcher::FormPath(mode_costing, mode, reader, trace,
                     path_location, path);
    if (ret) {
      LOG_INFO("RouteMatcher succeeded");
    } else {
      LOG_ERROR("RouteMatcher failed");
    }
  }

  // Run again to see benefits of caching
  if (multi_run) {
    uint32_t totalms = 0;
    for (uint32_t i = 0; i < iterations; i++) {
      t1 = std::chrono::high_resolution_clock::now();
      pathedges = pathalgorithm->GetBestPath(origin, dest, reader, mode_costing, mode);
      t2 = std::chrono::high_resolution_clock::now();
      totalms += std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
      pathalgorithm->Clear();
    }
    msecs = totalms / iterations;
    LOG_INFO("PathAlgorithm GetBestPath average: " + std::to_string(msecs) + " ms");
  }
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
                              TripPath& trip_path, const PathLocation& origin,
                              const PathLocation& destination, PathStatistics& data) {
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

    // Depart instruction
    if (maneuver.has_depart_instruction()) {
      valhalla::midgard::logging::Log(
          (boost::format("   %s")
              % maneuver.depart_instruction()).str(),
          " [NARRATIVE] ");
    }

    // Verbal depart instruction
    if (maneuver.has_verbal_depart_instruction()) {
      valhalla::midgard::logging::Log(
          (boost::format("   VERBAL_DEPART: %s")
              % maneuver.verbal_depart_instruction()).str(),
          " [NARRATIVE] ");
    }

    // Instruction
    valhalla::midgard::logging::Log(
        (boost::format("%d: %s | %.1f %s") % m++ % maneuver.text_instruction()
            % maneuver.length() % units).str(),
        " [NARRATIVE] ");

    // Verbal transition alert instruction
    if (maneuver.has_verbal_transition_alert_instruction()) {
      valhalla::midgard::logging::Log(
          (boost::format("   VERBAL_ALERT: %s")
              % maneuver.verbal_transition_alert_instruction()).str(),
          " [NARRATIVE] ");
    }

    // Verbal pre transition instruction
    if (maneuver.has_verbal_pre_transition_instruction()) {
      valhalla::midgard::logging::Log(
          (boost::format("   VERBAL_PRE: %s")
              % maneuver.verbal_pre_transition_instruction()).str(),
          " [NARRATIVE] ");
    }

    // Verbal post transition instruction
    if (maneuver.has_verbal_post_transition_instruction()) {
      valhalla::midgard::logging::Log(
          (boost::format("   VERBAL_POST: %s")
              % maneuver.verbal_post_transition_instruction()).str(),
          " [NARRATIVE] ");
    }

    // Arrive instruction
    if (maneuver.has_arrive_instruction()) {
      valhalla::midgard::logging::Log(
          (boost::format("   %s")
              % maneuver.arrive_instruction()).str(),
          " [NARRATIVE] ");
    }

    // Verbal arrive instruction
    if (maneuver.has_verbal_arrive_instruction()) {
      valhalla::midgard::logging::Log(
          (boost::format("   VERBAL_ARRIVE: %s")
              % maneuver.verbal_arrive_instruction()).str(),
          " [NARRATIVE] ");
    }

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
  if(origin.date_time_) {
    valhalla::midgard::logging::Log(
        "Departed at: " + *origin.date_time_,
        " [NARRATIVE] ");
  }
  if(destination.date_time_) {
    valhalla::midgard::logging::Log(
        "Arrived at: " + *destination.date_time_,
        " [NARRATIVE] ");
  }
  data.setTripTime(trip_directions.summary().time());
  data.setTripDist(trip_directions.summary().length());
  data.setManuevers(trip_directions.maneuver_size());

  return trip_directions;
}

// Returns the costing method (created from the dynamic cost factory).
// Get the costing options. Merge in any request costing options that
// override those in the config.
valhalla::sif::cost_ptr_t get_costing(CostFactory<DynamicCost> factory,
                                      boost::property_tree::ptree& request,
                                      const std::string& costing) {
  std::string method_options = "costing_options." + costing;
  auto costing_options = request.get_child(method_options, {});
  return factory.Create(costing, costing_options);
}*/

// Main method for testing a single path
int main(int argc, char *argv[]) {
  bpo::options_description options("valhalla_path_comparison " VERSION "\n"
  "\n"
  " Usage: valhalla_path_comparison [options]\n"
  "\n"
  "valhalla_path_comparison is a simple command line dev tool for comparing the cost between two routes. "
  "\n"
  "Use the -j option for specifying the locations. "
  "\n"
  "\n");

  std::string routetype, json, config;

  options.add_options()("help,h", "Print this help message.")(
      "version,v", "Print the version of this software.")(
      "type,t", boost::program_options::value<std::string>(&routetype),
      "Route Type: auto|bicycle|pedestrian|auto-shorter")(
      "json,j",
      boost::program_options::value<std::string>(&json),
      R"(JSON Example: {"paths":[[{"lat":12.47,"lon":15.2},{"lat":12.46,"lon":15.21}],[{"lat":12.36,"lon":15.17},{"lat":12.37,"lon":15.18}]],"costing":"bicycle","costing_options":{"bicycle":{"use_roads":0.55,"use_hills":0.1}}})")
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
    std::cout << "valhalla_path_comparison " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  // Path Traces
  std::vector<std::vector<valhalla::baldr::Location>> paths;

  // argument checking and verification
  boost::property_tree::ptree json_ptree;
  ////////////////////////////////////////////////////////////////////////////
  // Process json input
  if (vm.count("json")) {
    std::stringstream stream;
    stream << json;
    boost::property_tree::read_json(stream, json_ptree);

    try {
      for (const auto& path : json_ptree.get_child("paths")) {
        paths.push_back({});
        std::vector<valhalla::baldr::Location>& locations = paths.back();
        for (const auto& location: path.second) {
          locations.emplace_back(std::move(valhalla::baldr::Location::FromPtree(location.second)));
        }
      }
    } catch (...) {
      throw std::runtime_error(
          "insufficiently specified required parameter 'locations'");
    }

    // Parse out the type of route - this provides the costing method to use
    try {
      routetype = json_ptree.get<std::string>("costing");
    } catch (...) {
      throw std::runtime_error("No edge/node costing provided");
    }
  } else {
    std::cerr << "The json parameter was not supplied but is required.\n\n" << options << std::endl;
    return EXIT_FAILURE;
  }

  //parse the config
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config.c_str(), pt);

  // Construct costing
  CostFactory<DynamicCost> factory;
  factory.Register("bicycle", CreateBicycleDebugger);
  /*factory.Register("auto", CreateAutoCost);
  factory.Register("auto_shorter", CreateAutoShorterCost);
  factory.Register("bus", CreateBusCost);
  factory.Register("bicycle", CreateBicycleCost);
  factory.Register("pedestrian", CreatePedestrianCost);
  factory.Register("truck", CreateTruckCost);
  factory.Register("transit", CreateTransitCost);*/
  std::string method_options = "costing_options." + routetype;
  auto costing_options = json_ptree.get_child(method_options, {});
  cost_ptr_t costing = factory.Create(routetype, costing_options);

  // Get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir"));

  MapMatcherFactory map_matcher_factory (pt);
  std::shared_ptr<valhalla::meili::MapMatcher> matcher (map_matcher_factory.Create(pt));

  uint32_t i = 0;
  for (const auto& path : paths) {
    std::cout << "path " << i << std::endl;
    std::cout << "==========================================================\n\n";
    std::vector<Measurement> measurements;
    for (const auto& location : path) {
      measurements.emplace_back (Measurement{
          {location.latlng_.lng(),location.latlng_.lat()},
          matcher->config().get<float>("gps_accuracy"),
          matcher->config().get<float>("search_radius")
      });
    }

    std::vector<valhalla::meili::MatchResult> results = matcher->OfflineMatch(measurements);

    GraphId pred_id;
    GraphId current_id;
    for (const auto& result : results) {
      //std::cout << "lat: " << result.lnglat.lat() << " lon: " << result.lnglat.lng() << std::endl;
      if (result.edgeid == current_id) {
        //std::cout << "continue\n";
        continue;
      }
      if (result.edgeid == kInvalidGraphId) {
        //std::cout << "invalid\n";
        continue;
      }
      current_id = result.edgeid;
      //std::cout << "id: " << current_id << "\n";
      auto tile = reader.GetGraphTile(current_id);
      auto edge = tile->directededge(current_id);
      auto edgeinfo = tile->edgeinfo(edge->edgeinfo_offset());
      std::string name = edgeinfo.GetNames().size() == 0 ? "unnamed" : edgeinfo.GetNames()[0];
      std::cout << "wayid: " << edgeinfo.wayid() << std::endl;
      std::cout << "name: " << name << std::endl;
      std::cout << "-----------------------\n";
      std::cout << "Edge\n..............\n";
      Cost edgecost = costing->EdgeCost(edge);
      std::cout << "EdgeCost cost: " << edgecost.cost << " secs: " << edgecost.secs << std::endl;
      /*if (pred_id != kInvalidGraphId) {
        auto pred_tile = reader.GetGraphTile(pred_id);
        auto pred_edge = pred_tile->directededge(pred_id);
        auto predinfo = tile->edgeinfo(pred_edge->edgeinfo_offset());
        auto node_id = pred_edge->endnode();
        auto node = tile->node(node_id);
        EdgeLabel pred_label (0, pred_id, pred_edge, {}, 0.0f, 0.0f, static_cast<TravelMode>(0), 0);
        std::cout << "Transition\n..............\n";
        Cost transCost = costing->TransitionCost(edge, node, pred_label);
        std::cout << "TransitionCost cost: " << transCost.cost << " secs: " << transCost.secs << std::endl;
      }*/
      pred_id = current_id;
      std::cout << std::endl;
    }
    ++i;
    std::cout << "\n\n";
  }
/*
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
  auto t1 = std::chrono::high_resolution_clock::now();
  std::shared_ptr<DynamicCost> cost = mode_costing[static_cast<uint32_t>(mode)];
  const auto projections = Search(locations, reader, cost->GetEdgeFilter(), cost->GetNodeFilter());
  std::vector<PathLocation> path_location;
  for (auto loc : locations) {
    try {
      path_location.push_back(projections.at(loc));
      //TODO: get transit level for transit costing
      //TODO: if transit send a non zero radius
    } catch (...) {
      return EXIT_FAILURE;
    }
  }

  // Normalize the edge scores
  for (auto& correlated : path_location) {
    auto minScoreEdge = *std::min_element(correlated.edges.begin(), correlated.edges.end(),
       [](PathLocation::PathEdge i, PathLocation::PathEdge j)->bool {
         return i.score < j.score;
       });

    for(auto& e : correlated.edges) {
      e.score -= minScoreEdge.score;
    }
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(
                  t2 - t1).count();
  LOG_INFO("Location Processing took " + std::to_string(msecs) + " ms");

  // Get the route
  AStarPathAlgorithm astar;
  BidirectionalAStar bd;
  MultiModalPathAlgorithm mm;
  for (uint32_t i = 0; i < n; i++) {
    // Choose path algorithm
    PathAlgorithm* pathalgorithm;
    if (routetype == "multimodal") {
      pathalgorithm = &mm;
    } else {
      // Use bidirectional except for possible trivial cases
      pathalgorithm = &bd;
      for (auto& edge1 : path_location[i].edges) {
        for (auto& edge2 : path_location[i+1].edges) {
          if (edge1.id == edge2.id) {
            pathalgorithm = &astar;
          }
        }
      }
    }
    bool using_astar = (pathalgorithm == &astar);

    // Get the best path
    try {
      trip_path = PathTest(reader, path_location[i], path_location[i + 1],
                           pathalgorithm, mode_costing, mode, data, multi_run,
                           iterations, using_astar, match_test);
    } catch (std::runtime_error& rte) {
      LOG_ERROR("trip_path not found");
    }

    // If successful get directions
    if (trip_path.node().size() > 0) {
      // Try the the directions
      t1 = std::chrono::high_resolution_clock::now();
      TripDirections trip_directions = DirectionsTest(directions_options, trip_path,
                        path_location[i], path_location[i+1], data);
      t2 = std::chrono::high_resolution_clock::now();
      msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

      auto trip_time = trip_directions.summary().time();
      auto trip_length = trip_directions.summary().length() * 1609.344f;
      LOG_INFO("trip_processing_time (ms)::" + std::to_string(msecs));
      LOG_INFO("trip_time (secs)::" + std::to_string(trip_time));
      LOG_INFO("trip_length (meters)::" + std::to_string(trip_length));
    } else {
      // Check if origins are unreachable
      bool unreachable_origin = false;
      for (auto& edge : path_location[i].edges) {
        const GraphTile* tile = reader.GetGraphTile(edge.id);
        const DirectedEdge* directededge = tile->directededge(edge.id);
        auto ei = tile->edgeinfo(directededge->edgeinfo_offset());
        if (directededge->unreachable()) {
          LOG_INFO("Origin edge is unconnected: wayid = " + std::to_string(ei.wayid()));
          unreachable_origin = true;
        }
        LOG_INFO("Origin wayId = " + std::to_string(ei.wayid()));
      }

      // Check if destinations are unreachable
      bool unreachable_dest = false;
      for (auto& edge : path_location[i+1].edges) {
        const GraphTile* tile = reader.GetGraphTile(edge.id);
        const DirectedEdge* directededge = tile->directededge(edge.id);
        auto ei = tile->edgeinfo(directededge->edgeinfo_offset());
        if (directededge->unreachable()) {
          LOG_INFO("Destination edge is unconnected: wayid = " + std::to_string(ei.wayid()));
          unreachable_dest = true;
        }
        LOG_INFO("Destination wayId = " + std::to_string(ei.wayid()));
      }

      // Route was unsuccessful
      if (unreachable_origin && unreachable_dest) {
        data.setSuccess("fail_unreachable_locations");
      } else if (unreachable_origin) {
        data.setSuccess("fail_unreachable_origin");
      } else if (unreachable_dest) {
        data.setSuccess("fail_unreachable_dest");
      } else {
        data.setSuccess("fail_no_route");
      }
    }
  }*/

  return EXIT_SUCCESS;
}

