#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cxxopts.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>

#include "baldr/attributes_controller.h"
#include "baldr/connectivity_map.h"
#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "baldr/tilehierarchy.h"
#include "loki/search.h"
#include "loki/worker.h"
#include "midgard/distanceapproximator.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "odin/directionsbuilder.h"
#include "odin/enhancedtrippath.h"
#include "odin/util.h"
#include "sif/costfactory.h"
#include "thor/bidirectional_astar.h"
#include "thor/multimodal.h"
#include "thor/route_matcher.h"
#include "thor/triplegbuilder.h"
#include "thor/unidirectional_astar.h"
#include "worker.h"

#include "proto/api.pb.h"
#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"

#include "argparse_utils.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::odin;
using namespace valhalla::sif;
using namespace valhalla::thor;
using namespace valhalla::meili;

namespace {

std::string get_env(const std::string& key) {
  char* val = std::getenv(key.c_str());
  return val == nullptr ? std::string("") : std::string(val);
}

// Default maximum distance between locations to choose a time dependent path algorithm
const float kDefaultMaxTimeDependentDistance = 500000.0f;

class PathStatistics {
  std::pair<float, float> origin;
  std::pair<float, float> destination;
  std::string success;
  uint32_t passes;
  uint32_t runtime;
  uint32_t trip_time;
  float trip_dist;
  float arc_dist;
  uint32_t maneuvers;
  double elapsed_cost_seconds;
  double elapsed_cost_cost;

public:
  PathStatistics(std::pair<float, float> p1, std::pair<float, float> p2)
      : origin(p1), destination(p2), success("false"), passes(0), runtime(), trip_time(), trip_dist(),
        arc_dist(), maneuvers(), elapsed_cost_seconds(0), elapsed_cost_cost(0) {
  }

  void setSuccess(std::string s) {
    success = std::move(s);
  }
  void incPasses(void) {
    ++passes;
  }
  void addRuntime(uint32_t msec) {
    runtime += msec;
  }
  void setTripTime(uint32_t t) {
    trip_time = t;
  }
  void setTripDist(float d) {
    trip_dist = d;
  }
  void setArcDist(float d) {
    arc_dist = d;
  }
  void setManeuvers(uint32_t n) {
    maneuvers = n;
  }
  void setElapsedCostSeconds(double secs) {
    elapsed_cost_seconds = secs;
  }
  void setElapsedCostCost(double cost) {
    elapsed_cost_cost = cost;
  }
  void log() {
    valhalla::midgard::logging::Log((boost::format("%f,%f,%f,%f,%s,%d,%d,%d,%f,%f,%d,%f,%f") %
                                     origin.first % origin.second % destination.first %
                                     destination.second % success % passes % runtime % trip_time %
                                     trip_dist % arc_dist % maneuvers % elapsed_cost_seconds %
                                     elapsed_cost_cost)
                                        .str(),
                                    " [STATISTICS] ");
  }
};
} // namespace

/**
 * Test a single path from origin to destination.
 */
const valhalla::TripLeg* PathTest(GraphReader& reader,
                                  valhalla::Location& origin,
                                  valhalla::Location& dest,
                                  PathAlgorithm* pathalgorithm,
                                  const mode_costing_t& mode_costing,
                                  const TravelMode mode,
                                  PathStatistics& data,
                                  bool multi_run,
                                  uint32_t iterations,
                                  bool using_bd,
                                  bool match_test,
                                  const std::string& routetype,
                                  valhalla::Api& request) {
  auto t1 = std::chrono::high_resolution_clock::now();
  auto paths =
      pathalgorithm->GetBestPath(origin, dest, reader, mode_costing, mode, request.options());
  cost_ptr_t cost = mode_costing[static_cast<uint32_t>(mode)];

  // If bidirectional A*, disable use of destination only edges on the first pass.
  // If there is a failure, we allow them on the second pass.
  if (using_bd) {
    cost->set_allow_destination_only(false);
  }

  cost->set_pass(0);
  data.incPasses();
  if (paths.empty() || (routetype == "pedestrian" && pathalgorithm->has_ferry())) {
    if (cost->AllowMultiPass()) {
      LOG_INFO("Try again with relaxed hierarchy limits");
      cost->set_pass(1);
      pathalgorithm->Clear();
      cost->RelaxHierarchyLimits(using_bd);
      cost->set_allow_destination_only(true);
      paths = pathalgorithm->GetBestPath(origin, dest, reader, mode_costing, mode, request.options());
      data.incPasses();
    }
  }
  if (paths.empty()) {
    // Return an empty trip path
    return nullptr;
  }
  const auto& pathedges = paths.front();
  LOG_INFO("Number of alternates requested=" + std::to_string(request.options().alternates()));
  LOG_INFO("Number of paths=" + std::to_string(paths.size()));
  LOG_INFO("Number of pathedges=" + std::to_string(pathedges.size()));

  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("PathAlgorithm GetBestPath took " + std::to_string(msecs) + " ms");

  // Form trip path
  t1 = std::chrono::high_resolution_clock::now();
  AttributesController controller;
  auto& trip_path = *request.mutable_trip()->mutable_routes()->Add()->mutable_legs()->Add();
  TripLegBuilder::Build(request.options(), controller, reader, mode_costing, pathedges.begin(),
                        pathedges.end(), origin, dest, trip_path, {pathalgorithm->name()});
  t2 = std::chrono::high_resolution_clock::now();
  msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("TripLegBuilder took " + std::to_string(msecs) + " ms");

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

    // Use the shape to form a single edge correlation at the start and end of
    // the shape (using heading).
    std::vector<valhalla::baldr::Location> locations{shape.front(), shape.back()};
    locations.front().heading_ = std::round(PointLL::HeadingAlongPolyline(shape, 30.f));
    locations.back().heading_ = std::round(PointLL::HeadingAtEndOfPolyline(shape, 30.f));

    std::shared_ptr<DynamicCost> cost = mode_costing[static_cast<uint32_t>(mode)];
    const auto projections = Search(locations, reader, cost);
    std::vector<PathLocation> path_location;
    valhalla::Options options;

    for (const auto& ll : shape) {
      auto* sll = options.mutable_shape()->Add();
      sll->mutable_ll()->set_lat(ll.lat());
      sll->mutable_ll()->set_lng(ll.lng());
      // set type to via by default
      sll->set_type(valhalla::Location::kVia);
    }
    // first and last always get type break
    if (options.shape_size()) {
      options.mutable_shape(0)->set_type(valhalla::Location::kBreak);
      options.mutable_shape(options.shape_size() - 1)->set_type(valhalla::Location::kBreak);
    }

    for (const auto& loc : locations) {
      path_location.push_back(projections.at(loc));
      PathLocation::toPBF(path_location.back(), options.mutable_locations()->Add(), reader);
    }
    std::vector<std::vector<PathInfo>> paths;
    bool ret = RouteMatcher::FormPath(mode_costing, mode, reader, options, paths);
    if (ret) {
      LOG_INFO("RouteMatcher succeeded");
    } else {
      LOG_ERROR("RouteMatcher failed.");
    }
  }

  // Run again to see benefits of caching
  if (multi_run) {
    uint32_t total_get_best_path_ms = 0;
    uint32_t total_trip_leg_builder_ms = 0;
    for (uint32_t i = 0; i < iterations; i++) {
      t1 = std::chrono::high_resolution_clock::now();
      paths = pathalgorithm->GetBestPath(origin, dest, reader, mode_costing, mode, request.options());
      t2 = std::chrono::high_resolution_clock::now();
      total_get_best_path_ms +=
          std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

      // Form trip leg
      t1 = std::chrono::high_resolution_clock::now();
      AttributesController controller;
      valhalla::TripLeg trip_leg;
      const auto& pathedges = paths.front();
      TripLegBuilder::Build(request.options(), controller, reader, mode_costing, pathedges.begin(),
                            pathedges.end(), origin, dest, trip_leg, {pathalgorithm->name()});
      t2 = std::chrono::high_resolution_clock::now();
      total_trip_leg_builder_ms +=
          std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

      pathalgorithm->Clear();
    }
    msecs = total_get_best_path_ms / iterations;
    LOG_INFO("PathAlgorithm GetBestPath average: " + std::to_string(msecs) + " ms");
    msecs = total_trip_leg_builder_ms / iterations;
    LOG_INFO("TripLegBuilder average: " + std::to_string(msecs) + " ms");
  }
  return &request.trip().routes(0).legs(0);
}

namespace std {

// TODO: need to pass original json here for extra address bits that we dont store in Location
std::string to_string(const valhalla::baldr::Location& l) {
  std::string s;
  for (auto address : {&l.name_, &l.street_}) {
    s.append(*address);
    s.push_back(',');
  }
  s.erase(s.end() - 1);
  return s;
}

} // namespace std

std::string GetFormattedTime(uint32_t seconds) {
  uint32_t hours = (uint32_t)seconds / 3600;
  uint32_t minutes = ((uint32_t)(seconds / 60)) % 60;
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

valhalla::DirectionsLeg DirectionsTest(valhalla::Api& api,
                                       valhalla::Location& orig,
                                       valhalla::Location& dest,
                                       PathStatistics& data,
                                       bool verbose_lanes,
                                       valhalla::odin::MarkupFormatter& markup_formatter) {
  // TEMPORARY? Change to PathLocation...
  const PathLocation& origin = PathLocation::fromPBF(orig);
  const PathLocation& destination = PathLocation::fromPBF(dest);

  DirectionsBuilder::Build(api, markup_formatter);
  const auto& trip_directions = api.directions().routes(0).legs(0);
  EnhancedTripLeg etl(*api.mutable_trip()->mutable_routes(0)->mutable_legs(0));
  std::string units = (api.options().units() == valhalla::Options::kilometers ? "km" : "mi");
  int m = 1;
  valhalla::midgard::logging::Log("From: " + std::to_string(origin), " [NARRATIVE] ");
  valhalla::midgard::logging::Log("To: " + std::to_string(destination), " [NARRATIVE] ");
  valhalla::midgard::logging::Log("==============================================", " [NARRATIVE] ");
  for (int i = 0; i < trip_directions.maneuver_size(); ++i) {
    const auto& maneuver = trip_directions.maneuver(i);

    // Depart instruction
    if (!maneuver.depart_instruction().empty()) {
      valhalla::midgard::logging::Log((boost::format("   %s") % maneuver.depart_instruction()).str(),
                                      " [NARRATIVE] ");
    }

    // Verbal depart instruction
    if (!maneuver.verbal_depart_instruction().empty()) {
      valhalla::midgard::logging::Log((boost::format("   VERBAL_DEPART: %s") %
                                       maneuver.verbal_depart_instruction())
                                          .str(),
                                      " [NARRATIVE] ");
    }

    // Instruction
    valhalla::midgard::logging::Log((boost::format("%d: %s | %.1f %s") % m %
                                     maneuver.text_instruction() % maneuver.length() % units)
                                        .str(),
                                    " [NARRATIVE] ");

    // Turn lanes
    // Only for driving and no start/end maneuvers
    if ((maneuver.travel_mode() == valhalla::TravelMode::kDrive) &&
        !((maneuver.type() == valhalla::DirectionsLeg_Maneuver_Type_kStart) ||
          (maneuver.type() == valhalla::DirectionsLeg_Maneuver_Type_kStartRight) ||
          (maneuver.type() == valhalla::DirectionsLeg_Maneuver_Type_kStartLeft) ||
          (maneuver.type() == valhalla::DirectionsLeg_Maneuver_Type_kDestination) ||
          (maneuver.type() == valhalla::DirectionsLeg_Maneuver_Type_kDestinationRight) ||
          (maneuver.type() == valhalla::DirectionsLeg_Maneuver_Type_kDestinationLeft))) {
      auto prev_edge = etl.GetPrevEdge(maneuver.begin_path_index());
      if (prev_edge && (prev_edge->turn_lanes_size() > 0)) {
        std::string turn_lane_status = "ACTIVE_TURN_LANES";
        if (prev_edge->HasNonDirectionalTurnLane()) {
          turn_lane_status = "NON_DIRECTIONAL_TURN_LANES";
        } else if (!prev_edge->HasActiveTurnLane()) {
          turn_lane_status = "NO_ACTIVE_TURN_LANES";
        }
        valhalla::midgard::logging::Log((boost::format("   %d: TURN_LANES: %s %s") % m %
                                         prev_edge->TurnLanesToString() % turn_lane_status)
                                            .str(),
                                        " [NARRATIVE] ");
      }
    }

    // Verbal succinct transition instruction
    if (!maneuver.verbal_succinct_transition_instruction().empty()) {
      valhalla::midgard::logging::Log((boost::format("   VERBAL_SUCCINCT: %s") %
                                       maneuver.verbal_succinct_transition_instruction())
                                          .str(),
                                      " [NARRATIVE] ");
    }

    // Verbal transition alert instruction
    if (!maneuver.verbal_transition_alert_instruction().empty()) {
      valhalla::midgard::logging::Log((boost::format("   VERBAL_ALERT: %s") %
                                       maneuver.verbal_transition_alert_instruction())
                                          .str(),
                                      " [NARRATIVE] ");
    }

    // Verbal pre transition instruction
    if (!maneuver.verbal_pre_transition_instruction().empty()) {
      valhalla::midgard::logging::Log((boost::format("   VERBAL_PRE: %s") %
                                       maneuver.verbal_pre_transition_instruction())
                                          .str(),
                                      " [NARRATIVE] ");
    }

    // Verbal post transition instruction
    if (!maneuver.verbal_post_transition_instruction().empty()) {
      valhalla::midgard::logging::Log((boost::format("   VERBAL_POST: %s") %
                                       maneuver.verbal_post_transition_instruction())
                                          .str(),
                                      " [NARRATIVE] ");
    }

    // Arrive instruction
    if (!maneuver.arrive_instruction().empty()) {
      valhalla::midgard::logging::Log((boost::format("   %s") % maneuver.arrive_instruction()).str(),
                                      " [NARRATIVE] ");
    }

    // Verbal arrive instruction
    if (!maneuver.verbal_arrive_instruction().empty()) {
      valhalla::midgard::logging::Log((boost::format("   VERBAL_ARRIVE: %s") %
                                       maneuver.verbal_arrive_instruction())
                                          .str(),
                                      " [NARRATIVE] ");
    }

    // All turn lanes along maneuver
    if (verbose_lanes) {
      for (auto n = maneuver.begin_path_index() + 1; n < maneuver.end_path_index(); ++n) {
        auto prev_edge = etl.GetPrevEdge(n);
        auto q = n - maneuver.begin_path_index();
        if (prev_edge) {
          std::string turn_lane_status = "ACTIVE_TURN_LANES";
          if (prev_edge->HasNonDirectionalTurnLane()) {
            turn_lane_status = "NON_DIRECTIONAL_TURN_LANES";
          } else if (prev_edge->turn_lanes_size() == 0) {
            turn_lane_status = "NO_TURN_LANES";
          } else if (!prev_edge->HasActiveTurnLane()) {
            turn_lane_status = "NO_ACTIVE_TURN_LANES";
          }
          valhalla::midgard::logging::Log((boost::format("   %d-%d: TURN_LANES: %s %s") % m % q %
                                           prev_edge->TurnLanesToString() % turn_lane_status)
                                              .str(),
                                          " [NARRATIVE] ");
        }
      }
    }

    if (i < trip_directions.maneuver_size() - 1) {
      valhalla::midgard::logging::Log("----------------------------------------------",
                                      " [NARRATIVE] ");
    }

    // Increment maneuver number
    ++m;
  }
  valhalla::midgard::logging::Log("==============================================", " [NARRATIVE] ");
  valhalla::midgard::logging::Log("Total time: " + GetFormattedTime(trip_directions.summary().time()),
                                  " [NARRATIVE] ");
  valhalla::midgard::logging::Log((boost::format("Total length: %.1f %s") %
                                   trip_directions.summary().length() % units)
                                      .str(),
                                  " [NARRATIVE] ");
  if (origin.date_time_) {
    valhalla::midgard::logging::Log("Departed at: " + *origin.date_time_, " [NARRATIVE] ");
  }
  if (destination.date_time_) {
    valhalla::midgard::logging::Log("Arrived at: " + *destination.date_time_, " [NARRATIVE] ");
  }
  data.setTripTime(trip_directions.summary().time());
  data.setTripDist(trip_directions.summary().length());
  data.setManeuvers(trip_directions.maneuver_size());
  data.setElapsedCostSeconds(etl.node().rbegin()->cost().elapsed_cost().seconds());
  data.setElapsedCostCost(etl.node().rbegin()->cost().elapsed_cost().cost());

  return trip_directions;
}

// Main method for testing a single path
int main(int argc, char* argv[]) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  std::string json_str, json_file;
  boost::property_tree::ptree config;

  bool match_test, verbose_lanes;
  bool multi_run = false;
  uint32_t iterations;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_VERSION + "\n\n"
      "a command line test tool for shortest path routing.\n"
      "Use the -j option for specifying the locations and costing method and options.");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("j,json", "JSON Example: "
        "'{\"locations\":[{\"lat\":40.748174,\"lon\":-73.984984,\"type\":\"break\",\"heading\":200,"
        "\"name\":\"Empire State Building\",\"street\":\"350 5th Avenue\",\"city\":\"New "
        "York\",\"state\":\"NY\",\"postal_code\":\"10118-0110\",\"country\":\"US\"},{\"lat\":40."
        "749231,\"lon\":-73.968703,\"type\":\"break\",\"name\":\"United Nations "
        "Headquarters\",\"street\":\"405 East 42nd Street\",\"city\":\"New "
        "York\",\"state\":\"NY\",\"postal_code\":\"10017-3507\",\"country\":\"US\"}],\"costing\":"
        "\"auto\",\"directions_options\":{\"units\":\"miles\"}}'", cxxopts::value<std::string>())
      ("json-file", "File containing the JSON query", cxxopts::value<std::string>())
      ("match-test", "Test RouteMatcher with resulting shape.", cxxopts::value<bool>(match_test)->default_value("false"))
      ("multi-run", "Generate the route N additional times before exiting.", cxxopts::value<uint32_t>(iterations)->default_value("1"))
      ("verbose-lanes", "Include verbose lanes output in DirectionsTest.", cxxopts::value<bool>(verbose_lanes)->default_value("false"))
      ("c,config", "Valhalla configuration file", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, config, "mjolnir.logging"))
      return EXIT_SUCCESS;

    if (iterations > 1) {
      multi_run = true;
    }

    if (result.count("json-file") && result.count("json")) {
      LOG_WARN("json and json-file option are set, using json-file content");
    } else if (result.count("json-file")) {
      std::ifstream ifs(result["json-file"].as<std::string>());
      json_str.assign((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
    } else if (result.count("json")) {
      json_str = result["json"].as<std::string>();
    } else {
      throw cxxopts::exceptions::exception("Either json or json-file args must be set.");
    }
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << std::endl;
    return EXIT_FAILURE;
  }

  // Grab the directions options, if they exist
  valhalla::Api request;
  valhalla::ParseApi(json_str, valhalla::Options::route, request);
  const auto& options = request.options();

  // Get type of route - this provides the costing method to use.
  const std::string& routetype = valhalla::Costing_Enum_Name(options.costing_type());
  LOG_INFO("routetype: " + routetype);

  // Locations
  auto locations = valhalla::baldr::PathLocation::fromPBF(options.locations());
  if (locations.size() < 2) {
    throw;
  }

  // Something to hold the statistics
  uint32_t n = locations.size() - 1;
  PathStatistics data({locations[0].latlng_.lat(), locations[0].latlng_.lng()},
                      {locations[n].latlng_.lat(), locations[n].latlng_.lng()});
  // Crow flies distance between locations (km)
  float d1 = 0.0f;
  for (uint32_t i = 0; i < n; i++) {
    d1 += locations[i].latlng_.Distance(locations[i + 1].latlng_) * kKmPerMeter;
  }
  // Get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));

  // Get the maximum distance for time dependent routes
  float max_timedep_distance =
      config.get<float>("service_limits.max_timedep_distance", kDefaultMaxTimeDependentDistance);

  auto t0 = std::chrono::high_resolution_clock::now();

  // Construct costing
  CostFactory factory;
  // Get the costing method - pass the JSON configuration
  TravelMode mode;
  auto mode_costing = factory.CreateModeCosting(options, mode);

  // Find path locations (loki) for sources and targets
  auto tw0 = std::chrono::high_resolution_clock::now();
  loki_worker_t lw(config);
  auto tw1 = std::chrono::high_resolution_clock::now();
  auto msw = std::chrono::duration_cast<std::chrono::milliseconds>(tw1 - tw0).count();
  LOG_INFO("Location Worker construction took " + std::to_string(msw) + " ms");

  auto tl0 = std::chrono::high_resolution_clock::now();
  lw.route(request);
  auto tl1 = std::chrono::high_resolution_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tl1 - tl0).count();
  LOG_INFO("Location Processing took " + std::to_string(ms) + " ms");

  // Get the route
  BidirectionalAStar bd(config.get_child("thor"));
  MultiModalPathAlgorithm mm(config.get_child("thor"));
  TimeDepForward timedep_forward(config.get_child("thor"));
  TimeDepReverse timedep_reverse(config.get_child("thor"));
  MarkupFormatter markup_formatter(config);
  for (uint32_t i = 0; i < n; i++) {
    // Set origin and destination for this segment
    valhalla::Location origin = options.locations(i);
    valhalla::Location dest = options.locations(i + 1);

    PointLL ll1(origin.ll().lng(), origin.ll().lat());
    PointLL ll2(dest.ll().lng(), dest.ll().lat());

    // Choose path algorithm
    PathAlgorithm* pathalgorithm;
    if (routetype == "multimodal") {
      pathalgorithm = &mm;
    } else {
      // Use time dependent algorithms if date time is present
      // TODO - this isn't really correct for multipoint routes but should allow
      // simple testing.
      if (options.has_date_time_case() && ll1.Distance(ll2) < max_timedep_distance &&
          (options.date_time_type() == valhalla::Options_DateTimeType_depart_at ||
           options.date_time_type() == valhalla::Options_DateTimeType_current)) {
        pathalgorithm = &timedep_forward;
      } else if (options.date_time_type() == valhalla::Options_DateTimeType_arrive_by &&
                 ll1.Distance(ll2) < max_timedep_distance) {
        pathalgorithm = &timedep_reverse;
      } else {
        // Use bidirectional except for trivial cases (same edge or connected edges)
        pathalgorithm = &bd;
        for (auto& edge1 : origin.correlation().edges()) {
          for (auto& edge2 : dest.correlation().edges()) {
            if (edge1.graph_id() == edge2.graph_id() ||
                reader.AreEdgesConnected(GraphId(edge1.graph_id()), GraphId(edge2.graph_id()))) {
              pathalgorithm = &timedep_forward;
            }
          }
        }
      }
    }
    bool using_bd = pathalgorithm == &bd;

    // Get the best path
    const valhalla::TripLeg* trip_path = nullptr;
    try {
      trip_path = PathTest(reader, origin, dest, pathalgorithm, mode_costing, mode, data, multi_run,
                           iterations, using_bd, match_test, routetype, request);
    } catch (std::runtime_error& rte) { LOG_ERROR("trip_path not found"); }

    // If successful get directions
    if (trip_path && trip_path->node_size() != 0) {

      // Write the path.pbf if requested
      if (get_env("SAVE_PATH_PBF") == "true") {
        std::string path_bytes = request.SerializeAsString();
        std::string pbf_filename = "path.pbf";
        LOG_INFO("Writing TripPath to " + pbf_filename + " with size " +
                 std::to_string(path_bytes.size()));
        std::ofstream output_pbf(pbf_filename, std::ios::out | std::ios::trunc | std::ios::binary);
        if (output_pbf.is_open() && path_bytes.size() > 0) {
          output_pbf.write(&path_bytes[0], path_bytes.size());
          output_pbf.close();
        } else {
          std::cerr << "Failed to write " << pbf_filename << std::endl;
          return EXIT_FAILURE;
        }
      }

      // Try the the directions
      auto t1 = std::chrono::high_resolution_clock::now();
      const auto& trip_directions =
          DirectionsTest(request, origin, dest, data, verbose_lanes, markup_formatter);
      auto t2 = std::chrono::high_resolution_clock::now();
      auto msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

      auto trip_time = trip_directions.summary().time();
      auto trip_length = trip_directions.summary().length() * 1609.344f;
      LOG_INFO("trip_processing_time (ms)::" + std::to_string(msecs));
      LOG_INFO("trip_time (secs)::" + std::to_string(trip_time));
      LOG_INFO("trip_length (meters)::" + std::to_string(trip_length));
      data.setSuccess("success");
    } else {
      // Route was unsuccessful
      data.setSuccess("fail_no_route");
    }
  }

  // Set the arc distance. Convert to miles if needed
  if (options.units() == valhalla::Options::miles) {
    d1 *= kMilePerKm;
  }
  data.setArcDist(d1);

  // Time all stages for the stats file: location processing,
  // path computation, trip path building, and directions
  auto t2 = std::chrono::high_resolution_clock::now();
  auto msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0).count();
  LOG_INFO("Total time= " + std::to_string(msecs) + " ms");
  data.addRuntime(msecs);
  data.log();

  // Shutdown protocol buffer library
  google::protobuf::ShutdownProtobufLibrary();

  return EXIT_SUCCESS;
}
