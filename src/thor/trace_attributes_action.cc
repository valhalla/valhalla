#include <cstdint>
#include <algorithm>
#include <tuple>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "baldr/json.h"
#include "baldr/graphconstants.h"
#include "baldr/directededge.h"
#include "baldr/edge_elevation.h"
#include "midgard/logging.h"
#include "midgard/constants.h"
#include "odin/util.h"
#include "odin/enhancedtrippath.h"
#include "proto/tripdirections.pb.h"
#include "proto/trippath.pb.h"
#include "exception.h"
#include "thor/attributes_controller.h"
#include "thor/match_result.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::thor;

namespace {
  // <Confidence score, raw score, match results, trip path> tuple indexes
  constexpr size_t kConfidenceScoreIndex = 0;
  constexpr size_t kRawScoreIndex = 1;
  constexpr size_t kMatchResultsIndex = 2;
  constexpr size_t kTripPathIndex = 3;
}

namespace valhalla {
namespace thor {

void thor_worker_t::filter_attributes(const valhalla_request_t& request, AttributesController& controller) {
  auto filter_action = rapidjson::get<std::string>(request.document, "/filters/action", "");

  if (filter_action == "include") {
    controller.disable_all();
    for (const auto& v : rapidjson::get<rapidjson::Value::ConstArray>(request.document, "/filters/attributes"))
      controller.attributes.at(v.GetString()) = true;
  } else if (filter_action == "exclude") {
    controller.enable_all();
    for (const auto& v : rapidjson::get<rapidjson::Value::ConstArray>(request.document, "/filters/attributes"))
      controller.attributes.at(v.GetString()) = false;
  } else {
    controller.enable_all();
  }
}

/*
 * The trace_attributes action takes a GPS trace or latitude, longitude positions
 * from a portion of an existing route and returns detailed attribution along the
 * portion of the route. This includes details for each section of road along the
 * path as well as any intersections along the path.
 */
std::string thor_worker_t::trace_attributes(valhalla_request_t& request) {

  // Parse request
  parse_locations(request);
  parse_costing(request);
  parse_trace_config(request);
  parse_measurements(request);

  /*
   * A flag indicating whether the input shape is a GPS trace or exact points from a
   * prior route run against the Valhalla road network.  Knowing that the input is from
   * Valhalla will allow an efficient “edge-walking” algorithm rather than a more extensive
   * map-matching method. If true, this enforces to only use exact route match algorithm.
   */
  odin::TripPath trip_path;
  std::vector<std::tuple<float, float, std::vector<thor::MatchResult>, odin::TripPath>> map_match_results;
  AttributesController controller;
  filter_attributes(request, controller);
  auto shape_match = STRING_TO_MATCH.find(rapidjson::get<std::string>(request.document, "/shape_match", "walk_or_snap"));
  if (shape_match == STRING_TO_MATCH.cend())
    throw valhalla_exception_t{445};
  else {
    // If the exact points from a prior route that was run against the Valhalla road network,
    // then we can traverse the exact shape to form a path by using edge-walking algorithm
  switch (shape_match->second) {
      case EDGE_WALK:
        try {
          trip_path = route_match(request, controller);
          if (trip_path.node().size() == 0)
            throw std::exception{};
          map_match_results.emplace_back(1.0f, 0.0f, std::vector<thor::MatchResult>{}, trip_path);
        } catch (const std::exception& e) {
          throw valhalla_exception_t{443, shape_match->first + " algorithm failed to find exact route match.  Try using shape_match:'walk_or_snap' to fallback to map-matching algorithm"};
        }
        break;
      // If non-exact shape points are used, then we need to correct this shape by sending them
      // through the map-matching algorithm to snap the points to the correct shape
      case MAP_SNAP:
        try {
          uint32_t best_paths = rapidjson::get<uint32_t>(request.document, "/best_paths", 1);
          map_match_results = map_match(controller, true, best_paths);
        } catch (const std::exception& e) {
          throw valhalla_exception_t{444, shape_match->first + " algorithm failed to snap the shape points to the correct shape."};
        }
        break;
      // If we think that we have the exact shape but there ends up being no Valhalla route match,
      // then we want to fallback to try and use meili map matching to match to local route network.
      // No shortcuts are used and detailed information at every intersection becomes available.
      case WALK_OR_SNAP:
        trip_path = route_match(request, controller);
        if (trip_path.node().size() == 0) {
          LOG_WARN(shape_match->first + " algorithm failed to find exact route match; Falling back to map_match...");
          try {
            map_match_results = map_match(controller, true);
          } catch (const std::exception& e) {
            throw valhalla_exception_t{444, shape_match->first + " algorithm failed to snap the shape points to the correct shape."};
          }
        } else {
          map_match_results.emplace_back(1.0f, 0.0f, std::vector<thor::MatchResult>{}, trip_path);
        }
        break;
    }
  }

  if(map_match_results.empty()
      || std::get<kTripPathIndex>(map_match_results.at(0)).node().size() == 0)
    throw valhalla_exception_t { 442 };
  return tyr::serializeTraceAttributes(request, controller, map_match_results);
}
}
}
