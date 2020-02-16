#include <algorithm>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/json.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "odin/enhancedtrippath.h"
#include "odin/util.h"
#include "thor/attributes_controller.h"
#include "thor/match_result.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/trip.pb.h>

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
constexpr size_t kTripLegIndex = 3;
} // namespace

namespace valhalla {
namespace thor {
/*
 * The trace_attributes action takes a GPS trace or latitude, longitude positions
 * from a portion of an existing route and returns detailed attribution along the
 * portion of the route. This includes details for each section of road along the
 * path as well as any intersections along the path.
 */
std::string thor_worker_t::trace_attributes(Api& request) {

  // Parse request
  parse_locations(request);
  parse_costing(request);
  parse_measurements(request);
  parse_filter_attributes(request, true);
  const auto& options = *request.mutable_options();

  /*
   * A flag indicating whether the input shape is a GPS trace or exact points from a
   * prior route run against the Valhalla road network.  Knowing that the input is from
   * Valhalla will allow an efficient “edge-walking” algorithm rather than a more extensive
   * map-matching method. If true, this enforces to only use exact route match algorithm.
   */

  std::vector<std::tuple<float, float, std::vector<thor::MatchResult>>> map_match_results;

  switch (options.shape_match()) {
    // If the exact points from a prior route that was run against the Valhalla road network,
    // then we can traverse the exact shape to form a path by using edge-walking algorithm
    case ShapeMatch::edge_walk:
      try {
        route_match(request);
        map_match_results.emplace_back(1.0f, 0.0f, std::vector<thor::MatchResult>{});
      } catch (const std::exception& e) {
        throw valhalla_exception_t{
            443, ShapeMatch_Enum_Name(options.shape_match()) +
                     " algorithm failed to find exact route match.  Try using "
                     "shape_match:'walk_or_snap' to fallback to map-matching algorithm"};
      }
      break;
    // If non-exact shape points are used, then we need to correct this shape by sending them
    // through the map-matching algorithm to snap the points to the correct shape
    case ShapeMatch::map_snap:
      try {
        map_match_results = map_match(request);
      } catch (const std::exception& e) {
        throw valhalla_exception_t{
            444, ShapeMatch_Enum_Name(options.shape_match()) +
                     " algorithm failed to snap the shape points to the correct shape."};
      }
      break;
    // If we think that we have the exact shape but there ends up being no Valhalla route match,
    // then we want to fallback to try and use meili map matching to match to local route
    // network. No shortcuts are used and detailed information at every intersection becomes
    // available.
    case ShapeMatch::walk_or_snap:
      try {
        route_match(request);
        map_match_results.emplace_back(1.0f, 0.0f, std::vector<thor::MatchResult>{});
      } catch (...) {
        LOG_WARN(ShapeMatch_Enum_Name(options.shape_match()) +
                 " algorithm failed to find exact route match; Falling back to map_match...");
        try {
          map_match_results = map_match(request);
        } catch (const std::exception& e) {
          throw valhalla_exception_t{
              444, ShapeMatch_Enum_Name(options.shape_match()) +
                       " algorithm failed to snap the shape points to the correct shape."};
        }
      }
      break;
  }

  return tyr::serializeTraceAttributes(request, controller, map_match_results);
}

} // namespace thor
} // namespace valhalla
