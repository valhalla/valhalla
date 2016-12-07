#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/json.h>
#include <valhalla/baldr/errorcode_util.h>
#include <valhalla/proto/trippath.pb.h>


#include "thor/service.h"
#include "thor/trip_path_controller.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::odin;
using namespace valhalla::thor;


namespace {
  const headers_t::value_type CORS { "Access-Control-Allow-Origin", "*" };
  const headers_t::value_type JSON_MIME { "Content-type", "application/json;charset=utf-8" };
  const headers_t::value_type JS_MIME { "Content-type", "application/javascript;charset=utf-8" };

  json::MapPtr serialize(odin::TripPath trip_path, const boost::optional<std::string>& id, double scale) {
    json::ArrayPtr edges = json::array({});
    for (int i = 1; i < trip_path.node().size(); i++) {
      if (trip_path.node(i-1).has_edge()) {
        const auto& edge = trip_path.node(i - 1).edge();
        auto names = json::array({});
        for (const auto& name : edge.name())
          names->push_back(name);

        edges->emplace_back(json::map({
          {"max_downward_grade", static_cast<int64_t>(edge.max_downward_grade())},
          {"max_upward_grade", static_cast<int64_t>(edge.max_upward_grade())},
          {"weighted_grade", json::fp_t{edge.weighted_grade(), 3}},
          {"length", json::fp_t{edge.length() * scale, 3}},
          {"speed", json::fp_t{edge.speed() * scale, 3}},
          {"way_id", static_cast<uint64_t>(edge.way_id())},
          {"id", static_cast<uint64_t>(edge.id())},
          {"names", names}
        }));
      }
    }
    auto json = json::map({
      {"edges", edges}
    });
    if (id)
      json->emplace("id", *id);
    return json;
  }
}

namespace valhalla {
namespace thor {

/*
 * The trace_attributes action takes a GPS trace or latitude, longitude positions
 * from a portion of an existing route and returns detailed attribution along the
 * portion of the route. This includes details for each section of road along the
 * path as well as any intersections along the path.
 */
worker_t::result_t thor_worker_t::trace_attributes(
    const boost::property_tree::ptree &request,
    const std::string &request_str, http_request_t::info_t& request_info) {
  //get time for start of request
  auto s = std::chrono::system_clock::now();

  // Parse request
  parse_locations(request);
  parse_shape(request);
  parse_costing(request);
  parse_trace_config(request);
  /*
   * A flag indicating whether the input shape is a GPS trace or exact points from a
   * prior route run against the Valhalla road network.  Knowing that the input is from
   * Valhalla will allow an efficient “edge-walking” algorithm rather than a more extensive
   * map-matching method. If true, this enforces to only use exact route match algorithm.
   */
  bool exact_match = request.get<bool>("exact_match_only", false);

  TripPathController controller;
  // TODO parse include/exclude and set controller as needed - for now just default

  // If the exact points from a prior route that was run agains the Valhalla road network,
  //then we can traverse the exact shape to form a path by using edge-walking algorithm
  odin::TripPath trip_path = route_match(controller);
  if (trip_path.node().size() == 0) {
    if (!exact_match) {
      //If no Valhalla route match, then use meili map matching to match to local route network.
      //No shortcuts are used and detailed information at every intersection becomes available.
      LOG_INFO("Could not find exact route match; Sending trace to map_match...");
      try {
        trip_path = map_match(controller);
      } catch (...) {
        valhalla_exception_t{400, 444};
      }
    } else throw valhalla_exception_t{400, 443};
  }
  auto id = request.get_optional<std::string>("id");
  //length and speed default to km
  double scale = 1;
  auto units = request.get<std::string>("units", "km");
  if ((units == "mi") || (units == "miles"))
    scale = kMilePerKm;

  //serialize output to Thor
  json::MapPtr json = serialize(trip_path, id, scale);

  //jsonp callback if need be
  std::ostringstream stream;
  auto jsonp = request.get_optional<std::string>("jsonp");
  if (jsonp)
    stream << *jsonp << '(';
  stream << *json;
  if (jsonp)
    stream << ')';

  // Get processing time for thor
  auto e = std::chrono::system_clock::now();
  std::chrono::duration<float, std::milli> elapsed_time = e - s;
  // TODO determine what to log
  //log request if greater than X (ms)
  if (!request_info.do_not_track && (elapsed_time.count() / correlated.size()) > long_request) {
    LOG_WARN("thor::trace_attributes elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
    LOG_WARN("thor::trace_attributes exceeded threshold::"+ request_str);
    midgard::logging::Log("valhalla_thor_long_request_trace_attributes", " [ANALYTICS] ");
  }
  http_response_t response(200, "OK", stream.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
  response.from_info(request_info);
  worker_t::result_t result{false};
  result.messages.emplace_back(response.to_string());
  return result;
}
}
}
