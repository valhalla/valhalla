#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "midgard/util.h"
#include "midgard/pointll.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "midgard/encoded.h"
#include "baldr/json.h"
#include "baldr/turn.h"
#include "baldr/rapidjson_utils.h"
#include "exception.h"
#include "odin/util.h"
#include "proto/directions_options.pb.h"
#include "tyr/serializers.h"


using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::tyr;
using namespace std;

namespace osrm {

  valhalla::baldr::json::MapPtr waypoint(const valhalla::odin::Location& location) {
    // Create a waypoint to add to the array
    auto waypoint = json::map({});

    // Output location as a lon,lat array. Note this is the projected
    // lon,lat on the nearest road.
    PointLL input_ll(location.ll().lng(), location.ll().lat());
    PointLL proj_ll(location.projected_ll().lng(), location.projected_ll().lat());
    auto loc = json::array({});
    loc->emplace_back(json::fp_t{proj_ll.lng(), 6});
    loc->emplace_back(json::fp_t{proj_ll.lat(), 6});
    waypoint->emplace("location", loc);

    // Add street name.
    waypoint->emplace("street", location.name());

    // Add distance in meters from the input location to the nearest
    // point on the road used in the route
    float distance = input_ll.Distance(proj_ll);
    waypoint->emplace("distance", json::fp_t{distance, 1});

    // Add hint. Goal is for the hint returned from a locate request to be able
    // to quickly find the edge and point along the edge in a route request.
    // Defer this - not currently used in OSRM.
    waypoint->emplace("hint", std::string("TODO"));

    return waypoint;
  }

  // Serialize locations (called waypoints in OSRM). Waypoints are described here:
  //     http://project-osrm.org/docs/v5.5.1/api/#waypoint-object
  json::ArrayPtr waypoints(const google::protobuf::RepeatedPtrField<valhalla::odin::Location>& locations){
    auto waypoints = json::array({});
    for (const auto& location : locations)
      waypoints->emplace_back(waypoint(location));
    return waypoints;
  }
}

