#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/turn.h"
#include "midgard/aabb2.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "odin/util.h"
#include "tyr/serializers.h"

#include <valhalla/proto/options.pb.h>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::tyr;
using namespace std;

namespace {
midgard::PointLL to_ll(const LatLng& ll) {
  return midgard::PointLL{ll.lng(), ll.lat()};
}
} // namespace

namespace osrm {

// Serialize a location (waypoint) in OSRM compatible format. Waypoint format is described here:
//     http://project-osrm.org/docs/v5.5.1/api/#waypoint-object
valhalla::baldr::json::MapPtr waypoint(const valhalla::Location& location,
                                       bool is_tracepoint,
                                       const bool is_optimized,
                                       const int64_t waypoint_index) {
  // Create a waypoint to add to the array
  auto waypoint = json::map({});

  // Output location as a lon,lat array. Note this is the projected
  // lon,lat on the nearest road.
  auto loc = json::array({});
  loc->emplace_back(json::fp_t{location.path_edges(0).ll().lng(), 6});
  loc->emplace_back(json::fp_t{location.path_edges(0).ll().lat(), 6});
  waypoint->emplace("location", loc);

  // Add street name.
  std::string name = location.path_edges_size() && location.path_edges(0).names_size()
                         ? location.path_edges(0).names(0)
                         : "";
  waypoint->emplace("name", name);

  // Add distance in meters from the input location to the nearest
  // point on the road used in the route
  // TODO: since distance was normalized in thor - need to recalculate here
  //       in the future we shall have store separately from score
  waypoint->emplace("distance",
                    json::fp_t{to_ll(location.ll()).Distance(to_ll(location.path_edges(0).ll())), 3});

  // If the location was used for a tracepoint we trigger extra serialization
  if (is_tracepoint) {
    waypoint->emplace("alternatives_count", static_cast<uint64_t>(location.path_edges_size() - 1));
    if (waypoint_index > -1) {
      waypoint->emplace("waypoint_index", static_cast<int64_t>(waypoint_index));
    } else {
      waypoint->emplace("waypoint_index", static_cast<std::nullptr_t>(nullptr));
    }
    waypoint->emplace("matchings_index",
                      static_cast<uint64_t>(0)); // we only have one matching for now
  }

  // If the location was used for optimized route we add trips_index and waypoint
  // index (index of the waypoint in the trip)
  if (is_optimized) {
    int trips_index = 0; // TODO
    waypoint->emplace("trips_index", static_cast<uint64_t>(trips_index));
    waypoint->emplace("waypoint_index", static_cast<uint64_t>(waypoint_index));
  }

  return waypoint;
}

// Serialize locations (called waypoints in OSRM). Waypoints are described here:
//     http://project-osrm.org/docs/v5.5.1/api/#waypoint-object
json::ArrayPtr waypoints(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                         bool is_tracepoint) {
  auto waypoints = json::array({});
  int64_t waypoint_index = -1;
  for (const auto& location : locations) {
    if (location.path_edges().size() == 0) {
      waypoints->emplace_back(static_cast<std::nullptr_t>(nullptr));
    } else if (location.type() == valhalla::Location::kBreak ||
               location.type() == valhalla::Location::kBreakThrough) {
      waypoints->emplace_back(waypoint(location, is_tracepoint, false, ++waypoint_index));
    } else {
      waypoints->emplace_back(waypoint(location, is_tracepoint));
    }
  }
  return waypoints;
}

json::ArrayPtr waypoints(const valhalla::Trip& trip) {
  auto waypoints = json::array({});
  // For multi-route the same waypoints are used for all routes.
  const auto& legs = trip.routes(0).legs();
  for (const auto& leg : legs) {
    for (const auto& location : leg.location()) {
      if (&location == &leg.location(0) && &leg != &*legs.begin()) {
        continue;
      }
      waypoints->emplace_back(waypoint(location, false));
    }
  }
  return waypoints;
}

} // namespace osrm
