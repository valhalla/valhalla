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

#include <valhalla/proto/directions_options.pb.h>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::tyr;
using namespace std;

namespace osrm {

// Serialize a location (waypoint) in OSRM compatible format. Waypoint format is described here:
//     http://project-osrm.org/docs/v5.5.1/api/#waypoint-object
valhalla::baldr::json::MapPtr waypoint(const odin::Location& location,
                                       bool tracepoint,
                                       const bool optimized,
                                       const uint32_t waypoint_index) {
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
  waypoint->emplace("distance", json::fp_t{location.path_edges(0).distance(), 3});

  // Add hint. Goal is for the hint returned from a locate request to be able
  // to quickly find the edge and point along the edge in a route request.
  // Defer this - not currently used in OSRM.
  waypoint->emplace("hint", std::string("TODO"));

  // If the location was used for a tracepoint we trigger extra serialization
  if (tracepoint) {
    waypoint->emplace("alternatives_count", static_cast<uint64_t>(location.path_edges_size() - 1));
    waypoint->emplace("waypoint_index", static_cast<uint64_t>(location.original_index()));
    waypoint->emplace("matchings_index",
                      static_cast<uint64_t>(0)); // we only have one matching for now
  }

  // If the location was used for optimized route we add trips_index and waypoint
  // index (index of the waypoint in the trip)
  if (optimized) {
    int trips_index = 0; // TODO
    waypoint->emplace("trips_index", static_cast<uint64_t>(trips_index));
    waypoint->emplace("waypoint_index", static_cast<uint64_t>(waypoint_index));
  }

  return waypoint;
}

// Serialize locations (called waypoints in OSRM). Waypoints are described here:
//     http://project-osrm.org/docs/v5.5.1/api/#waypoint-object
json::ArrayPtr waypoints(const google::protobuf::RepeatedPtrField<odin::Location>& locations,
                         bool tracepoints) {
  auto waypoints = json::array({});
  for (const auto& location : locations) {
    waypoints->emplace_back(waypoint(location, tracepoints));
  }
  return waypoints;
}

} // namespace osrm
