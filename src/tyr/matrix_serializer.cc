#include <cstdint>

#include "baldr/json.h"
#include "thor/costmatrix.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::thor;


namespace osrm_serializers {

  /*json::MapPtr waypoints(const baldr::PathLocation& correlated) {
    auto waypoint = json::map({});
    auto loc = json::array({});
    loc->emplace_back(json::fp_t{correlated.latlng_.lat(), 6});
    loc->emplace_back(json::fp_t{correlated.latlng_.lng(), 6});

    waypoint->emplace("location", loc);
    waypoint->emplace("name", std::string(correlated.name_));
    waypoint->emplace("hint", std::string("shape"));
    return waypoint;
  }
*/

  json::ArrayPtr waypoints(const std::vector<valhalla::odin::Location>& locations){
    int index = 0;
    auto waypoints = json::array({});
    for (const auto& location : locations) {

      index = 1;

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

      // Add the waypoint to the JSON array
      waypoints->emplace_back(waypoint);
    }
    return waypoints;
  }
  json::ArrayPtr serialize_duration(const std::vector<TimeDistance>& tds, size_t start_td, const size_t td_count) {
    auto time = json::array({});
    for(size_t i = start_td; i < start_td + td_count; ++i) {
     //check to make sure a route was found; if not, return null for time in matrix result
     if (tds[i].time != kMaxCost) {
       time->emplace_back(static_cast<uint64_t>(tds[i].time));
     } else {
       time->emplace_back(static_cast<std::nullptr_t>(nullptr));
     }
    }
    return time;
  }

  json::ArrayPtr serialize_distance(const std::vector<TimeDistance>& tds,
     size_t start_td, const size_t td_count, const size_t source_index, const size_t target_index, double distance_scale) {
    auto distance = json::array({});
    for(size_t i = start_td; i < start_td + td_count; ++i) {
      //check to make sure a route was found; if not, return null for distance in matrix result
      if (tds[i].time != kMaxCost) {
        distance->emplace_back(json::fp_t{tds[i].dist * distance_scale, 3});
      } else {
        distance->emplace_back(static_cast<std::nullptr_t>(nullptr));
      }
    }
    return distance;
   }

  // Serialize route response in OSRM compatible format.
  json::MapPtr serialize(const valhalla_request_t& request, const std::vector<TimeDistance>& time_distances, double distance_scale) {
    auto json = json::map({});
    auto time = json::array({});
    auto distance = json::array({});

    // If here then the matrix succeeded. Set status code to OK and serialize
    // waypoints (locations).
    json->emplace("code", std::string("Ok"));
    json->emplace("sources", osrm::waypoints(request.options.sources()));
    json->emplace("destinations", osrm::waypoints(request.options.targets()));

    for(size_t source_index = 0; source_index < request.options.sources_size(); ++source_index) {
      time->emplace_back(serialize_duration(time_distances, source_index * request.options.targets_size(), request.options.targets_size()));
      distance->emplace_back(serialize_distance(time_distances, source_index * request.options.targets_size(), request.options.targets_size(),
          source_index, 0, distance_scale));
    }
    json->emplace("durations", time);
    json->emplace("distances", distance);
    return json;
  }
}

namespace valhalla_serializers {

  /*
  valhalla output looks like this:

  */

json::ArrayPtr locations(const google::protobuf::RepeatedPtrField<odin::Location>& correlated) {
    auto input_locs = json::array({});
    for(size_t i = 0; i < correlated.size(); i++) {
      input_locs->emplace_back(
        json::map({
          {"lat", json::fp_t{correlated.Get(i).ll().lat(), 6}},
          {"lon", json::fp_t{correlated.Get(i).ll().lng(), 6}}
        })
      );
    }
    return input_locs;
  }

  json::ArrayPtr serialize_row(const std::vector<TimeDistance>& tds,
      size_t start_td, const size_t td_count, const size_t source_index, const size_t target_index, double distance_scale) {
    auto row = json::array({});
    for(size_t i = start_td; i < start_td + td_count; ++i) {
      //check to make sure a route was found; if not, return null for distance & time in matrix result
      if (tds[i].time != kMaxCost) {
        row->emplace_back(json::map({
          {"from_index", static_cast<uint64_t>(source_index)},
          {"to_index", static_cast<uint64_t>(target_index + (i - start_td))},
          {"time", static_cast<uint64_t>(tds[i].time)},
          {"distance", json::fp_t{tds[i].dist * distance_scale, 3}}
        }));
      } else {
        row->emplace_back(json::map({
          {"from_index", static_cast<uint64_t>(source_index)},
          {"to_index", static_cast<uint64_t>(target_index + (i - start_td))},
          {"time", static_cast<std::nullptr_t>(nullptr)},
          {"distance", static_cast<std::nullptr_t>(nullptr)}
        }));
      }
    }
    return row;
  }

  json::MapPtr serialize(const valhalla_request_t& request, const std::vector<TimeDistance>& time_distances, double distance_scale) {
    json::ArrayPtr matrix = json::array({});
    for(size_t source_index = 0; source_index < request.options.sources_size(); ++source_index) {
        matrix->emplace_back(
          serialize_row(time_distances, source_index * request.options.targets_size(), request.options.targets_size(),
                        source_index, 0, distance_scale));
    }
    auto json = json::map({
      {"sources_to_targets", matrix},
      {"units", odin::DirectionsOptions::Units_Name(request.options.units())},
    });
    json->emplace("targets", json::array({locations(request.options.targets())}));
    json->emplace("sources", json::array({locations(request.options.sources())}));

    if (request.options.has_id())
      json->emplace("id", request.options.id());
    return json;
  }
}

namespace valhalla {
  namespace tyr {

    std::string serializeMatrix(const valhalla_request_t& request, const std::vector<TimeDistance>& time_distances, double distance_scale) {

      auto json = request.options.format() == odin::DirectionsOptions::osrm ?
      osrm_serializers::serialize(request, time_distances, distance_scale) :
      valhalla_serializers::serialize(request, time_distances, distance_scale);

      std::stringstream ss;
      ss << *json;
      return ss.str();
    }

  }
}
