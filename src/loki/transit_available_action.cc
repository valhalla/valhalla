#include "loki/worker.h"
#include <unordered_set>
#include <cstdint>

#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/connectivity_map.h"
#include "midgard/logging.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace {

  json::MapPtr serialize(const PathLocation& location, bool istransit) {
    //serialze all the edges
    auto json = json::map
    ({
      {"input_lat", json::fp_t{location.latlng_.lat(), 6}},
      {"input_lon", json::fp_t{location.latlng_.lng(), 6}},
      {"radius", static_cast<uint64_t>(location.radius_)}
    });
    json->emplace("istransit", istransit);
    return json;
  }
}

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_transit_available(rapidjson::Document& request) {
      locations = parse_locations(request, "locations");
      if(locations.size() < 1)
        throw valhalla_exception_t{120};

      if(request.HasMember("costing"))
        parse_costing(request);
    }

    json::ArrayPtr loki_worker_t::transit_available(rapidjson::Document& request) {
      init_transit_available(request);
      auto json = json::array({});
      try{
        const auto& tiles = TileHierarchy::levels().find(TileHierarchy::levels().rbegin()->first)->second.tiles;

        for (const auto& location : locations) {
          // Get a list of tiles required within the radius of the projected point
          const auto& ll = location.latlng_;
          DistanceApproximator approximator(ll);
          float latdeg = (location.radius_ / kMetersPerDegreeLat);
          float lngdeg = (location.radius_ / DistanceApproximator::MetersPerLngDegree(ll.lat()));
          AABB2<PointLL> bbox(Point2(ll.lng() - lngdeg, ll.lat() - latdeg),
                              Point2(ll.lng() + lngdeg, ll.lat() + latdeg));
          std::vector<int32_t> tilelist = tiles.TileList(bbox);
          bool istransit = false;
          for (auto id : tilelist) {
            // transit is level hierarchy level 3
            auto color = connectivity_map->get_color(GraphId(id, 3, 0));
            if (color != 0) {
              istransit = true;
              break;
            }
          }
          json->emplace_back(serialize(location, istransit));
        }
      } catch (const std::exception&) {
        throw valhalla_exception_t { 170 };
      }
      return json;
    }

  }
}
