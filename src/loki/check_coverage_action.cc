#include "loki/worker.h"
#include "loki/search.h"
#include <functional>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <sstream>

#include "baldr/json.h"
#include "baldr/pathlocation.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/connectivity_map.h"
#include "midgard/logging.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace {

  json::MapPtr serialize(const PathLocation& location, int level, bool istransit) {
    //serialze all the edges
    auto json = json::map
    ({
      {"input_lat", json::fp_t{location.latlng_.lat(), 6}},
      {"input_lon", json::fp_t{location.latlng_.lng(), 6}},
      {"radius", location.radius_},
      {"level", static_cast<int64_t>(level)}
    });
    json->emplace("istransit", istransit);
    return json;
  }
}

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_check_coverage(rapidjson::Document& request) {
      //strip off unused information
      locations = parse_locations(request, "locations");

      if(request.HasMember("costing"))
        parse_costing(request);
    }

    json::ArrayPtr loki_worker_t::check_coverage(rapidjson::Document& request) {
      init_check_coverage(request);

      // Validate optional trace options
      const auto level = GetOptionalFromRapidJson<int>(request, "/level").get_value_or(-1);
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
            auto color = connectivity_map->get_color(GraphId(id, level, 0));
            if (level == 3 && color != 0) {
              istransit = true;
              break;
            } else if (level == 3 && color == 0) {
              istransit = false;
            }//for now we only check level 3 transit coverage; can add more in future
            else throw valhalla_exception_t { 172 };
          }
          json->emplace_back(serialize(location, level, istransit));
        }
      } catch (const std::exception&) {
        throw valhalla_exception_t { 172 };
      }
      return json;
    }

  }
}
