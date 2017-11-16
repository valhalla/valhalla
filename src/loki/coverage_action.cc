#include "loki/worker.h"

#include <unordered_set>
#include <cstdint>

#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/connectivity_map.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace {

json::MapPtr serialize(const PathLocation& location, const std::unordered_set<GraphId>& ids) {
  //put the ids in an array
  json::ArrayPtr covered_ids = json::array({});
  for(auto id : ids)
    covered_ids->push_back(static_cast<uint64_t>(id));

  //serialze all the edges
  auto json = json::map
  ({
    {"input_lat", json::fp_t{location.latlng_.lat(), 6}},
    {"input_lon", json::fp_t{location.latlng_.lng(), 6}},
    {"radius", static_cast<uint64_t>(location.radius_)},
    {"coverage", covered_ids},
  });
  return json;
}

}

namespace valhalla {
  namespace loki {

    std::vector<const baldr::TileLevel*> loki_worker_t::init_coverage(rapidjson::Document& request) {
      std::vector<const baldr::TileLevel*> levels;
      const auto& transit_level = TileHierarchy::GetTransitLevel();
      auto request_levels = GetOptionalFromRapidJson<rapidjson::Value::ConstArray>(request, "/levels");
      if (request_levels) {
        for(const auto& request_level : *request_levels) {
          if(!request_level.IsInt())
            continue;
          auto level = request_level.GetInt();
          auto l = TileHierarchy::levels().find(level);
          if(l != TileHierarchy::levels().end())
            levels.emplace_back(&l->second);
          else if(level == transit_level.level)
            levels.emplace_back(&transit_level);
        }
      }
      if(levels.empty())
        throw valhalla_exception_t{115};

      locations = parse_locations(request, "locations");
      if(locations.size() < 1)
        throw valhalla_exception_t{120};

      return levels;
    }

    json::ArrayPtr loki_worker_t::coverage(rapidjson::Document& request) {
      auto levels = init_coverage(request);
      auto json = json::array({});

      for (const auto& location : locations) {
        // Get a list of tiles required within the radius of the projected point
        const auto& ll = location.latlng_;
        DistanceApproximator approximator(ll);
        float latdeg = (location.radius_ / kMetersPerDegreeLat);
        float lngdeg = (location.radius_ / DistanceApproximator::MetersPerLngDegree(ll.lat()));
        AABB2<PointLL> bbox(Point2(ll.lng() - lngdeg, ll.lat() - latdeg),
                            Point2(ll.lng() + lngdeg, ll.lat() + latdeg));
        std::unordered_set<GraphId> covered;
        for(const auto& level : levels) {
          std::vector<int32_t> tilelist = level->tiles.TileList(bbox);
          for (auto id : tilelist) {
            GraphId gid(id, level->level, 0);
            if (connectivity_map->get_color(gid) != 0 )
              covered.emplace(gid);
          }
        }
        json->emplace_back(serialize(location, covered));
      }

      return json;
    }
  }
}
