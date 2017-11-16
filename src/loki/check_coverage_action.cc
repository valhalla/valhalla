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

json::MapPtr serialize(const PathLocation& location, const std::unordered_set<int32_t>& ids) {
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

    const TileLevel& loki_worker_t::init_check_coverage(rapidjson::Document& request) {
      size_t level = GetFromRapidJson<size_t>(request, "level", -1);
      auto l = TileHierarchy::levels().find(level);
      if(l == TileHierarchy::levels().end())
        throw valhalla_exception_t{115};

      locations = parse_locations(request, "locations");
      if(locations.size() < 1)
        throw valhalla_exception_t{120};

      return l->second;
    }

    json::ArrayPtr loki_worker_t::check_coverage(rapidjson::Document& request) {
      auto level = init_check_coverage(request);
      auto json = json::array({});

      for (const auto& location : locations) {
        // Get a list of tiles required within the radius of the projected point
        const auto& ll = location.latlng_;
        DistanceApproximator approximator(ll);
        float latdeg = (location.radius_ / kMetersPerDegreeLat);
        float lngdeg = (location.radius_ / DistanceApproximator::MetersPerLngDegree(ll.lat()));
        AABB2<PointLL> bbox(Point2(ll.lng() - lngdeg, ll.lat() - latdeg),
                            Point2(ll.lng() + lngdeg, ll.lat() + latdeg));
        std::vector<int32_t> tilelist = level.tiles.TileList(bbox);
        std::unordered_set<int32_t> covered;
        for (auto id : tilelist)
          if (connectivity_map->get_color(GraphId(id, 3, 0)) != 0 )
            covered.emplace(id);
        json->emplace_back(serialize(location, covered));
      }

      return json;
    }
  }
}
