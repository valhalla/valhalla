#include <unordered_set>

#include "baldr/connectivity_map.h"
#include "loki/worker.h"
#include "midgard/distanceapproximator.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace loki {

void loki_worker_t::init_transit_available(Api& request) {
  if (request.options().locations_size() < 1) {
    throw valhalla_exception_t{120};
  };
}

std::string loki_worker_t::transit_available(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  init_transit_available(request);
  auto locations = PathLocation::fromPBF(request.options().locations());
  std::unordered_set<baldr::Location> found;
  try {
    const auto& tiles = TileHierarchy::levels().back().tiles;
    for (const auto& location : locations) {
      // Get a list of tiles required within the radius of the projected point
      const auto& ll = location.latlng_;
      DistanceApproximator<PointLL> approximator(ll);
      double latdeg = double(location.radius_) / kMetersPerDegreeLat;
      double lngdeg = location.radius_ / DistanceApproximator<PointLL>::MetersPerLngDegree(ll.lat());
      AABB2<PointLL> bbox(Point2d(ll.lng() - lngdeg, ll.lat() - latdeg),
                          Point2d(ll.lng() + lngdeg, ll.lat() + latdeg));
      std::vector<int32_t> tilelist = tiles.TileList(bbox);
      for (auto id : tilelist) {
        // transit is level hierarchy level 3
        auto color = connectivity_map->get_color(GraphId(id, 3, 0));
        if (color != 0) {
          found.emplace(location);
        }
      }
    }
  } catch (const std::exception&) { throw valhalla_exception_t{170}; }

  return tyr::serializeTransitAvailable(request, locations, found);
}

} // namespace loki
} // namespace valhalla
