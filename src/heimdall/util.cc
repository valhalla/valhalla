#include "heimdall/util.h"
#include "midgard/constants.h"

#include <cmath>

using namespace valhalla::midgard;

namespace valhalla {
namespace heimdall {

midgard::AABB2<midgard::PointLL> tile_to_bbox(const uint32_t z, const uint32_t x, const uint32_t y) {
  const double n = std::pow(2.0, z);

  double min_lon = x / n * 360.0 - 180.0;
  double max_lon = (x + 1) / n * 360.0 - 180.0;

  double min_lat_rad = std::atan(std::sinh(kPiD * (1 - 2 * (y + 1) / n)));
  double max_lat_rad = std::atan(std::sinh(kPiD * (1 - 2 * y / n)));

  double min_lat = min_lat_rad * 180.0 / kPiD;
  double max_lat = max_lat_rad * 180.0 / kPiD;

  return AABB2<PointLL>(PointLL(min_lon, min_lat), PointLL(max_lon, max_lat));
}

double lon_to_merc_x(const double lon) {
  return kEarthRadiusMeters * lon * kPiD / 180.0;
}

double lat_to_merc_y(const double lat) {
  return kEarthRadiusMeters * std::log(std::tan(kPiD / 4.0 + lat * kPiD / 360.0));
}

} // namespace heimdall
} // namespace valhalla
