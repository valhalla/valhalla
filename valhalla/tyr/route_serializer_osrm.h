#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "baldr/json.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"
#include "midgard/polyline2.h"
#include "midgard/util.h"
#include "odin/enhancedtrippath.h"
#include "odin/util.h"
#include "tyr/route_summary_cache.h"
#include "tyr/serializer_constants.h"
#include "tyr/serializers.h"
#include "worker.h"

#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"
#include "proto_conversions.h"
#ifdef INLINE_TEST
#include "test.h"
#endif

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::tyr;
using namespace std;

#if 0
namespace {
const std::string kSignElementDelimiter = ", ";
const std::string kDestinationsDelimiter = ": ";
const std::string kSpeedLimitSignVienna = "vienna";
const std::string kSpeedLimitSignMutcd = "mutcd";
const std::string kSpeedLimitUnitsKph = "km/h";
const std::string kSpeedLimitUnitsMph = "mph";

constexpr std::size_t MAX_USED_SEGMENTS = 2;

struct Coordinate {
  std::int32_t lng;
  std::int32_t lat;

  Coordinate(const std::int32_t lng_, const std::int32_t lat_) : lng(lng_), lat(lat_) {
  }
};

inline std::int32_t toFixed(const float floating) {
  const auto d = static_cast<double>(floating);
  const auto fixed = static_cast<std::int32_t>(std::round(d * ENCODE_PRECISION));
  return fixed;
}

inline double toFloating(const std::int32_t fixed) {
  const auto i = static_cast<std::int32_t>(fixed);
  const auto floating = static_cast<double>(i) * DECODE_PRECISION;
  return floating;
}

const constexpr double TILE_SIZE = 256.0;
static constexpr unsigned MAX_ZOOM = 18;
static constexpr unsigned MIN_ZOOM = 1;
// this is an upper bound to current display sizes
static constexpr double VIEWPORT_WIDTH = 8 * TILE_SIZE;
static constexpr double VIEWPORT_HEIGHT = 5 * TILE_SIZE;
static double INV_LOG_2 = 1. / std::log(2);
const constexpr double DEGREE_TO_RAD = 0.017453292519943295769236907684886;
const constexpr double RAD_TO_DEGREE = 1. / DEGREE_TO_RAD;
const constexpr double EPSG3857_MAX_LATITUDE = 85.051128779806592378; // 90(4*atan(exp(pi))/pi-1)

const constexpr PointLL::first_type DOUGLAS_PEUCKER_THRESHOLDS[19] = {
    703125.0, // z0
    351562.5, // z1
    175781.2, // z2
    87890.6,  // z3
    43945.3,  // z4
    21972.6,  // z5
    10986.3,  // z6
    5493.1,   // z7
    2746.5,   // z8
    1373.2,   // z9
    686.6,    // z10
    343.3,    // z11
    171.6,    // z12
    85.8,     // z13
    42.9,     // z14
    21.4,     // z15
    10.7,     // z16
    5.3,      // z17
    2.6,      // z18
};

inline double clamp(const double lat) {
  return std::max(std::min(lat, double(EPSG3857_MAX_LATITUDE)), double(-EPSG3857_MAX_LATITUDE));
}

inline double latToY(const double latitude) {
  // apparently this is the (faster) version of the canonical log(tan()) version
  const auto clamped_latitude = clamp(latitude);
  const double f = std::sin(DEGREE_TO_RAD * static_cast<double>(clamped_latitude));
  return RAD_TO_DEGREE * 0.5 * std::log((1 + f) / (1 - f));
}

inline double lngToPixel(double lon, unsigned zoom) {
  const double shift = (1u << zoom) * TILE_SIZE;
  const double b = shift / 2.0;
  const double x = b * (1 + static_cast<double>(lon) / 180.0);
  return x;
}

inline double latToPixel(double lat, unsigned zoom) {
  const double shift = (1u << zoom) * TILE_SIZE;
  const double b = shift / 2.0;
  const double y = b * (1. - latToY(lat) / 180.);
  return y;
}

inline unsigned getFittedZoom(Coordinate south_west, Coordinate north_east) {
  const auto min_x = lngToPixel(toFloating(south_west.lng), MAX_ZOOM);
  const auto max_y = latToPixel(toFloating(south_west.lat), MAX_ZOOM);
  const auto max_x = lngToPixel(toFloating(north_east.lng), MAX_ZOOM);
  const auto min_y = latToPixel(toFloating(north_east.lat), MAX_ZOOM);
  const double width_ratio = (max_x - min_x) / VIEWPORT_WIDTH;
  const double height_ratio = (max_y - min_y) / VIEWPORT_HEIGHT;
  const auto zoom = MAX_ZOOM - std::max(std::log(width_ratio), std::log(height_ratio)) * INV_LOG_2;

  if (std::isfinite(zoom))
    return std::max<unsigned>(MIN_ZOOM, zoom);
  else
    return MIN_ZOOM;
}

// Sign style and unit conventions for speed limit signs by country.
// Most countries use Vienna style and km/h, but the countries below
// use MUTCD and/or mph conventions.
std::unordered_map<std::string, std::pair<std::string, std::string>> speed_limit_info = {
    {"AG", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"AI", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"AS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"BS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"BZ", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"CA", {kSpeedLimitSignMutcd, kSpeedLimitUnitsKph}},
    {"DM", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"FK", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GB", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GD", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GG", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"GU", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"IM", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"JE", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"KN", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"KY", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"LC", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"LR", {kSpeedLimitSignMutcd, kSpeedLimitUnitsKph}},
    {"MP", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"MS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"PR", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"SH", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"TC", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"US", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"VC", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"VG", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
    {"VI", {kSpeedLimitSignMutcd, kSpeedLimitUnitsMph}},
    {"WS", {kSpeedLimitSignVienna, kSpeedLimitUnitsMph}},
};

} // namespace
#endif
namespace osrm_serializers {
/*
OSRM output is described in: http://project-osrm.org/docs/v5.5.1/api/
{
    "code":"Ok"
    "waypoints": [{ }, { }...],
    "routes": [
        {
            "geometry":"....."
            "distance":xxx.y
            "duration":yyy.z
            "legs":[
                {
                    "steps":[
                        "intersections":[
                        ]
                        "geometry":" "
                        "maneuver":{
                        }
                    ]
                }
            ]
        },
        ...
    ]
}
*/

// Serialize route response in OSRM compatible format.
// Inputs are:
//     directions options
//     TripLeg protocol buffer
//     DirectionsLeg protocol buffer
std::string serialize(valhalla::Api& api);

} // namespace osrm_serializers
