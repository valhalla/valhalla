#ifndef VALHALLA_GEO_CONSTANTS_H_
#define VALHALLA_GEO_CONSTANTS_H_

#include <math.h>

namespace valhalla{
namespace geo{

// Time constants
constexpr float kSecPerHour = 3600.0f;
constexpr float kHourPerSec = 1.0f / 3600.0f;

// Distance constants
constexpr float kMilePerKm      = 1.609344f;
constexpr float kKmPerMile      = 1.0f / kMilePerKm;
constexpr float kRadEarthKm     = 6378.160187;
constexpr float kKmPerDegreeLat = 110.567f;

// Angular measures
constexpr float kPi        = 3.14159265f;
constexpr float kPiOver2   = (kPi * 0.5f);
constexpr float kPiOver4   = (kPi * 0.25f);
constexpr float kDegPerRad = (180.0f / kPi);  // Radians to degrees conversion
constexpr float kRadPerDeg = (kPi / 180.0f);  // Degrees to radians conversion
constexpr float kEpsilon   = 0.000001f;

}
}
#endif  // VALHALLA_GEO_CONSTANTS_H_

