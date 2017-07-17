#ifndef VALHALLA_MIDGARD_CONSTANTS_H_
#define VALHALLA_MIDGARD_CONSTANTS_H_

#include <math.h>
 #include <stdint.h>

namespace valhalla {
namespace midgard {

// Time constants
constexpr float kSecPerMinute     = 60.0f;
constexpr float kMinPerSec        = 1.0f / 60.0f;
constexpr float kSecPerHour       = 3600.0f;
constexpr float kHourPerSec       = 1.0f / 3600.0f;
constexpr uint32_t kSecondsPerDay = 86400;

// Distance constants
constexpr float kFeetPerMeter       = 3.2808399f;
constexpr float kMetersPerKm        = 1000.0f;
constexpr float kKmPerMeter         = 0.001f;
constexpr float kMilePerKm          = 0.621371f;
constexpr float kKmPerMile          = 1.60934f;
constexpr float kRadEarthMeters     = 6378160.187f;
constexpr float kMetersPerDegreeLat = 110567.0f;

// Speed conversion constants
constexpr float kMPHtoMetersPerSec  = 0.44704f;

// Angular measures
constexpr float kPi        = 3.14159265f;
constexpr float kPiOver2   = (kPi * 0.5f);
constexpr float kPiOver4   = (kPi * 0.25f);
constexpr float kDegPerRad = (180.0f / kPi);  // Radians to degrees conversion
constexpr float kRadPerDeg = (kPi / 180.0f);  // Degrees to radians conversion
constexpr float kEpsilon   = 0.000001f;

// To avoid using M_PI
constexpr double kPiDouble = 3.14159265358979323846;

// Weight measures
constexpr float kTonsShortToMetric = 0.907f;  // Short tons to metric

}
}
#endif  // VALHALLA_MIDGARD_CONSTANTS_H_

