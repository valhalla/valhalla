#ifndef VALHALLA_MIDGARD_CONSTANTS_H_
#define VALHALLA_MIDGARD_CONSTANTS_H_

#include <math.h>
#include <stdint.h>

namespace valhalla {
namespace midgard {

// Time constants
constexpr float kSecPerMinute = 60.0f;
constexpr float kMinPerSec = 1.0f / 60.0f;
constexpr float kSecPerHour = 3600.0f;
constexpr float kHourPerSec = 1.0f / 3600.0f;
constexpr double kSecPerMillisecond = 0.001;
constexpr double kMillisecondPerSec = 1000;
constexpr uint32_t kSecondsPerMinute = 60;
constexpr uint32_t kSecondsPerHour = 3600;
constexpr uint32_t kSecondsPerDay = 86400;
constexpr uint32_t kSecondsPerWeek = 604800;

// Distance constants
constexpr float kFeetPerMile = 5280.0f;
constexpr float kMilePerFoot = 1.0f / kFeetPerMile;
constexpr float kFeetPerMeter = 3.2808399f;
constexpr float kMetersPerKm = 1000.0f;
constexpr float kKmPerMeter = 0.001f;
constexpr float kMilePerKm = 0.621371f;
constexpr float kMilePerMeter = kMilePerKm / 1000;
constexpr float kKmPerMile = 1.609344f;
constexpr float kRadEarthMeters = 6378160.187f;
constexpr double kMetersPerDegreeLat = 110567.0f;
constexpr double kKmPerDecimeter = 0.0001;
constexpr double kMeterPerDecimeter = 0.1;
constexpr double kDecimeterPerMeter = 10;

// Speed conversion constants
constexpr float kMPHtoMetersPerSec = 0.44704f;
constexpr double kDecimeterPerSectoKPH = 0.36; // dm/s to km/h
constexpr double kKPHtoMetersPerSec = 1000. / 3600.;
constexpr double kMetersPerSectoKPH = 3600. / 1000.;

// Angular measures
constexpr float kPi = 3.14159265f;
constexpr double kPiD = 3.14159265358979323846264338327950288;
constexpr float kPiOver2 = (kPi * 0.5f);
constexpr float kPiOver4 = (kPi * 0.25f);
constexpr float kDegPerRad = (180.0f / kPi);   // Radians to degrees conversion
constexpr double kDegPerRadD = (180.0 / kPiD); // Radians to degrees conversion in double precision
constexpr float kRadPerDeg = (kPi / 180.0f);   // Degrees to radians conversion
constexpr double kRadPerDegD = (kPiD / 180.0); // Degrees to radians conversion in double precision
constexpr float kEpsilon = 0.000001f;

// To avoid using M_PI
constexpr double kPiDouble = 3.14159265358979323846;

// Weight measures
constexpr float kTonsShortToMetric = 0.907f; // Short tons to metric

} // namespace midgard
} // namespace valhalla
#endif // VALHALLA_MIDGARD_CONSTANTS_H_
