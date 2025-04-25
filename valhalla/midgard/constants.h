#ifndef VALHALLA_MIDGARD_CONSTANTS_H_
#define VALHALLA_MIDGARD_CONSTANTS_H_

#include <math.h>
#include <stdint.h>

namespace valhalla {
namespace midgard {

// Time constants
inline constexpr float kSecPerMinute = 60.0f;
inline constexpr float kMinPerSec = 1.0f / 60.0f;
inline constexpr float kSecPerHour = 3600.0f;
inline constexpr float kHourPerSec = 1.0f / 3600.0f;
inline constexpr double kSecPerMillisecond = 0.001;
inline constexpr double kMillisecondPerSec = 1000;
inline constexpr uint32_t kSecondsPerMinute = 60;
inline constexpr uint32_t kSecondsPerHour = 3600;
inline constexpr uint32_t kSecondsPerDay = 86400;
inline constexpr uint32_t kSecondsPerWeek = 604800;

// Distance constants
inline constexpr float kFeetPerMile = 5280.0f;
inline constexpr float kMilePerFoot = 1.0f / kFeetPerMile;
inline constexpr float kFeetPerMeter = 3.2808399f;
inline constexpr float kMetersPerKm = 1000.0f;
inline constexpr float kKmPerMeter = 0.001f;
inline constexpr float kMilePerKm = 0.621371f;
inline constexpr float kMilePerMeter = kMilePerKm / 1000;
inline constexpr float kKmPerMile = 1.609344f;
inline constexpr float kRadEarthMeters = 6378160.187f;
inline constexpr double kMetersPerDegreeLat = 110567.0f;
inline constexpr double kKmPerDecimeter = 0.0001;
inline constexpr double kMeterPerDecimeter = 0.1;
inline constexpr double kDecimeterPerMeter = 10;

// Speed conversion constants
inline constexpr float kMPHtoMetersPerSec = 0.44704f;
inline constexpr double kDecimeterPerSectoKPH = 0.36; // dm/s to km/h
inline constexpr double kKPHtoMetersPerSec = 1000. / 3600.;
inline constexpr double kMetersPerSectoKPH = 3600. / 1000.;

// Angular measures
inline constexpr float kPi = 3.14159265f;
inline constexpr double kPiD = 3.14159265358979323846264338327950288;
inline constexpr float kPiOver2 = (kPi * 0.5f);
inline constexpr float kPiOver4 = (kPi * 0.25f);
inline constexpr float kDegPerRad = (180.0f / kPi);   // Radians to degrees conversion
inline constexpr double kDegPerRadD = (180.0 / kPiD); // Radians to degrees conversion in double precision
inline constexpr float kRadPerDeg = (kPi / 180.0f);   // Degrees to radians conversion
inline constexpr double kRadPerDegD = (kPiD / 180.0); // Degrees to radians conversion in double precision
inline constexpr float kEpsilon = 0.000001f;

// To avoid using M_PI
inline constexpr double kPiDouble = 3.14159265358979323846;

// Weight measures
inline constexpr float kTonsShortToMetric = 0.907f; // Short tons to metric

} // namespace midgard
} // namespace valhalla
#endif // VALHALLA_MIDGARD_CONSTANTS_H_
