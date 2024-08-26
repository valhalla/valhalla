#ifndef VALHALLA_MIDGARD_ELEVATION_ENCODING_H_
#define VALHALLA_MIDGARD_ELEVATION_ENCODING_H_

#include <cmath>
#include <type_traits>
#include <vector>

#include <valhalla/midgard/logging.h>

namespace valhalla {
namespace midgard {

// Maximum sampling interval used when encoding elevation along an edge.
const uint32_t kMaxEdgeElevationSampling = 32;

// NO_DATA elevation value (occurs where no elevation data exists, e.g. high latitudes)
constexpr int32_t ELEVATION_NO_DATA_VALUE = -32768;

// Use fixed precision (0.25 degrees)
const double kElevationPrecision = 0.25f;
const double kInvElevationPrecision = 4.0;
inline int32_t to_fixed_precision(const double v) {
  return static_cast<int32_t>((v * kInvElevationPrecision) + 0.5);
}
inline float from_fixed_precision(const int8_t v) {
  return (v * kElevationPrecision);
}

/**
 * Get the sampling interval to use along an edge of a given length. To allow
 * consistent encoding forward and reverse along an edge we use a sampling
 * interval that breaks the edge into an integral number of samples.
 * @return Returns the sampling interval in meters.
 */
inline double sampling_interval(const double length) {
  uint32_t sample_count = length / kMaxEdgeElevationSampling;
  return length / static_cast<double>(sample_count + 1);
}

/**
 * Get the count of encoded elevations given the edge length. Computes the
 * desired sampling interval (not to exceed kMaxEdgeElevationSampling) and
 * computes the number of vertices. Excludes the first and last (these are
 * not encoded).
 * @param length Edge length.
 * @return Returns the number of encoded elevations.
 */
inline uint32_t encoded_elevation_count(const uint32_t length) {
  return static_cast<uint32_t>(std::round(length / sampling_interval(length))) - 1;
}

/**
 * Encode elevation. Uses delta/offset encoding - storing the delta with fixed
 * precision (0.25m). Does not encode the first or last shape point (these are
 * stored in NodeInfo).
 * @param elevation Elevation sampled along an edge.
 * @param error Boolean value indicating if an error occurred (clamped a value
 *              that would have exceeded 1 byte). This is returned by this
 *              argument to the calling method.
 * @return Returns encoding of elevation along the edge. Can be an empty
 *         vector if the elevation vector size is <= 2 (length of the edge
 *         is less than the max interval).
 */
inline std::vector<int8_t> encode_elevation(const std::vector<double>& elevation, bool& error) {
  // Return empty encoded vector if first or last value is NO_DATA
  std::vector<int8_t> encoding;
  if (elevation.front() == ELEVATION_NO_DATA_VALUE || elevation.back() == ELEVATION_NO_DATA_VALUE) {
    return encoding;
  }

  // First elevation is stored in NodeInfo...skip it but round it to nearest int
  int prior = to_fixed_precision(elevation[0]);

  for (uint32_t i = 1; i < elevation.size() - 1; ++i) {
    // Special case for NO_DATA values
    if (elevation[i] == ELEVATION_NO_DATA_VALUE) {
      encoding.push_back(0);
      continue;
    }

    // Convert to fixed precision offset. Clamp to 1 byte values. It should be
    // very rare that the limit is exceeded, but if it is we try to make up for
    // any shortfall on subsequent postings.
    auto v = to_fixed_precision(elevation[i]);
    auto delta = v - prior;
    if (delta > 127) {
      if (delta > 256) {
        error = true;
      }
      delta = 127;
      prior += 127;
    } else if (delta < -128) {
      if (delta < -256) {
        error = true;
      }
      delta = -128;
      prior -= 128;
    } else {
      prior = v;
    }
    encoding.push_back(static_cast<int8_t>(delta));
  }
  return encoding;
}

/**
 * Decode elevation. The start and end elevation (at the end nodes of the edge) are
 * provided.
 * @param encoded  Encoded elevation offsets.
 * @param h1       Elevation at the start.
 * @param h2       Elevation at the end.
 * @return Returns elevation along the edge.
 */
inline std::vector<float>
decode_elevation(std::vector<int8_t>& encoded, const float h1, const float h2, const bool forward) {
  // Allocate elevation vector. Swap h1 and h2 if the edge is not forward
  std::vector<float> elevation(encoded.size() + 2);
  elevation.front() = (forward) ? h1 : h2;
  elevation.back() = forward ? h2 : h1;

  // Decode in forward direction.
  for (uint32_t i = 0; i < encoded.size(); ++i) {
    elevation[i + 1] = elevation[i] + from_fixed_precision(encoded[i]);
  }

  // Reverse if the direction is not forward.
  if (!forward) {
    std::reverse(elevation.begin(), elevation.end());
  }
  return elevation;
}

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_ELEVATION_ENCODING_H_
