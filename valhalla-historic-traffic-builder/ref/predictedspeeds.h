#ifndef VALHALLA_BALDR_PREDICTEDSPEEDS_H_
#define VALHALLA_BALDR_PREDICTEDSPEEDS_H_

#include <array>
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace baldr {

constexpr uint32_t kSpeedBucketSizeMinutes = 5;
constexpr uint32_t kSpeedBucketSizeSeconds = kSpeedBucketSizeMinutes * 60;
constexpr uint32_t kBucketsPerWeek = (7 * 24 * 60) / kSpeedBucketSizeMinutes;

// Length of transformed speed buckets array.
constexpr uint32_t kCoefficientCount = 200;

// Expected size of base64-encoded predicted speeds coefficients. Each int16_t coefficient is
// encoded by two bytes in an array of uint8_t's.
constexpr uint32_t kDecodedSpeedSize = 2 * kCoefficientCount;

/**
 * Compress speed buckets by truncating its DCT-II transform.
 * @param speeds    Array of speed values for each bucket (must be 2016 values).
 * @return  Transformed values.
 */
std::array<int16_t, kCoefficientCount> compress_speed_buckets(const float* speeds);

/**
 * Recover speed value in the bucket (apply DCT-III transform)
 * @param coefficients  Transformed speed buckets (must be 200 values).
 * @param bucket_idx    Index of the bucket we want to recover.
 * @return  Speed value (in KPH) in the bucket.
 */
float decompress_speed_bucket(const int16_t* coefficients, uint32_t bucket_idx);

/**
 * Pack transformed speed values into base64-encoded string.
 * @param coefficients  Array of transformed speed buckets (must be 200 values).
 * @return  Encoded string.
 */
std::string encode_compressed_speeds(const int16_t* coefficients);

/**
 * Decode base64-encoded string and recover transformed speed buckets. Throw an exception on fail.
 * @param encoded   base64-encoded string (length must be equal to 400).
 * @return  Transformed speed buckets.
 */
std::array<int16_t, kCoefficientCount> decode_compressed_speeds(const std::string& encoded);

/**
 * Class to access predicted speed information within a tile.
 */
class PredictedSpeeds {
public:
  /**
   * Constructor.
   */
  PredictedSpeeds() : offset_(nullptr), profiles_(nullptr) {
  }

  /**
   * Set a pointer to the offset data within the GraphTile.
   * @param  offset Pointer to the offset array in the GraphTile.
   * @param  profiles Pointer to the profiles array in the GraphTile.
   */
  void set_offset(const uint32_t* offset) {
    offset_ = offset;
  }

  /**
   * Set a pointer to the speed profile data within the GraphTile.
   * @param  profiles Pointer to the profiles array in the GraphTile.
   */
  void set_profiles(const int16_t* profiles) {
    profiles_ = profiles;
  }

  /**
   * Get the speed given the edge Id and the seconds of the week.
   * @param  idx  Directed edge index.
   * @param  seconds_of_week  Seconds from start of the week (local time).
   */
  float speed(const uint32_t idx, const uint32_t seconds_of_week) const {
    // Get a pointer to the compressed speed profile for this edge. Assume the edge Id is valid
    // (otherwise an exception would be thrown when getting the directed edge) and the profile
    // offset is valid. If there is no predicted speed profile this method will not be called due
    // to DirectedEdge::has_predicted_speed being false.
    const int16_t* coefficients = profiles_ + offset_[idx];

    return decompress_speed_bucket(coefficients, seconds_of_week / kSpeedBucketSizeSeconds);
  }

protected:
  const uint32_t* offset_;  // Offset into the array of compressed speed profiles
                            // for each directed edge
  const int16_t* profiles_; // Compressed speed profiles
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_PREDICTEDSPEEDS_H_
