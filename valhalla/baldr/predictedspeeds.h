#ifndef VALHALLA_BALDR_PREDICTEDSPEEDS_H_
#define VALHALLA_BALDR_PREDICTEDSPEEDS_H_

#include <valhalla/midgard/util.h>
#include <vector>

namespace valhalla {
namespace baldr {

constexpr uint32_t kSpeedBucketSizeMinutes = 5;
constexpr uint32_t kSpeedBucketSizeSeconds = kSpeedBucketSizeMinutes * 60;
constexpr uint32_t kBucketsPerWeek = (7 * 24 * 60) / kSpeedBucketSizeMinutes;
constexpr float kSecondsPerWeek = 7.0f * 24.0f * 60.0f * 60.0f;
constexpr float kPiConstant = 3.14159265f / static_cast<float>(kBucketsPerWeek);

// DTC-III constants for speed decoding and normalization
constexpr uint32_t kCoefficientCount = 200;
constexpr float k1OverSqrt2 = 1.0f / sqrtf(2.0f);
constexpr float kPiBucketConstant = 3.14159265f / 2016.0f;
constexpr float kSpeedNormalization = sqrtf(2.0f / 2016.0f);

/**
 * Class to access predicted speed information within a tile.
 */
class PredictedSpeeds {
public:
  /**
   * Constructor.
   */
  PredictedSpeeds() : index_(nullptr), profiles_(nullptr) {
  }

  /**
   * Set a pointer to the index data within the GraphTile.
   * @param  index_ Pointer to the index array in the GraphTile.
   * @param  profiles Pointer to the profiles array in the GraphTile.
   */
  void set_index(const uint32_t* index) {
    index_ = index;
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
  uint32_t speed(const uint32_t idx, const uint32_t seconds_of_week) const {
    // Get a pointer to the compressed speed profile for this edge. Assume the edge Id is valid
    // (otherwise an exception would be thrown when getting the directed edge) and the profile
    // index is valid. If there is no predicted speed profile this method will not be called due
    // to DirectedEdge::predicted_speed being false.
    const int16_t* coefficients = profiles_ + (kCoefficientCount * index_[idx]);

    // Compute the time bucket
    int bucket = (seconds_of_week / kSpeedBucketSizeSeconds);

    // DTC-III with some speed normalization
    float b = kPiBucketConstant * (bucket + 0.5f);
    float speed = coefficients[0] * k1OverSqrt2;
    for (int k = 1; k < kCoefficientCount; k++) {
      speed += coefficients[k] * cosf(b * k);
    }
    return speed * kSpeedNormalization;
  }

protected:
  const uint32_t* index_; // Index into the array of compressed speed profiles for each directed edge
  const int16_t* profiles_; // Compressed speed profiles
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_PREDICTEDSPEEDS_H_
