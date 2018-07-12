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
constexpr uint32_t kSpeedBucketCount = 200;
constexpr float kPiConstant = 3.14159265f / static_cast<float>(kBucketsPerWeek);

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
    const int16_t* speeds = profiles_ + (kSpeedBucketCount * index_[idx]);

    // TODO - how to figure out the bucket?
 //   int bucket = (seconds_of_week / kSpeedBucketSizeSeconds);
    float i = (static_cast<float>(seconds_of_week) / kSecondsPerWeek) * static_cast<float>(kSpeedBucketCount);

    // DTC-III
    float speed = 0.5f * speeds[0];
    for (uint32_t n = 1; n < kSpeedBucketCount; ++n) {
      speed += speeds[n] * cosf(kPiConstant * n * (i + 0.5f));
    }
    return speed > 0.0f ? static_cast<uint32_t>(speed + 0.5f) : 0;
  }

protected:
  const uint32_t* index_; // Index into the array of compressed speed profiles for each directed edge
  const int16_t* profiles_; // Compressed speed profiles
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_PREDICTEDSPEEDS_H_
