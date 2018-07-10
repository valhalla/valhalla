#ifndef VALHALLA_BALDR_PREDICTEDSPEEDS_H_
#define VALHALLA_BALDR_PREDICTEDSPEEDS_H_

#include <valhalla/midgard/util.h>
#include <vector>

namespace valhalla {
namespace baldr {

constexpr uint32_t kSpeedBucketSize = 5 * 60; // 5 minute buckets (in seconds)
constexpr uint32_t kBucketCount = 200;
constexpr float kPiConstant = kPi / 2016.0f;

/**
 * Class to access predicted speed information within a tile.
 */
class PredictedSpeeds {
public:
  /**
   * Constructor given arguments.
   * @param  index_ Pointer to the index array in the GraphTile.
   * @param  profiles Pointer to the profiles array in the GraphTile.
   */
  PredictedSpeeds(uint32_t* index, uint16_t* profiles) : index_(index), profiles_(profiles) {
  }

  /**
   * Get the speed given the edge Id and the seconds of the week.
   * @param  idx  Directed edge index.
   * @param  seconds_of_week  Seconds from start of the week (local time).
   */
  uint32_t speed(const uint32_t idx, const uint32_t seconds_of_week) {
    // Assume the index is valid. If there is no predicted speed profile this
    // method will not be called due to DirectedEdge::predicted_speed being false.
    uint16_t* speeds = profiles_ + (kBucketCount * index_[idx]);
    float b = ((seconds_of_week / kSpeedBucketSize) + 0.5f) * kPiConstant;
    float speed = 0.5f * speeds[0];
    uint16_t* s = &speeds[1];
    for (uint32_t k = 1; k < kBucketCount; ++k, ++s) {
      speed += *s * cosf(k * b);
      ++k;
    }
    return static_cast<uint32_t>(speed + 0.5f);
  }

protected:
  uint32_t* index_;    // Index into the array of compressed speed profiles for each directed edge.
  uint16_t* profiles_; // Compressed speed profiles
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_PREDICTEDSPEEDS_H_
