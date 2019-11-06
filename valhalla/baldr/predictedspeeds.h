#ifndef VALHALLA_BALDR_PREDICTEDSPEEDS_H_
#define VALHALLA_BALDR_PREDICTEDSPEEDS_H_

#include <valhalla/midgard/util.h>
#include <vector>

namespace valhalla {
namespace baldr {

constexpr uint32_t kSpeedBucketSizeMinutes = 5;
constexpr uint32_t kSpeedBucketSizeSeconds = kSpeedBucketSizeMinutes * 60;
constexpr uint32_t kBucketsPerWeek = (7 * 24 * 60) / kSpeedBucketSizeMinutes;

// DCT-III constants for speed decoding and normalization
constexpr uint32_t kCoefficientCount = 200;
constexpr float k1OverSqrt2 = 0.707106781f; // 1 / sqrt(2)
constexpr float kPiBucketConstant = 3.14159265f / 2016.0f;
constexpr float kSpeedNormalization = 0.031497039f; // sqrt(2.0f / 2016.0f);

// Size of the cos table for the buckets
constexpr uint32_t kCosBucketTableSize = kCoefficientCount * kBucketsPerWeek;

// Precompute a cos table for each bucket of the week as a singleton.
class BucketCosTable final {
public:
  static BucketCosTable& GetInstance() {
    static BucketCosTable instance;
    return instance;
  }

  /**
   * Get a const pointer to the start of the stored cos values for the specified
   * bucket.
   * @param bucket  Bucket of the week.
   * @return Returns a pointer to the first cos value for the bucket.
   */
  const float* get(const uint32_t bucket) const {
    return &table_[bucket * kCoefficientCount];
  }

private:
  // Construct the cos table
  BucketCosTable() {
    // Fill out the table in bucket order.
    float* t = &table_[0];
    for (uint32_t bucket = 0; bucket < kBucketsPerWeek; ++bucket) {
      for (uint32_t c = 0; c < kCoefficientCount; ++c) {
        *t++ = cosf(kPiBucketConstant * (bucket + 0.5f) * c);
      }
    }
  }

  ~BucketCosTable() = default;
  BucketCosTable(const BucketCosTable&) = delete;
  BucketCosTable& operator=(const BucketCosTable&) = delete;
  BucketCosTable(BucketCosTable&&) = delete;
  BucketCosTable& operator=(BucketCosTable&&) = delete;

  // cos table (this uses about 1.6MB of memory)
  float table_[kCosBucketTableSize];
};

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

    // Get a pointer to the precomputed cos values for this bucket
    const float* b = BucketCosTable::GetInstance().get(seconds_of_week / kSpeedBucketSizeSeconds);

    // DCT-III with speed normalization
    float speed = *coefficients * k1OverSqrt2;
    ++coefficients;
    ++b;
    for (uint32_t k = 1; k < kCoefficientCount; ++k, ++coefficients, ++b) {
      speed += *coefficients * *b;
    }
    return speed * kSpeedNormalization;
  }

protected:
  const uint32_t* offset_;  // Offset into the array of compressed speed profiles
                            // for each directed edge
  const int16_t* profiles_; // Compressed speed profiles
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_PREDICTEDSPEEDS_H_
