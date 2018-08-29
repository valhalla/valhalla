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
constexpr float k1OverSqrt2 = 0.707106781f; // 1 / sqrt(2)
constexpr float kPiBucketConstant = 3.14159265f / 2016.0f;
constexpr float kSpeedNormalization = 0.031497039f; // sqrt(2.0f / 2016.0f);

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
    // to DirectedEdge::predicted_speed being false.
    const int16_t* coefficients = profiles_ + offset_[idx];

    // Precompute the constant based on the "bucket" within the week
    float b =
        kPiBucketConstant * (static_cast<int>(seconds_of_week / kSpeedBucketSizeSeconds) + 0.5f);

    // DTC-III with speed normalization
    /*float speed = *coefficients * k1OverSqrt2;
        ++coefficients;
        for (int k = 1; k < kCoefficientCount; ++k, ++coefficients) {
          speed += *coefficients * cosf(b * k);
        }
        return speed * kSpeedNormalization;*/

    // Unroll the for loop to optimize
    return kSpeedNormalization *
           ((*coefficients++ * k1OverSqrt2) + (*coefficients++ * cosf(b)) +
            (*coefficients++ * cosf(b * 2)) + (*coefficients++ * cosf(b * 3)) +
            (*coefficients++ * cosf(b * 4)) + (*coefficients++ * cosf(b * 5)) +
            (*coefficients++ * cosf(b * 6)) + (*coefficients++ * cosf(b * 7)) +
            (*coefficients++ * cosf(b * 8)) + (*coefficients++ * cosf(b * 9)) +
            (*coefficients++ * cosf(b * 10)) + (*coefficients++ * cosf(b * 11)) +
            (*coefficients++ * cosf(b * 12)) + (*coefficients++ * cosf(b * 13)) +
            (*coefficients++ * cosf(b * 14)) + (*coefficients++ * cosf(b * 15)) +
            (*coefficients++ * cosf(b * 16)) + (*coefficients++ * cosf(b * 17)) +
            (*coefficients++ * cosf(b * 18)) + (*coefficients++ * cosf(b * 19)) +
            (*coefficients++ * cosf(b * 20)) + (*coefficients++ * cosf(b * 21)) +
            (*coefficients++ * cosf(b * 22)) + (*coefficients++ * cosf(b * 23)) +
            (*coefficients++ * cosf(b * 24)) + (*coefficients++ * cosf(b * 25)) +
            (*coefficients++ * cosf(b * 26)) + (*coefficients++ * cosf(b * 27)) +
            (*coefficients++ * cosf(b * 28)) + (*coefficients++ * cosf(b * 29)) +
            (*coefficients++ * cosf(b * 30)) + (*coefficients++ * cosf(b * 31)) +
            (*coefficients++ * cosf(b * 32)) + (*coefficients++ * cosf(b * 33)) +
            (*coefficients++ * cosf(b * 34)) + (*coefficients++ * cosf(b * 35)) +
            (*coefficients++ * cosf(b * 36)) + (*coefficients++ * cosf(b * 37)) +
            (*coefficients++ * cosf(b * 38)) + (*coefficients++ * cosf(b * 39)) +
            (*coefficients++ * cosf(b * 40)) + (*coefficients++ * cosf(b * 41)) +
            (*coefficients++ * cosf(b * 42)) + (*coefficients++ * cosf(b * 43)) +
            (*coefficients++ * cosf(b * 44)) + (*coefficients++ * cosf(b * 45)) +
            (*coefficients++ * cosf(b * 46)) + (*coefficients++ * cosf(b * 47)) +
            (*coefficients++ * cosf(b * 48)) + (*coefficients++ * cosf(b * 49)) +
            (*coefficients++ * cosf(b * 50)) + (*coefficients++ * cosf(b * 51)) +
            (*coefficients++ * cosf(b * 52)) + (*coefficients++ * cosf(b * 53)) +
            (*coefficients++ * cosf(b * 54)) + (*coefficients++ * cosf(b * 55)) +
            (*coefficients++ * cosf(b * 56)) + (*coefficients++ * cosf(b * 57)) +
            (*coefficients++ * cosf(b * 58)) + (*coefficients++ * cosf(b * 59)) +
            (*coefficients++ * cosf(b * 60)) + (*coefficients++ * cosf(b * 61)) +
            (*coefficients++ * cosf(b * 62)) + (*coefficients++ * cosf(b * 63)) +
            (*coefficients++ * cosf(b * 64)) + (*coefficients++ * cosf(b * 65)) +
            (*coefficients++ * cosf(b * 66)) + (*coefficients++ * cosf(b * 67)) +
            (*coefficients++ * cosf(b * 68)) + (*coefficients++ * cosf(b * 69)) +
            (*coefficients++ * cosf(b * 70)) + (*coefficients++ * cosf(b * 71)) +
            (*coefficients++ * cosf(b * 72)) + (*coefficients++ * cosf(b * 73)) +
            (*coefficients++ * cosf(b * 74)) + (*coefficients++ * cosf(b * 75)) +
            (*coefficients++ * cosf(b * 76)) + (*coefficients++ * cosf(b * 77)) +
            (*coefficients++ * cosf(b * 78)) + (*coefficients++ * cosf(b * 79)) +
            (*coefficients++ * cosf(b * 80)) + (*coefficients++ * cosf(b * 81)) +
            (*coefficients++ * cosf(b * 82)) + (*coefficients++ * cosf(b * 83)) +
            (*coefficients++ * cosf(b * 84)) + (*coefficients++ * cosf(b * 85)) +
            (*coefficients++ * cosf(b * 86)) + (*coefficients++ * cosf(b * 87)) +
            (*coefficients++ * cosf(b * 88)) + (*coefficients++ * cosf(b * 89)) +
            (*coefficients++ * cosf(b * 90)) + (*coefficients++ * cosf(b * 91)) +
            (*coefficients++ * cosf(b * 92)) + (*coefficients++ * cosf(b * 93)) +
            (*coefficients++ * cosf(b * 94)) + (*coefficients++ * cosf(b * 95)) +
            (*coefficients++ * cosf(b * 96)) + (*coefficients++ * cosf(b * 97)) +
            (*coefficients++ * cosf(b * 98)) + (*coefficients++ * cosf(b * 99)) +
            (*coefficients++ * cosf(b * 100)) + (*coefficients++ * cosf(b * 101)) +
            (*coefficients++ * cosf(b * 102)) + (*coefficients++ * cosf(b * 103)) +
            (*coefficients++ * cosf(b * 104)) + (*coefficients++ * cosf(b * 105)) +
            (*coefficients++ * cosf(b * 106)) + (*coefficients++ * cosf(b * 107)) +
            (*coefficients++ * cosf(b * 108)) + (*coefficients++ * cosf(b * 109)) +
            (*coefficients++ * cosf(b * 110)) + (*coefficients++ * cosf(b * 111)) +
            (*coefficients++ * cosf(b * 112)) + (*coefficients++ * cosf(b * 113)) +
            (*coefficients++ * cosf(b * 114)) + (*coefficients++ * cosf(b * 115)) +
            (*coefficients++ * cosf(b * 116)) + (*coefficients++ * cosf(b * 117)) +
            (*coefficients++ * cosf(b * 118)) + (*coefficients++ * cosf(b * 119)) +
            (*coefficients++ * cosf(b * 120)) + (*coefficients++ * cosf(b * 121)) +
            (*coefficients++ * cosf(b * 122)) + (*coefficients++ * cosf(b * 123)) +
            (*coefficients++ * cosf(b * 124)) + (*coefficients++ * cosf(b * 125)) +
            (*coefficients++ * cosf(b * 126)) + (*coefficients++ * cosf(b * 127)) +
            (*coefficients++ * cosf(b * 128)) + (*coefficients++ * cosf(b * 129)) +
            (*coefficients++ * cosf(b * 130)) + (*coefficients++ * cosf(b * 131)) +
            (*coefficients++ * cosf(b * 132)) + (*coefficients++ * cosf(b * 133)) +
            (*coefficients++ * cosf(b * 134)) + (*coefficients++ * cosf(b * 135)) +
            (*coefficients++ * cosf(b * 136)) + (*coefficients++ * cosf(b * 137)) +
            (*coefficients++ * cosf(b * 138)) + (*coefficients++ * cosf(b * 139)) +
            (*coefficients++ * cosf(b * 140)) + (*coefficients++ * cosf(b * 141)) +
            (*coefficients++ * cosf(b * 142)) + (*coefficients++ * cosf(b * 143)) +
            (*coefficients++ * cosf(b * 144)) + (*coefficients++ * cosf(b * 145)) +
            (*coefficients++ * cosf(b * 146)) + (*coefficients++ * cosf(b * 147)) +
            (*coefficients++ * cosf(b * 148)) + (*coefficients++ * cosf(b * 149)) +
            (*coefficients++ * cosf(b * 150)) + (*coefficients++ * cosf(b * 151)) +
            (*coefficients++ * cosf(b * 152)) + (*coefficients++ * cosf(b * 153)) +
            (*coefficients++ * cosf(b * 154)) + (*coefficients++ * cosf(b * 155)) +
            (*coefficients++ * cosf(b * 156)) + (*coefficients++ * cosf(b * 157)) +
            (*coefficients++ * cosf(b * 158)) + (*coefficients++ * cosf(b * 159)) +
            (*coefficients++ * cosf(b * 160)) + (*coefficients++ * cosf(b * 161)) +
            (*coefficients++ * cosf(b * 162)) + (*coefficients++ * cosf(b * 163)) +
            (*coefficients++ * cosf(b * 164)) + (*coefficients++ * cosf(b * 165)) +
            (*coefficients++ * cosf(b * 166)) + (*coefficients++ * cosf(b * 167)) +
            (*coefficients++ * cosf(b * 168)) + (*coefficients++ * cosf(b * 169)) +
            (*coefficients++ * cosf(b * 170)) + (*coefficients++ * cosf(b * 171)) +
            (*coefficients++ * cosf(b * 172)) + (*coefficients++ * cosf(b * 173)) +
            (*coefficients++ * cosf(b * 174)) + (*coefficients++ * cosf(b * 175)) +
            (*coefficients++ * cosf(b * 176)) + (*coefficients++ * cosf(b * 177)) +
            (*coefficients++ * cosf(b * 178)) + (*coefficients++ * cosf(b * 179)) +
            (*coefficients++ * cosf(b * 180)) + (*coefficients++ * cosf(b * 181)) +
            (*coefficients++ * cosf(b * 182)) + (*coefficients++ * cosf(b * 183)) +
            (*coefficients++ * cosf(b * 184)) + (*coefficients++ * cosf(b * 185)) +
            (*coefficients++ * cosf(b * 186)) + (*coefficients++ * cosf(b * 187)) +
            (*coefficients++ * cosf(b * 188)) + (*coefficients++ * cosf(b * 189)) +
            (*coefficients++ * cosf(b * 190)) + (*coefficients++ * cosf(b * 191)) +
            (*coefficients++ * cosf(b * 192)) + (*coefficients++ * cosf(b * 193)) +
            (*coefficients++ * cosf(b * 194)) + (*coefficients++ * cosf(b * 195)) +
            (*coefficients++ * cosf(b * 196)) + (*coefficients++ * cosf(b * 197)) +
            (*coefficients++ * cosf(b * 198)) + (*coefficients * cosf(b * 199)));
  }

protected:
  const uint32_t* offset_;  // Offset into the array of compressed speed profiles
                            // for each directed edge
  const int16_t* profiles_; // Compressed speed profiles
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_PREDICTEDSPEEDS_H_
