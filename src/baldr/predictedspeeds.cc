#include "baldr/predictedspeeds.h"

namespace valhalla {
namespace baldr {

// DCT-III constants for speed decoding and normalization
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

std::array<int16_t, kCoefficientCount> compress_speed_buckets(const float* speeds) {
  std::array<float, kCoefficientCount> coefficients;
  coefficients.fill(0.f);

  // DCT-II with speed normalization
  for (uint32_t bucket = 0; bucket < kBucketsPerWeek; ++bucket) {
    // Get a pointer to the precomputed cos values for this bucket
    const float* cos_values = BucketCosTable::GetInstance().get(bucket);
    for (uint32_t c = 0; c < kCoefficientCount; ++c) {
      coefficients[c] += cos_values[c] * speeds[bucket];
    }
  }
  coefficients[0] *= k1OverSqrt2;

  std::array<int16_t, kCoefficientCount> result;
  for (size_t i = 0; i < coefficients.size(); ++i) {
    result[i] = static_cast<int16_t>(roundf(kSpeedNormalization * coefficients[i]));
  }
  return result;
}

float decompress_speed_bucket(const int16_t* coefficients, uint32_t bucket_idx) {
  // Get a pointer to the precomputed cos values for this bucket
  const float* b = BucketCosTable::GetInstance().get(bucket_idx);

  // DCT-III with speed normalization
  float speed = *coefficients * k1OverSqrt2;
  const auto* coef_end = coefficients + kCoefficientCount;
  for (++b, ++coefficients; coefficients < coef_end; ++coefficients, ++b) {
    speed += *coefficients * *b;
  }
  return speed * kSpeedNormalization;
}

std::string encode_compressed_speeds(const int16_t* coefficients) {
  std::string result;
  result.reserve(kCoefficientCount * sizeof(uint16_t) / sizeof(char));
  for (uint32_t i = 0; i < kCoefficientCount; ++i) {
    // change byte order: little endian -> big endian
    const uint16_t bytes = midgard::to_big_endian(static_cast<uint16_t>(coefficients[i]));
    result.append(reinterpret_cast<const char*>(&bytes), sizeof(uint16_t) / sizeof(char));
  }
  return midgard::encode64(result);
}

std::array<int16_t, kCoefficientCount> decode_compressed_speeds(const std::string& encoded) {
  const std::string& decoded_str = midgard::decode64(encoded);
  if (decoded_str.size() != kDecodedSpeedSize) {
    throw std::runtime_error(
        "Decoded speed string size expected= " + std::to_string(kDecodedSpeedSize) +
        " actual=" + std::to_string(decoded_str.size()));
  }
  const int8_t* raw = reinterpret_cast<const int8_t*>(decoded_str.data());
  // Create the coefficients. Each group of 2 bytes represents a signed, int16 number
  // (big endian). Convert to little endian.
  std::array<int16_t, kCoefficientCount> coefficients;
  for (uint32_t i = 0, idx = 0; i < kCoefficientCount; ++i, idx += 2) {
    coefficients[i] = midgard::to_little_endian(*(reinterpret_cast<const uint16_t*>(&raw[idx])));
  }
  return coefficients;
}

} // namespace baldr
} // namespace valhalla
