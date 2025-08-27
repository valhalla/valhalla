#include <napi.h>
#include <cmath>
#include <vector>
#include <cstdint>
#include <memory>

constexpr int kCoefficientCount = 200;
constexpr int kBucketsPerWeek = 2016;
constexpr float k1OverSqrt2 = 0.707106781f;
constexpr float kPiBucketConstant = 3.14159265f / static_cast<float>(kBucketsPerWeek);
constexpr float kSpeedNormalization = 0.031497039f;

// Singleton for precomputed cosine table
class BucketCosTable {
 public:
  static BucketCosTable& Instance() {
    static BucketCosTable instance;
    return instance;
  }

  const float* Get(int bucket) const {
    return &table_[bucket * kCoefficientCount];
  }

 private:
  BucketCosTable() {
    table_.resize(kCoefficientCount * kBucketsPerWeek);
    for (int bucket = 0; bucket < kBucketsPerWeek; ++bucket) {
      for (int c = 0; c < kCoefficientCount; ++c) {
        table_[bucket * kCoefficientCount + c] = std::cos(kPiBucketConstant * (bucket + 0.5f) * c);
      }
    }
  }

  std::vector<float> table_;
};

Napi::Value CompressSpeedBuckets(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  if (!info[0].IsTypedArray()) {
    Napi::TypeError::New(env, "Expected a Uint8Array").ThrowAsJavaScriptException();
    return env.Null();
  }

  Napi::Uint8Array input = info[0].As<Napi::Uint8Array>();
  if (input.ByteLength() != kBucketsPerWeek) {
    Napi::TypeError::New(env, "Input size must be 2016 bytes").ThrowAsJavaScriptException();
    return env.Null();
  }

  std::vector<float> coefficients(kCoefficientCount, 0.0f);

  // DCT-II computation
  for (int bucket = 0; bucket < kBucketsPerWeek; ++bucket) {
    const float speed = static_cast<float>(input[bucket]);
    const float* cosValues = BucketCosTable::Instance().Get(bucket);
    for (int c = 0; c < kCoefficientCount; ++c) {
      coefficients[c] += cosValues[c] * speed;
    }
  }

  coefficients[0] *= k1OverSqrt2;

  // Normalize and convert to int16
  Napi::Int16Array result = Napi::Int16Array::New(env, kCoefficientCount);
  for (int i = 0; i < kCoefficientCount; ++i) {
    result[i] = static_cast<int16_t>(std::round(coefficients[i] * kSpeedNormalization));
  }

  return result;
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
  exports.Set("compressSpeedBuckets", Napi::Function::New(env, CompressSpeedBuckets));
  return exports;
}

NODE_API_MODULE(compression, Init)
