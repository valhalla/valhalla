#include "baldr/predictedspeeds.h"
#include "module.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
namespace vb = valhalla::baldr;

namespace pyvalhalla::baldr::utils {

void init_predicted_speeds(nb::module_& m) {
  m.attr("BUCKETS_PER_WEEK") = vb::kBucketsPerWeek;
  m.attr("COEFFICIENT_COUNT") = vb::kCoefficientCount;
  m.attr("SPEED_BUCKET_SIZE_MINUTES") = vb::kSpeedBucketSizeMinutes;
  m.attr("SPEED_BUCKET_SIZE_SECONDS") = vb::kSpeedBucketSizeSeconds;

  m.def(
      "compress_speed_buckets",
      [](const nb::ndarray<float, nb::shape<vb::kBucketsPerWeek>>& speeds) {
        auto result = vb::compress_speed_buckets(speeds.data());
        int16_t* data = new int16_t[vb::kCoefficientCount];
        std::copy(result.begin(), result.end(), data);
        size_t shape[1] = {vb::kCoefficientCount};
        nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<int16_t*>(p); });
        return nb::ndarray<nb::numpy, int16_t, nb::shape<vb::kCoefficientCount>>(data, 1, shape,
                                                                                 owner);
      },
      nb::arg("speeds"),
      "Compress 2016 speed buckets into 200 DCT-II coefficients.\n\n"
      ":param speeds: NumPy array of 2016 float values (one per 5-minute bucket)\n"
      ":returns: NumPy array of 200 int16 coefficients");

  m.def(
      "decompress_speed_bucket",
      [](const std::array<int16_t, vb::kCoefficientCount>& coefficients, uint32_t bucket_idx) {
        if (bucket_idx >= vb::kBucketsPerWeek) {
          throw nb::value_error("bucket_idx must be < BUCKETS_PER_WEEK (2016)");
        }
        return vb::decompress_speed_bucket(coefficients.data(), bucket_idx);
      },
      nb::arg("coefficients"), nb::arg("bucket_idx"),
      "Decompress a single speed bucket using DCT-III.\n\n"
      ":param coefficients: Array of 200 int16 coefficients\n"
      ":param bucket_idx: Bucket index (0 to 2015)\n"
      ":returns: Speed in KPH for the specified bucket\n"
      ":raises ValueError: If bucket_idx is out of range");

  m.def(
      "encode_compressed_speeds",
      [](const std::array<int16_t, vb::kCoefficientCount>& coefficients) {
        return vb::encode_compressed_speeds(coefficients.data());
      },
      nb::arg("coefficients"),
      "Encode 200 coefficients as base64 string.\n\n"
      ":param coefficients: Array of 200 int16 coefficients\n"
      ":returns: Base64-encoded string");

  m.def(
      "decode_compressed_speeds",
      [](const std::string& encoded) {
        auto result = vb::decode_compressed_speeds(encoded);
        int16_t* data = new int16_t[vb::kCoefficientCount];
        std::copy(result.begin(), result.end(), data);
        size_t shape[1] = {vb::kCoefficientCount};
        nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<int16_t*>(p); });
        return nb::ndarray<nb::numpy, int16_t, nb::shape<vb::kCoefficientCount>>(data, 1, shape,
                                                                                 owner);
      },
      nb::arg("encoded"),
      "Decode base64 string to 200 coefficients.\n\n"
      ":param encoded: Base64-encoded string (536 characters)\n"
      ":returns: Array of 200 int16 coefficients\n"
      ":raises RuntimeError: If decoded size is incorrect");
}

} // namespace pyvalhalla::baldr::utils
