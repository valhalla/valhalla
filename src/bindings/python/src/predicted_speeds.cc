#include "baldr/predictedspeeds.h"
#include "predicted_speeds_module.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
namespace vb = valhalla::baldr;

namespace pyvalhalla {

void init_predicted_speeds(nb::module_& m) {
  // Module constants
  m.attr("BUCKETS_PER_WEEK") = vb::kBucketsPerWeek;
  m.attr("COEFFICIENT_COUNT") = vb::kCoefficientCount;
  m.attr("SPEED_BUCKET_SIZE_MINUTES") = vb::kSpeedBucketSizeMinutes;
  m.attr("SPEED_BUCKET_SIZE_SECONDS") = vb::kSpeedBucketSizeSeconds;

  // compress_speed_buckets: numpy array[2016 float] -> array[200 int16]
  m.def(
      "compress_speed_buckets",
      [](nb::ndarray<float, nb::shape<vb::kBucketsPerWeek>> speeds) {
        auto result = vb::compress_speed_buckets(speeds.data());
        // Convert std::array to numpy array
        int16_t* data = new int16_t[vb::kCoefficientCount];
        std::copy(result.begin(), result.end(), data);
        size_t shape[1] = {vb::kCoefficientCount};
        return nb::ndarray<nb::numpy, int16_t, nb::shape<vb::kCoefficientCount>>(
            data, 1, shape, nb::handle(), nullptr, nb::dtype<int16_t>());
      },
      nb::arg("speeds"),
      "Compress 2016 speed buckets into 200 DCT-II coefficients.\n\n"
      "Args:\n"
      "    speeds: NumPy array of 2016 float values (one per 5-minute bucket)\n\n"
      "Returns:\n"
      "    NumPy array of 200 int16 coefficients");

  // decompress_speed_bucket: array[200 int16], int -> float
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
      "Args:\n"
      "    coefficients: Array of 200 int16 coefficients\n"
      "    bucket_idx: Bucket index (0 to 2015)\n\n"
      "Returns:\n"
      "    Speed in KPH for the specified bucket");

  // encode_compressed_speeds: array[200 int16] -> str (base64)
  m.def(
      "encode_compressed_speeds",
      [](const std::array<int16_t, vb::kCoefficientCount>& coefficients) {
        return vb::encode_compressed_speeds(coefficients.data());
      },
      nb::arg("coefficients"),
      "Encode 200 coefficients as base64 string.\n\n"
      "Args:\n"
      "    coefficients: Array of 200 int16 coefficients\n\n"
      "Returns:\n"
      "    Base64-encoded string");

  // decode_compressed_speeds: str (base64) -> array[200 int16]
  m.def(
      "decode_compressed_speeds",
      [](const std::string& encoded) {
        auto result = vb::decode_compressed_speeds(encoded);
        // Convert std::array to numpy array
        int16_t* data = new int16_t[vb::kCoefficientCount];
        std::copy(result.begin(), result.end(), data);
        size_t shape[1] = {vb::kCoefficientCount};
        return nb::ndarray<nb::numpy, int16_t, nb::shape<vb::kCoefficientCount>>(
            data, 1, shape, nb::handle(), nullptr, nb::dtype<int16_t>());
      },
      nb::arg("encoded"),
      "Decode base64 string to 200 coefficients.\n\n"
      "Args:\n"
      "    encoded: Base64-encoded string (400 characters)\n\n"
      "Returns:\n"
      "    Array of 200 int16 coefficients\n\n"
      "Raises:\n"
      "    RuntimeError: If decoded size is incorrect");
}

} // namespace pyvalhalla
