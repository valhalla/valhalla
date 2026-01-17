#include "baldr/predictedspeeds.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
namespace vb = valhalla::baldr;

namespace pyvalhalla {

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
      nb::arg("speeds"));

  m.def(
      "decompress_speed_bucket",
      [](const std::array<int16_t, vb::kCoefficientCount>& coefficients, uint32_t bucket_idx) {
        if (bucket_idx >= vb::kBucketsPerWeek) {
          throw nb::value_error("bucket_idx must be < BUCKETS_PER_WEEK (2016)");
        }
        return vb::decompress_speed_bucket(coefficients.data(), bucket_idx);
      },
      nb::arg("coefficients"), nb::arg("bucket_idx"));

  m.def(
      "encode_compressed_speeds",
      [](const std::array<int16_t, vb::kCoefficientCount>& coefficients) {
        return vb::encode_compressed_speeds(coefficients.data());
      },
      nb::arg("coefficients"));

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
      nb::arg("encoded"));
}

NB_MODULE(_predicted_speeds, m) {
  init_predicted_speeds(m);
  m.doc() = "Valhalla DCT-2 speed compression utilities";
}

} // namespace pyvalhalla
