#include <valhalla/baldr/boundingcircle.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/pointll.h>

#include <cstdint>
#include <string>

namespace valhalla {
namespace baldr {

DiscretizedBoundingCircle::DiscretizedBoundingCircle(
    const midgard::DistanceApproximator<midgard::PointLL>& bin_center_approx,
    const midgard::PointLL& bin_center,
    const midgard::PointLL& circle_center,
    double radius) {

  midgard::PointLL offset{circle_center.lng() - bin_center.lng(),
                          circle_center.lat() - bin_center.lat()};

  double x_meters = offset.lng() * bin_center_approx.GetLngScale() * midgard::kMetersPerDegreeLat;
  double y_meters = offset.lat() * midgard::kMetersPerDegreeLat;

  // if any of the offsets is larger than the largest value we support bail
  if (std::abs(x_meters) >= kMaxOffsetMeters - 0.5 || std::abs(y_meters) >= kMaxOffsetMeters - 0.5) {
    radius_index_ = 0;
    y_offset_ = kMaxOffsetValue;
    x_offset_ = kMaxOffsetValue;
    return;
  }

  // this gives us the offset values we store
  uint32_t x_offset_increments =
      static_cast<uint32_t>((x_meters + kMaxOffsetMeters) / kOffsetIncrement + 0.5);

  uint32_t y_offset_increments =
      static_cast<uint32_t>((y_meters + kMaxOffsetMeters) / kOffsetIncrement + 0.5);

  // but we're not done yet, we need to account for the loss of precision
  // and resize the radius accordingly so that the bounding circle is
  // still guaranteed to cover the edge shape
  double discretized_y_offset =
      ((static_cast<double>(y_offset_increments) / static_cast<double>(1 << kCoordinateBits)) *
       kMaxOffsetMeters * 2) -
      kMaxOffsetMeters;
  double discretized_x_offset =
      ((static_cast<double>(x_offset_increments) / static_cast<double>(1 << kCoordinateBits)) *
       kMaxOffsetMeters * 2) -
      kMaxOffsetMeters;

  // turn it back to absolute ll
  midgard::PointLL discretized_center{(discretized_x_offset / (bin_center_approx.GetLngScale() *
                                                               midgard::kMetersPerDegreeLat)) +
                                          bin_center.lng(),
                                      discretized_y_offset / midgard::kMetersPerDegreeLat +
                                          bin_center.lat()};

  // ...to calculate how far we moved the center to align with our grid
  auto loss_of_precision = circle_center.Distance(discretized_center);
  radius += loss_of_precision;

  size_t index = kInvalidRadiusIndex;
  for (size_t i = 0; i < kBoundingCircleRadii.size(); ++i) {
    if (radius <= kBoundingCircleRadii[i]) {
      index = i;
      break;
    }
  }
  // set the offset and radius if radius doesn't exceed the max radius we support
  if (index != kInvalidRadiusIndex) {
    radius_index_ = index;
    y_offset_ = y_offset_increments;
    x_offset_ = x_offset_increments;
  } else {
    // In the rare case the max supported radius is exceeded,
    // set the offset and radius to an impossible combination to indicate that we
    // couldn't store any actual information about the bounding circle.
    // This is necessary because we have to store something so that indices into the
    // edge bins also work as indices into the bounding circles.
    radius_index_ = 0;
    y_offset_ = kMaxOffsetValue;
    x_offset_ = kMaxOffsetValue;
  }
}

std::pair<midgard::PointLL, uint16_t>
DiscretizedBoundingCircle::get(const midgard::DistanceApproximator<midgard::PointLL>& approx,
                               const midgard::PointLL& bin_center) const {

  // we offset relative to the center, so we need a way to tell whether the offset is in the
  // positive or negative direction. Instead of storing a sign bit, we simply shift the range
  // so that the value 0 represents the max offset in the negative direction.
  //
  //
  //                 +-------------+
  //                 |             |
  //   -5262m        |             |      +5262m
  //        <–––––––-|------o––––––|-------->
  //                 |             |
  //                 |             |
  //                 +-------------+
  //
  // (note: ~5262 meters is the max offset we support)
  //
  // In our quantized coordinate space, this translates to
  //
  //                 +-------------+
  //                 |             |
  //        0        |    4096     |       8192
  //        |–––––––-|------o––––––|-------->
  //                 |             |
  //                 |             |
  //                 +-------------+
  //
  // So when we turn our stored offset back to meter offsets from the center, we have to shift the
  // range back again, so that the stored value 0 becomes -max_offset meters, 4096 becomes 0 meters,
  // and 8192 becomes +max_offset meters
  auto y_offset_meters =
      ((static_cast<double>(y_offset_) / static_cast<double>(1 << kCoordinateBits)) *
       kMaxOffsetMeters * 2) -
      kMaxOffsetMeters;

  auto x_offset_meters =
      ((static_cast<double>(x_offset_) / static_cast<double>(1 << kCoordinateBits)) *
       kMaxOffsetMeters * 2) -
      kMaxOffsetMeters;
  midgard::PointLL center{(x_offset_meters / (approx.GetMetersPerLngDegree())) + bin_center.lng(),
                          y_offset_meters / midgard::kMetersPerDegreeLat + bin_center.lat()};

  return std::make_pair(center, kBoundingCircleRadii[radius_index_]);
}

std::ostream& operator<<(std::ostream& os, const DiscretizedBoundingCircle& circle) {
  return os << std::to_string(circle);
}
} // namespace baldr
} // namespace valhalla
