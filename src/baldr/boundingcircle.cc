#include <valhalla/baldr/boundingcircle.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/pointll.h>

#include <array>
#include <cstdint>
#include <ostream>
#include <string>

namespace {

using namespace valhalla;
using namespace valhalla::baldr;

// the possible radii for our discretized bounding circles. we only store indices into this
// array on disk, so we predefine the set of all possible radii here loosely
// based on the distribution of all actual bounding circles in a planet (most edges are
// small, so we want a finer resolution in the 1-20 meter range)
constexpr std::array<double, kRadiiCount> kBoundingCircleRadii =
    // This one was arbitrarily chosen an eon ago
    {1.,   2.,   3.,   4.,   5.,   6.,   7.,   8.,   9.,    10.,   13.,   15.,  17.,
     18.,  20.,  23.,  25.,  27.,  30.,  35.,  40.,  43.,   45.,   50.,   55.,  60.,
     65.,  70.,  75.,  80.,  85.,  90.,  95.,  100., 110.,  120.,  130.,  140., 150.,
     160., 170., 180., 190., 200., 210., 220., 230., 240.,  250.,  275.,  300., 325.,
     350., 375., 400., 500., 550., 600., 700., 800., 1000., 1500., 2000., 2500.};

//   (name=bc1)
// {0.7,   1.4,   2.1,   2.8,   3.5,   4.2,   4.9,   5.6,    6.3,    7.0,    7.7,    8.4,   9.1,
//  9.8,   10.5,  11.2,  11.9,  12.6,  13.3,  14.0,  14.7,   15.4,   16.1,   16.8,   17.5,  32.1,
//  38.1,  42.7,  46.6,  50.0,  53.1,  56.0,  58.6,  61.1,   63.5,   65.7,   67.9,   69.9,  71.9,
//  73.8,  75.7,  77.5,  79.2,  80.9,  82.5,  84.1,  85.7,   104.5,  127.4,  155.4,  189.4, 231.0,
//  281.6, 343.4, 418.7, 510.5, 622.4, 758.9, 925.4, 1128.3, 1375.7, 1677.4, 2045.2, 2493.7};

// (name=bc2)
// I think this was created using kevins kmeans approach with a 99.9% cutoff on NL data
// {2.89,   4.95,   7.05,   9.32,   11.76,  14.33,  17,     19.77,  22.65,  25.66,  28.84,
//  32.19,  35.71,  39.41,  43.28,  47.33,  51.56,  56.03,  60.78,  65.82,  71.18,  76.84,
//  82.77,  89.01,  95.59,  102.47, 109.64, 117.13, 124.99, 133.25, 141.88, 150.88, 160.26,
//  170.06, 180.31, 191.11, 202.47, 214.37, 226.94, 240.14, 253.82, 268.09, 282.96, 298.39,
//  314.2,  330.4,  347.13, 364.48, 382.89, 402.37, 423.08, 445.28, 468.81, 493.41, 519,
//  546.3,  575.63, 606.42, 638.59, 674.07, 713.2,  755.05, 798.75, 844.95};

// (name=bc3)
// log-transformed kmeans on NL data, 99.99% cutoff
// result was not that good
// {0.2,    0.41,   0.6,    0.79,   0.98,   1.17,   1.36,   1.58,    1.82,   2.09,   2.39,
//  2.73,   3.1,    3.48,   3.87,   4.26,   4.66,   5.1,    5.58,    6.12,   6.72,   7.38,
//  8.09,   8.85,   9.66,   10.55,  11.54,  12.61,  13.8,   15.11,   16.52,  18.04,  19.68,
//  21.42,  23.28,  25.25,  27.36,  29.62,  32.04,  34.68,  37.59,   40.85,  44.51,  48.69,
//  53.54,  59.2,   65.81,  73.5,   82.35,  92.42,  103.94, 117.35,  133.47, 153.34, 178.14,
//  209.41, 249.09, 300.14, 367.06, 456.68, 580.32, 763.44, 1077.27, 1788.08};

// (name=bc4)
// globally optimized k-means with a 99.95% cutoff
// also no good
// {5.8374,    11.0372,   16.5725,   22.3434,   28.4162,   34.9298,   41.9952,   49.6057,
//  57.7612,   66.6569,   76.3961,   86.947,    98.2737,   110.4266,  123.5267,  137.6031,
//  152.5978,  168.57,    185.6141,  203.4761,  222.371,   242.6063,  264.0602,  286.7592,
//  310.7088,  335.974,   362.6443,  390.6649,  420.4156,  452.1364,  485.6746,  520.9043,
//  558.0109,  597.2139,  639.031,   683.7511,  730.9604,  780.3316,  829.5901,  879.6104,
//  932.9026,  988.8838,  1048.3152, 1111.7328, 1178.6184, 1249.108,  1323.6787, 1402.3282,
//  1485.1327, 1572.7373, 1665.1049, 1762.7826, 1866.3386, 1975.4114, 2090.9775, 2213.4741,
//  2342.7554, 2478.3599, 2622.5605, 2777.6304, 2943.0947, 3119.8274, 3308.842,  3510.9941};

constexpr uint32_t kInvalidRadiusIndex = kBoundingCircleRadii.size();

// the maximum offset in the x and y axis we support is derived from
//   - half the size of the bin (since we compute offsets relative to the center) plus
//   - the radius of the largest radius we support
constexpr double kMaxOffsetMeters =
    0.05 * midgard::kMetersPerDegreeLat / 2 + kBoundingCircleRadii.back();

// the offset resolution is the max offset divided by the number of unique values
// given the number of bits we use for each offset
constexpr double kOffsetIncrement = kMaxOffsetMeters / (1 << (kCoordinateBits - 1));
} // namespace

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
      ((static_cast<double>(y_offset_increments) / static_cast<double>(kMaxOffsetValue)) *
       kMaxOffsetMeters * 2) -
      kMaxOffsetMeters;
  double discretized_x_offset =
      ((static_cast<double>(x_offset_increments) / static_cast<double>(kMaxOffsetValue)) *
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

std::pair<midgard::PointLL, double>
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
  //        0        |     4095    |       8191
  //        |–––––––-|------o––––––|-------->
  //                 |             |
  //                 |             |
  //                 +-------------+
  //
  // So when we turn our stored offset back to meter offsets from the center, we have to shift the
  // range back again, so that the stored value 0 becomes -max_offset meters, 4095 becomes 0 meters,
  // and 8191 becomes +max_offset meters
  auto y_offset_meters = ((static_cast<double>(y_offset_) / static_cast<double>(kMaxOffsetValue)) *
                          kMaxOffsetMeters * 2) -
                         kMaxOffsetMeters;

  auto x_offset_meters = ((static_cast<double>(x_offset_) / static_cast<double>(kMaxOffsetValue)) *
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
