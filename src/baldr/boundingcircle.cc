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
//
//  gut approach number 2: smaller steps up to 100
// {1.,   2.,   3.,   4.,   5.,   6.,   7.,   8.,   9.,    10.,   11.,   12.,  13.,
//  15.,  17.,  19.,  21.,  24.,  27.,  30.,  33.,  37.,   41.,   45.,   50.,  55.,
//  60.,  65.,  70.,  76.,  82.,  88.,  94.,  100., 110.,  120.,  130.,  140., 150.,
//  160., 170., 180., 190., 200., 210., 220., 230., 240.,  250.,  275.,  300., 325.,
//  350., 375., 400., 500., 550., 600., 700., 800., 1000., 1500., 2000., 2500.};

// (name=new1)
// 1% sample globally, 99.99% cutoff, log-transformed
// {0.20,   0.38,   0.58,   0.80,    1.05,    1.31,    1.60,    1.91,    2.25,    2.60,   2.99,
//  3.41,   3.87,   4.37,   4.93,    5.55,    6.26,    7.04,    7.89,    8.82,    9.85,   10.99,
//  12.25,  13.63,  15.12,  16.73,   18.47,   20.35,   22.40,   24.63,   27.05,   29.71,  32.64,
//  35.86,  39.43,  43.39,  47.78,   52.62,   57.98,   64.03,   70.92,   78.76,   87.75,  98.03,
//  109.90, 123.77, 140.06, 159.15,  181.63,  208.15,  239.80,  278.23,  325.41,  384.40, 460.94,
//  562.55, 699.54, 884.06, 1140.67, 1526.88, 2167.86, 3359.29, 5977.84, 14819.10};

// (name=new2)
// 1% sample globally, 99.92% cutoff, log-transformed
// {0.20,   0.38,   0.58,   0.80,   1.05,    1.31,    1.60,    1.91,    2.25,   2.60,   2.99,
//  3.41,   3.87,   4.36,   4.92,   5.53,    6.22,    6.98,    7.82,    8.73,   9.73,   10.83,
//  12.05,  13.38,  14.81,  16.35,  18.01,   19.80,   21.73,   23.80,   26.02,  28.43,  31.06,
//  33.94,  37.11,  40.62,  44.52,  48.85,   53.68,   59.09,   65.26,   72.29,  80.28,  89.44,
//  99.87,  111.87, 125.88, 142.24, 161.40,  183.89,  210.35,  241.93,  280.16, 326.94, 385.26,
//  460.77, 560.76, 694.67, 869.81, 1102.97, 1440.64, 1968.28, 2870.40, 4611.49};

// (name=new3)
// 1% sample globally, 99.92% cutoff, non-transformed
// {6.44,    12.44,   18.92,   25.68,   32.88,   40.73,   49.23,   58.42,   68.49,   79.57,
//  91.70,   104.82,  119.04,  134.50,  151.08,  168.89,  187.97,  208.11,  229.39,  251.89,
//  275.83,  301.43,  328.68,  357.83,  388.66,  421.30,  456.31,  493.98,  534.67,  578.25,
//  624.43,  673.10,  724.53,  779.18,  836.79,  897.96,  962.97,  1030.49, 1100.67, 1173.48,
//  1249.83, 1330.07, 1412.42, 1496.58, 1584.08, 1677.78, 1778.56, 1883.77, 1993.48, 2107.18,
//  2226.03, 2352.19, 2483.05, 2623.22, 2774.23, 2933.60, 3100.61, 3276.44, 3464.24, 3671.71,
//  3891.71, 4117.44, 4356.79, 4611.49};

// geometric sequence
// mh... not as good as the gut approach
// {0.50,   0.57,   0.66,    0.75,    0.86,    0.98,    1.13,    1.29,    1.47,   1.69,   1.93,
//  2.21,   2.53,   2.90,    3.32,    3.80,    4.35,    4.98,    5.70,    6.52,   7.47,   8.55,
//  9.79,   11.20,  12.83,   14.68,   16.81,   19.24,   22.03,   25.22,   28.87,  33.04,  37.83,
//  43.30,  49.57,  56.75,   64.96,   74.37,   85.13,   97.46,   111.56,  127.71, 146.20, 167.36,
//  191.59, 219.33, 251.08,  287.42,  329.03,  376.66,  431.18,  493.60,  565.05, 646.85, 740.48,
//  847.68, 970.38, 1110.85, 1271.66, 1455.74, 1666.47, 1907.71, 2183.87, 2500.00};
//
//  quantile based
//  also not that good
// {0.78,   1.76,   2.34,   2.84,   3.30,   3.78,   4.27,   4.76,   5.31,   5.95,   6.64,
//  7.36,   8.09,   8.84,   9.57,   10.35,  11.21,  12.08,  12.99,  13.92,  14.88,  15.89,
//  16.92,  17.95,  19.01,  20.11,  21.23,  22.40,  23.62,  24.86,  26.13,  27.44,  28.81,
//  30.29,  31.87,  33.54,  35.29,  37.10,  38.96,  41.00,  43.25,  45.69,  48.23,  50.94,
//  53.84,  56.96,  60.51,  64.56,  68.95,  74.14,  80.14,  86.91,  94.92,  104.18, 115.25,
//  128.42, 144.32, 165.37, 192.53, 229.63, 285.25, 375.67, 562.29, 2482.04};

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
