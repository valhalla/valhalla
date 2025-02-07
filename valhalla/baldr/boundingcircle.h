#ifndef VALHALLA_BALDR_BOUNDING_CIRCLE_H_
#define VALHALLA_BALDR_BOUNDING_CIRCLE_H_

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/json.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace baldr {

// bin size in meters at the equator (half that since we offset from center) plus the largest
// radius we support
constexpr unsigned int kCoordinateBits = 13;
constexpr unsigned int kRadiiBits = 32 - kCoordinateBits * 2;
constexpr unsigned int kRadiiCount = 1 << kRadiiBits;
constexpr uint32_t kMaxOffsetValue = (1 << kCoordinateBits) - 1;

// TODO: derive empirically. Ideally, we keep a couple of these arrays around for
// tiles of different densities and store an identifier in the tile header that points
// to the array that best fits the edge shapes in the given tile
constexpr std::array<int, kRadiiCount> kBoundingCircleRadii =
    {2,   4,   5,   7,   10,  13,  15,   18,   20,   25,   30,   35,   40,   45,   50,   55,
     60,  65,  70,  75,  80,  85,  90,   95,   100,  110,  120,  130,  140,  150,  160,  170,
     180, 190, 200, 210, 220, 230, 240,  250,  275,  300,  325,  350,  375,  400,  500,  550,
     600, 650, 700, 750, 800, 900, 1000, 1100, 1200, 1300, 1500, 1750, 2000, 2500, 3000, 4000};

constexpr double kMaxOffsetMeters =
    0.05 * midgard::kMetersPerDegreeLat / 2 + kBoundingCircleRadii.back();
const double kOffsetIncrement = kMaxOffsetMeters / (1 << (kCoordinateBits - 1));

struct DiscretizedBoundingCircle {

  /**
   * Default constructor
   */
  DiscretizedBoundingCircle() : radius_index(0), x_offset(0), y_offset(0) {};

  /**
   * Constructor.
   *
   * @param bin_center_approx distance approximator for the current bin's center point
   * @param bin_center        the current bin's center point
   * @param circle_center     the bounding circle's center point
   * @param radius            the bounding circle's radius in meters
   * @return the index of the radius used, or the max radius index + 1 if it was too large
   */
  DiscretizedBoundingCircle(const midgard::DistanceApproximator<midgard::PointLL>& bin_center_approx,
                            const midgard::PointLL& bin_center,
                            const midgard::PointLL& circle_center,
                            double radius) {

    midgard::PointLL offset{circle_center.lng() - bin_center.lng(),
                            circle_center.lat() - bin_center.lat()};

    double x_meters = offset.lng() * bin_center_approx.GetLngScale() * midgard::kMetersPerDegreeLat;
    double y_meters = offset.lat() * midgard::kMetersPerDegreeLat;

    // if any of the offsets is larger than the largest value we support bail
    if (std::abs(x_meters) >= kMaxOffsetMeters - 0.5 || std::abs(y_meters) >= kMaxOffsetMeters - 0.5)
      return;

    // this gives us the offset values we store
    uint32_t x_offset_increments =
        static_cast<uint64_t>((x_meters + kMaxOffsetMeters) / kOffsetIncrement + 0.5);

    uint32_t y_offset_increments =
        static_cast<uint64_t>((y_meters + kMaxOffsetMeters) / kOffsetIncrement + 0.5);

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

    midgard::PointLL discretized_center{(discretized_x_offset / (bin_center_approx.GetLngScale() *
                                                                 midgard::kMetersPerDegreeLat)) +
                                            bin_center.lng(),
                                        discretized_y_offset / midgard::kMetersPerDegreeLat +
                                            bin_center.lat()};

    auto loss_of_precision = circle_center.Distance(discretized_center);
    radius += loss_of_precision;

    unsigned int index = kBoundingCircleRadii.size();
    for (unsigned int i = 0; i < kBoundingCircleRadii.size(); ++i) {
      if (radius <= kBoundingCircleRadii[i]) {
        index = i;
        break;
      }
    }
    // set the offset and radius if radius doesn't exceed the max radius we support
    if (index != kBoundingCircleRadii.size()) {
      radius_index = index;
      y_offset = y_offset_increments;
      x_offset = x_offset_increments;
    } else {
      // In the rare case the max supported radius is exceeded,
      // set the offset and radius to an impossible combination to indicate that we
      // couldn't store any actual information about the bounding circle.
      // This is necessary because we have to store something so that indices into the
      // edge bins and into the discretized bounding circles are equal.
      radius_index = 0;
      y_offset = kMaxOffsetValue;
      x_offset = kMaxOffsetValue;
    }
  }

  // TODO(chris): remove after testing
  std::pair<midgard::PointLL, double>
  get(const midgard::DistanceApproximator<midgard::PointLL>& approx,
      const midgard::PointLL& bin_center) {

    // for the case where we created a valid circle, but it happens to be at
    // the bin's center and has the smallest radius
    auto y_offset_meters =
        ((static_cast<double>(y_offset) / static_cast<double>(1 << kCoordinateBits)) *
         kMaxOffsetMeters * 2) -
        kMaxOffsetMeters;
    auto x_offset_meters =
        ((static_cast<double>(x_offset) / static_cast<double>(1 << kCoordinateBits)) *
         kMaxOffsetMeters * 2) -
        kMaxOffsetMeters;
    midgard::PointLL center{(x_offset_meters / (approx.GetMetersPerLngDegree())) + bin_center.lng(),
                            y_offset_meters / midgard::kMetersPerDegreeLat + bin_center.lat()};

    return {center, kBoundingCircleRadii[radius_index]};
  }

protected:
  uint32_t y_offset : kCoordinateBits; // y offset in meters from the bin center
  uint32_t x_offset : kCoordinateBits; // x offset in meters from the bin center
  uint32_t radius_index : kRadiiBits;  // index into the fixed size array of available radii
};
} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_GRAPHID_H_
