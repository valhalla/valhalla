#ifndef VALHALLA_BALDR_BOUNDING_CIRCLE_H_
#define VALHALLA_BALDR_BOUNDING_CIRCLE_H_

#include <array>
#include <cstdint>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/constants.h>
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
constexpr std::array<uint16_t, kRadiiCount> kBoundingCircleRadii =
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
  DiscretizedBoundingCircle() : radius_index(0), x_offset(0), y_offset(0){};

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
                            double radius);

  /**
   * Get the bounding circle's radius and center point.
   */
  std::pair<midgard::PointLL, uint16_t>
  get(const midgard::DistanceApproximator<midgard::PointLL>& approx,
      const midgard::PointLL& bin_center);

protected:
  uint32_t y_offset : kCoordinateBits; // y offset in meters from the bin center
  uint32_t x_offset : kCoordinateBits; // x offset in meters from the bin center
  uint32_t radius_index : kRadiiBits;  // index into the fixed size array of available radii
};
} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_GRAPHID_H_
