#pragma once

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/pointll.h>

#include <array>
#include <cstdint>

namespace valhalla {
namespace baldr {

// bin size in meters at the equator (half that since we offset from center) plus the largest
// radius we support
constexpr unsigned int kCoordinateBits = 13;
constexpr unsigned int kRadiiBits = 32 - kCoordinateBits * 2;
constexpr unsigned int kRadiiCount = 1 << kRadiiBits;
constexpr uint32_t kMaxOffsetValue = (1 << kCoordinateBits) - 1;

constexpr std::array<uint16_t, kRadiiCount> kBoundingCircleRadii =
    {1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  13,  15,  17,   18,   20,   23,
     25,  27,  30,  35,  40,  43,  45,  50,  55,  60,  65,  70,  75,   80,   85,   90,
     95,  100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210,  220,  230,  240,
     250, 275, 300, 325, 350, 375, 400, 500, 550, 600, 700, 800, 1000, 1500, 2000, 2500};
// Optimal
// constexpr std::array<uint16_t, kRadiiCount> kBoundingCircleRadii =
//     {3,   5,   7,   10,  12,  15,   18,   21,   24,   27,   30,   33,   36,   39,   43,   47,
//      51,  55,  60,  65,  70,  76,   82,   89,   96,   104,  113,  123,  133,  143,  155,  167,
//      181, 195, 211, 228, 246, 266,  288,  311,  336,  365,  396,  430,  465,  507,  556,  611,
//      668, 730, 801, 890, 990, 1108, 1251, 1418, 1625, 1853, 2115, 2384, 2782, 3104, 3512, 4000};

constexpr double kMaxOffsetMeters =
    0.05 * midgard::kMetersPerDegreeLat / 2 + kBoundingCircleRadii.back();
constexpr double kOffsetIncrement = kMaxOffsetMeters / (1 << (kCoordinateBits - 1));

struct DiscretizedBoundingCircle {

  /**
   * Default constructor
   *
   * Creates an impossible bounding circle where the radius is smaller than the distance
   * to the bin (i.e. this edge cannot be possibly be intersecting the bin)
   */
  DiscretizedBoundingCircle()
      : y_offset(kMaxOffsetValue), x_offset(kMaxOffsetValue), radius_index(0){};

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

  bool operator==(const DiscretizedBoundingCircle& rhs) const {
    return y_offset == rhs.y_offset && x_offset == rhs.x_offset && radius_index == rhs.radius_index;
  }

  inline bool is_valid() const {
    return x_offset != kMaxOffsetValue && y_offset != kMaxOffsetValue && radius_index != 0;
  }

  friend std::ostream& operator<<(std::ostream& os, const DiscretizedBoundingCircle& circle);

  inline uint32_t get_y_offset() const {
    return y_offset;
  }

  inline uint32_t get_x_offset() const {
    return x_offset;
  }

  inline uint32_t get_radius_index() const {
    return radius_index;
  }

protected:
  uint32_t y_offset : 13;    // y offset in meters from the bin center
  uint32_t x_offset : 13;    // x offset in meters from the bin center
  uint32_t radius_index : 6; // index into the fixed size array of available radii
};
} // namespace baldr
} // namespace valhalla

namespace std {
inline std::string to_string(const valhalla::baldr::DiscretizedBoundingCircle& circle) {
  return "x=" + std::to_string(circle.get_x_offset()) +
         "|y=" + std::to_string(circle.get_y_offset()) +
         "|r=" + std::to_string(circle.get_radius_index());
}
} // namespace std
