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

// the possible radii for our discretized bounding circles. we only store indices into this
// array on disk, so we predefine the set of all possible radii here loosely
// based on the distribution of all actual bounding circles in a planet (most edges are
// small, so we want a finer resolution in the 1-20 meter range)
constexpr std::array<uint16_t, kRadiiCount> kBoundingCircleRadii =
    {1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  13,  15,  17,   18,   20,   23,
     25,  27,  30,  35,  40,  43,  45,  50,  55,  60,  65,  70,  75,   80,   85,   90,
     95,  100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210,  220,  230,  240,
     250, 275, 300, 325, 350, 375, 400, 500, 550, 600, 700, 800, 1000, 1500, 2000, 2500};

constexpr uint32_t kInvalidRadiusIndex = kBoundingCircleRadii.size();

// the maximum offset in the x and y axis we support is derived from
//   - half the size of the bin (since we compute offsets relative to the center) plus
//   - the radius of the largest radius we support
constexpr double kMaxOffsetMeters =
    0.05 * midgard::kMetersPerDegreeLat / 2 + kBoundingCircleRadii.back();

// the offset resolution is the max offset divided by the number of unique values
// given the number of bits we use for each offset
constexpr double kOffsetIncrement = kMaxOffsetMeters / (1 << (kCoordinateBits - 1));

struct DiscretizedBoundingCircle {

  /**
   * Default constructor
   *
   * Creates an impossible bounding circle where the radius is smaller than the distance
   * to the bin (i.e. this edge cannot be possibly be intersecting the bin)
   */
  constexpr DiscretizedBoundingCircle()
      : y_offset_(kMaxOffsetValue), x_offset_(kMaxOffsetValue), radius_index_(0){};

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
   * Get the bounding circle's absolute center point lat/lon and radius (in meters).
   */
  std::pair<midgard::PointLL, uint16_t>
  get(const midgard::DistanceApproximator<midgard::PointLL>& approx,
      const midgard::PointLL& bin_center) const;

  bool operator==(const DiscretizedBoundingCircle& rhs) const {
    return y_offset_ == rhs.y_offset_ && x_offset_ == rhs.x_offset_ &&
           radius_index_ == rhs.radius_index_;
  }

  inline bool is_valid() const {
    // impossible combination of maximum possible offsets but smallest possible radius
    return !(x_offset_ == kMaxOffsetValue && y_offset_ == kMaxOffsetValue && radius_index_ == 0);
  }

  friend std::ostream& operator<<(std::ostream& os, const DiscretizedBoundingCircle& circle);

  inline uint32_t y_offset() const {
    return y_offset_;
  }

  inline uint32_t x_offset() const {
    return x_offset_;
  }

  inline uint32_t radius_index() const {
    return radius_index_;
  }

protected:
  uint32_t y_offset_ : kCoordinateBits; // y offset in meters from the bin center
  uint32_t x_offset_ : kCoordinateBits; // x offset in meters from the bin center
  uint32_t radius_index_ : kRadiiBits;  // index into the fixed size array of available radii
};

static_assert(sizeof(DiscretizedBoundingCircle) == 4);

} // namespace baldr
} // namespace valhalla

namespace std {
inline std::string to_string(const valhalla::baldr::DiscretizedBoundingCircle& circle) {
  return "x=" + std::to_string(circle.x_offset()) + "|y=" + std::to_string(circle.y_offset()) +
         "|r=" + std::to_string(circle.radius_index());
}
} // namespace std
