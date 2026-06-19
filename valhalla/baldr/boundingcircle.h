#pragma once

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/pointll.h>

#include <cstdint>

namespace valhalla {
namespace baldr {

constexpr unsigned int kCoordinateBits = 13;
constexpr unsigned int kRadiiBits = 32 - kCoordinateBits * 2;
constexpr unsigned int kRadiiCount = 1 << kRadiiBits;
constexpr uint32_t kMaxOffsetValue = (1 << kCoordinateBits) - 1;

struct DiscretizedBoundingCircle {

  /**
   * Default constructor
   *
   * Creates an impossible bounding circle where the radius is smaller than the distance
   * to the bin (i.e. this edge cannot be possibly be intersecting the bin)
   */
  constexpr DiscretizedBoundingCircle()
      : y_offset_(kMaxOffsetValue), x_offset_(kMaxOffsetValue), radius_index_(0) {};

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
  std::pair<midgard::PointLL, double>
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
