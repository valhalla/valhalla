#ifndef VALHALLA_BALDR_EDGEELEVATION_H_
#define VALHALLA_BALDR_EDGEELEVATION_H_

#include <cstdint>
namespace valhalla {
namespace baldr {

// Use elevation bins of 8 meters to store mean elevation. Clamp to a range
// from -500 meters to 7683 meters.
constexpr uint32_t kMaxStoredElevation = 4095;  // 12 bits
constexpr float kElevationBinSize = 2.0f;
constexpr float kMinElevation = -500.0f;
constexpr float kMaxElevation = kMinElevation + (kElevationBinSize * kMaxStoredElevation);
constexpr float kNoElevationData = 32768.0f;

/**
 * Structure to store elevation information for a directed edge.
 */
class EdgeElevation {
 public:
  /**
   * Constructor with arguments.
   * @param  mean_elev       Mean elevation (meters).
   * @param  max_up_slope    Maximum up slope (degrees).
   * @param  max_down_slope  Maximum up slope (degrees).
   */
  EdgeElevation(const float mean_elev, const float max_up_slope,
                const float max_down_slope);

  /**
   * Get the mean elevation along the edge.
   * @return  Returns mean elevation in meters relative to sea level.
   */
  float mean_elevation() const {
    return kMinElevation + (mean_elevation_ * kElevationBinSize);
  }

  /**
   * Set the mean elevation.
   * @param  mean_elev  Mean elevation in meters.
   */
  void set_mean_elevation(const float mean_elev);

  /**
   * Gets the maximum upward slope. Uses 1 degree precision for slopes to 16
   * degrees and 4 degree precision afterwards (up to a max of 76 degrees).
   * @return  Returns the maximum upward slope (0 to 76 degrees).
   */
  int max_up_slope() const {
    return ((max_up_slope_ & 0x10) == 0) ? max_up_slope_ :
            16 + ((max_up_slope_ & 0xf) * 4);
  }

  /**
   * Sets the maximum upward slope. If slope is negative, 0 is set.
   * @param  slope  Maximum upward slope (degrees).
   */
  void set_max_up_slope(const float slope);

  /**
   * Gets the maximum downward slope. Uses 1 degree precision for slopes to
   * -16 degrees, and 4 degree precision afterwards (up to a max of -76 degs).
   * @return  Returns the maximum downward slope (0 to -76 degrees).
   */
  int max_down_slope() const {
    return ((max_down_slope_ & 0x10) == 0) ? -static_cast<int>(max_down_slope_) :
            -static_cast<int>(16 + ((max_down_slope_ & 0xf) * 4));
  }

  /**
   * Sets the maximum downward slope. If slope is positive, 0 is set.
   * @param  slope  Maximum downward slope (degrees).
   */
  void set_max_down_slope(const float slope);

 protected:
  uint32_t max_up_slope_    : 5;  // Maximum upward slope
  uint32_t max_down_slope_  : 5;  // Maximum downward slope
  uint32_t mean_elevation_  : 12; // Mean elevation with 2 meter precision
  uint32_t spare_           : 10;
};

}
}

#endif  // VALHALLA_BALDR_EDGEELEVATION_H_
