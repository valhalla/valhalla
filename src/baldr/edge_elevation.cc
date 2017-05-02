#include <cmath>
#include <valhalla/midgard/logging.h>
#include "baldr/edge_elevation.h"

namespace valhalla {
namespace baldr {

// Constructor with arguments.
EdgeElevation:: EdgeElevation(const float mean_elev, const float max_up_slope,
                              const float max_down_slope) {
  set_mean_elevation(mean_elev);
  set_max_up_slope(max_up_slope);
  set_max_down_slope(max_down_slope);
}

// Set the mean elevation.
void EdgeElevation::set_mean_elevation(const float mean_elev) {
  if (mean_elev < kMinElevation) {
    mean_elevation_ = 0;
  } else {
    uint32_t elev = static_cast<uint32_t>((mean_elev - kMinElevation) / kElevationBinSize);
    mean_elevation_ = (elev > kMaxStoredElevation) ? kMaxStoredElevation : elev;
  }
}

// Sets the maximum upward slope. If slope is negative, 0 is set.
void EdgeElevation::set_max_up_slope(const float slope) {
  if (slope < 0.0f) {
    max_up_slope_ = 0;
  } else if (slope < 16.0f) {
    max_up_slope_ = static_cast<int>(std::ceil(slope));
  } else if (slope < 76.0f) {
    max_up_slope_ = 0x10 | static_cast<int>(std::ceil((slope - 16.0f) * 0.25f));
  } else {
    max_up_slope_ = 0x1f;
  }
}

// Sets the maximum downward slope. If slope is positive, 0 is set.
void EdgeElevation::set_max_down_slope(const float slope) {
  if (slope > 0.0f) {
    max_down_slope_ = 0;
  } else if (slope > -16.0f) {
    max_down_slope_ = static_cast<int>(std::ceil(-slope));
  } else if (slope > -76.0f) {
    max_down_slope_ = 0x10 | static_cast<int>(std::ceil((-slope - 16.0f) * 0.25f));
  } else {
    max_down_slope_ = 0x1f;
  }
}

}
}
