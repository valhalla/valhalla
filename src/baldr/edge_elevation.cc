#include "baldr/edge_elevation.h"
#include <cmath>
#include <valhalla/midgard/logging.h>

namespace valhalla {
namespace baldr {

// Constructor with arguments.
EdgeElevation::EdgeElevation(const float mean_elev,
                             const float max_up_slope,
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

}

} // namespace baldr
} // namespace valhalla
