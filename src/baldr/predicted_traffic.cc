#include <cmath>
#include <valhalla/midgard/logging.h>
#include "baldr/predicted_traffic.h"

namespace valhalla {
namespace baldr {

// Constructor with arguments.
PredictedTraffic:: PredictedTraffic(const float constrained_speed, const uint32_t compressed_offset) {
  set_constrained_speed(constrained_speed);
  set_compressed_offset(compressed_offset_);
}

// Set the mean elevation.
void PredictedTraffic::set_constrained_speed(const float constrained_speed) {
  if (constrained_speed > kMaxSpeedKph) {
    LOG_WARN("Exceeding maximum.  Constrainted Traffic speed: " + std::to_string(constrained_speed));
    constrained_speed_ = kMaxSpeedKph;
  } else {
    constrained_speed_ = constrained_speed;
  }
}

// Get the offset to the common edge data.
void PredictedTraffic::set_compressed_offset(const uint32_t compressed_offset) {
  if (compressed_offset > kMaxCompressedOffset) {
    // Consider this a catastrophic error
    LOG_ERROR("Exceeded maximum compressed traffic offset: " + std::to_string(compressed_offset));
    throw std::runtime_error("PredictedTraffic: exceeded maximum compressed traffic offset");
  } else {
    compressed_offset_ = compressed_offset;
  }
}

}
}
