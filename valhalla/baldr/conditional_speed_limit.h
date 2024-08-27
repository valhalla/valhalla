#pragma once

#include <cstdint>

namespace valhalla {
namespace baldr {

/** A combination of `TimeDomain` condition and a speed limit in a single 8b word */
struct ConditionalSpeedLimit {
  /**
   * Provides speed limit value in KPH.
   * @return Speed limit value in KPH.
   */
  uint32_t speed_limit() const {
    return speed_limit_;
  }

  /**
   * Provides the `TimeDomain` value of the condition, that defines when this speed limit is applied.
   * @return `TimeDomain` value of the condition.
   */
  uint64_t condition() const {
    return condition_;
  }

  uint64_t condition_ : 56;  // TimeDomain meaningful bits
  uint64_t speed_limit_ : 8; // speed limit in KPH
};
// This structure is used for storing data in tiles, so its size should be fixed
static_assert(sizeof(ConditionalSpeedLimit) == 8, "invalid ConditionalSpeedLimit struct size");

} // namespace baldr
} // namespace valhalla
