#pragma once

#include <cstdint>
#include <string>

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

  /**
   * String representation of the condition of this speed limit. Intended to use for debug purpose.
   * @return Condition string representation in the opening_hours format.
   */
  std::string condition_str() const;

  uint64_t condition_ : 56;  // TimeDomain meaningful bits
  uint64_t speed_limit_ : 8; // speed limit in KPH
};
// This structure is used for storing data in tiles, so its size should be fixed
static_assert(sizeof(ConditionalSpeedLimit) == 8, "invalid ConditionalSpeedLimit struct size");

} // namespace baldr
} // namespace valhalla
