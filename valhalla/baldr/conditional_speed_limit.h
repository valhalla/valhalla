#pragma once

#include <cstdint>
#include <string>

namespace valhalla {
namespace baldr {

struct ConditionalSpeedLimit {
  /**
   * Constructs `ConditionalSpeedLimit` from the provided speed and condition.
   * @param speed Speed limit value in KPH
   * @param condition `TimeDomain` value of the condition. Can be accuired from the
   * `TimeDomain::td_value()` or from `get_time_range(opening_hours_str)`.
   */
  explicit ConditionalSpeedLimit(uint8_t speed, uint64_t condition);

  ConditionalSpeedLimit() : ConditionalSpeedLimit(0, 0) {
  }

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
  uint64_t condition() const;

  /**
   * String representation of the condition of this speed limit. Intended to use for debug purpose.
   * @return Condition string representation in the opening_hours format.
   */
  std::string condition_str() const;

private:
  // This structure basically copies `TimeDomain` definition and adds `speed_limit` value into the
  // `TimeDomain::spare` bits, which allows us to use only 8 bytes per conditional speed limit.
  uint64_t day_dow_type_ : 1;  // type of day_dow, 0 - day of month [1,31], 1 - nth day of week [1,7]
  uint64_t dow_mask_ : 7;      // day of week mask, e.g. 0b0111110 for Mo-Fr as week starts from Su
  uint64_t begin_hrs_ : 5;     // begin hours, 0 if not set
  uint64_t begin_mins_ : 6;    // begin minutes, 0 if not set
  uint64_t begin_month_ : 4;   // begin month, from 1 (January) to 12 (December), 0 if not set
  uint64_t begin_day_dow_ : 5; // begin day of month or nth dow, i.e. 1st Sunday
  uint64_t begin_week_ : 3;    // which week does this start, i.e. 1st week in Oct
  uint64_t end_hrs_ : 5;       // end hours, 0 if not set
  uint64_t end_mins_ : 6;      // end minutes, 0 if not set
  uint64_t end_month_ : 4;     // end month, from 1 (January) to 12 (December), 0 if not set
  uint64_t end_day_dow_ : 5;   // end day of month or nth dow, i.e. last Sunday
  uint64_t end_week_ : 3;      // which week does this end, i.e. last week in Oct
  uint64_t spare_ : 2;

  uint64_t speed_limit_ : 8; // speed limit in KPH
};
// This structure is used for storing data in tiles, so its size should be fixed
static_assert(sizeof(ConditionalSpeedLimit) == 8, "invalid ConditionalSpeedLimit struct size");

} // namespace baldr
} // namespace valhalla
