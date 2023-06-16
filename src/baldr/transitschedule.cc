#include <stdexcept>

#include "baldr/transitschedule.h"
#include "midgard/logging.h"

namespace valhalla {
namespace baldr {

/**
 * Constructor with arguments
 * @param  days      Bit field indicating the days (from tile creation
 *                   date) that a schedule entry is valid.
 * @param  dow       Days of week (beyond the end day) that the schedule
 *                   entry is valid.
 * @param  end_day   End day (from tile creation date) for this schedule
 *                  entry.
 */
TransitSchedule::TransitSchedule(const uint64_t days, const uint32_t dow, const uint32_t end_day)
    : days_(days), spare_(0) {
  // Validate inputs
  if (dow > kAllDaysOfWeek) {
    throw std::runtime_error("TransitSchedule: Exceeded days of week mask");
  }
  days_of_week_ = dow;

  // If exceeds kMaxEndDay
  if (end_day > kMaxEndDay) {
    LOG_ERROR("TransitSchedule: Exceeded maximum end day");
    end_day_ = kMaxEndDay;
  } else {
    end_day_ = end_day;
  }
}

} // namespace baldr
} // namespace valhalla
