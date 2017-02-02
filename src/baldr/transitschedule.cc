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
TransitSchedule::TransitSchedule(const uint64_t days, const uint32_t dow,
                const uint32_t end_day)
    : days_(days),
      spare_(0) {
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

// Gets the days that this departure is valid.  Supports 64 days from tile
// creation date.
uint64_t TransitSchedule::days() const {
  return days_;
}

// Gets the days of the week for this departure.
uint32_t TransitSchedule::days_of_week() const {
  return days_of_week_;
}

// Get the end day for this scheduled departure.
uint32_t TransitSchedule::end_day() const {
  return end_day_;
}

// Checks if the schedule entry is valid for the specified day and
// day of week. If within end_day_ days of tile creation use the days mask
// else fall back to the day of week mask.
bool TransitSchedule::IsValid(const uint32_t day, const uint32_t dow,
             bool date_before_tile) const {
  if (!date_before_tile && day <= end_day_) {
    // Check days bit
    return ((days_ & (1ULL << day)));
  } else {
    return ((days_of_week_ & dow) > 0);
  }
}

// For sorting so we can make unique list of schedule records per tile
 bool TransitSchedule::operator < (const TransitSchedule& other) const {
   if (days_ == other.days_) {
     if (days_of_week_ == other.days_of_week_) {
       return end_day_ < other.end_day_;
     } else {
       return days_of_week_ < other.days_of_week_;
     }
   } else {
     return days_ < other.days_;
   }
 }

}
}
