#ifndef VALHALLA_BALDR_TRANSITSCHEDULE_H_
#define VALHALLA_BALDR_TRANSITSCHEDULE_H_

#include <cstdint>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Transit schedule validity information for departures from a stop.
 * Stores which days a departure at a given time is valid. A list of
 * unique schedule validity entries within a tile is kept in this record.
 */
class TransitSchedule {
public:
  /**
   * Constructor with arguments
   * @param  days     Bit field indicating the days (from tile creation
   *                  date) that a schedule entry is valid.
   * @param  dow      Days of week (beyond the end day) that the schedule
   *                  entry is valid.
   * @param  end_day  End day (from tile creation date) for this schedule
   *                  entry.
   */
  TransitSchedule(const uint64_t days, const uint32_t dow, const uint32_t end_day);

  /**
   * Gets the days that this departure is valid. Supports 64 days from tile
   * creation date.
   * @return  Returns the days
   */
  uint64_t days() const {
    return days_;
  }

  /**
   * Gets the days of the week for this departure.
   * @return  Returns the days of the week (bit mask)
   */
  uint32_t days_of_week() const {
    return days_of_week_;
  }

  /**
   * Get the end day for this scheduled departure (number of days
   * from tile creation date that is represented in days_).
   * @return  Returns the end day.
   */
  uint32_t end_day() const {
    return end_day_;
  }

  /**
   * Checks if the schedule entry is valid for the specified day and
   * day of week. If within end_day_ days of tile creation use the days mask
   * else fall back to the day of week bit mask.
   * @param  day  Days since tile creation.
   * @param  dow  Day of week.
   * @param  date_before_tile  Is the date prior to the tile creation date.
   * @return Returns true if the departure is valid, false otherwise.
   */
  bool IsValid(const uint32_t day, const uint32_t dow, bool date_before_tile) const {
    return (!date_before_tile && day <= end_day_) ? ((days_ & (1ULL << day)))
                                                  : ((days_of_week_ & dow) > 0);
  }

  // For sorting so we can make unique list of schedule records per tile
  bool operator<(const TransitSchedule& other) const {
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

protected:
  uint64_t days_; // Days this departure is active relative to the
                  // tile's creation date. Stores bit field with 1's
                  // meaning the departure applies to the day.

  uint64_t days_of_week_ : 7; // Days of the week. Bit mask.
  uint64_t end_day_ : 6;      // End day (what is our end day in the days_).
  uint64_t spare_ : 51;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_TRANSITSCHEDULE_H_
