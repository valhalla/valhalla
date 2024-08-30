#ifndef VALHALLA_BALDR_TIMEDOMAIN_H_
#define VALHALLA_BALDR_TIMEDOMAIN_H_

#include <valhalla/baldr/graphconstants.h>

#include <stdexcept>

namespace valhalla {
namespace baldr {

constexpr uint32_t kYMD = 0;
constexpr uint32_t kNthDow = 1;
constexpr uint32_t kMaxDateRangeWeek = 5;
constexpr uint32_t kMaxDateRangeDowMask = 127;
constexpr uint32_t kMaxDateRangeHrs = 23;
constexpr uint32_t kMaxDateRangeMins = 59;
constexpr uint32_t kMaxDateRangeMonth = 12;
constexpr uint32_t kMaxDateRangeDay = 31;
constexpr uint32_t kMaxDateRangeDow = 7;
union TimeDomain {
public:
  /**
   * Default constructor
   */
  TimeDomain() : value(0) {
  }

  /**
   * Constructor
   * @param value all the datetime bits as a value
   */
  explicit TimeDomain(const uint64_t value) : value(value) {
  }

  /**
   * Set the type.
   * @param  v  the type.
   */
  void set_type(const bool type) {
    daterange.type = type;
  }

  /**
   * Set the days of week this time domain is active.
   * @param dow  the dow.  This is a mask (e.g., Mo-Fr = 62)
   */
  void set_dow(const uint8_t dow) {
    if (dow > kMaxDateRangeDowMask) {
      throw std::runtime_error("Exceeding max dow value. Skipping");
    } else {
      daterange.dow = dow;
    }
  }

  /**
   * Set the hour when this time domain starts
   * @param begin_hrs  the begin_hrs.
   */
  void set_begin_hrs(const uint8_t begin_hrs) {
    if (begin_hrs == 24) {
      daterange.begin_hrs = 0;
    } else if (begin_hrs > kMaxDateRangeHrs) {
      throw std::runtime_error("Exceeding max begin hrs value. Skipping");
    } else {
      daterange.begin_hrs = begin_hrs;
    }
  }

  /**
   * Set the min when this time domain starts
   * @param begin_mins  the begin_mins.
   */
  void set_begin_mins(const uint8_t begin_mins) {
    if (begin_mins == 60) {
      daterange.begin_mins = 0;
    } else if (begin_mins > kMaxDateRangeMins) {
      throw std::runtime_error("Exceeding max begin mins value. Skipping");
    } else {
      daterange.begin_mins = begin_mins;
    }
  }

  /**
   * Set the month when this time domain starts
   * @param begin_month  the begin_month.
   */
  void set_begin_month(const uint8_t begin_month) {
    if (begin_month > kMaxDateRangeMonth) {
      throw std::runtime_error("Exceeding max begin month value. Skipping");
    } else {
      daterange.begin_month = begin_month;
    }
  }

  /**
   * Set the day when this time domain starts
   * @param begin_day_dow   the begin_day_dow.
   *                        begin_day_dow = begin day or dow enum (i.e. 1st Sunday in some month)
   */
  void set_begin_day_dow(const uint8_t begin_day_dow) {
    if (daterange.type == kYMD && begin_day_dow > kMaxDateRangeDay) {
      throw std::runtime_error("Exceeding max begin day value. Skipping");
    } else if (daterange.type == kNthDow && begin_day_dow > kMaxDateRangeDow) {
      throw std::runtime_error("Exceeding max begin dow value. Skipping");
    } else {
      daterange.begin_day_dow = begin_day_dow;
    }
  }

  /**
   * Set the week when this time domain starts
   *@param begin_week    the begin_week.
   *                     which week(1-5) does this start. (i.e. 1st week in Oct)
   */
  void set_begin_week(const uint8_t begin_week) {
    if (begin_week > kMaxDateRangeWeek) {
      throw std::runtime_error("Exceeding max begin week value. Skipping");
    } else {
      daterange.begin_week = begin_week;
    }
  }

  /**
   * Set the hour when this time domain ends
   * @param end_hrs  the end_hrs.
   */
  void set_end_hrs(const uint8_t end_hrs) {
    if (end_hrs == 24) {
      daterange.end_hrs = 0;
    } else if (end_hrs > kMaxDateRangeHrs) {
      throw std::runtime_error("Exceeding max end hrs value. Skipping");
    } else {
      daterange.end_hrs = end_hrs;
    }
  }

  /**
   * Set the min when this time domain ends
   * @return end_mins  the end_mins.
   */
  void set_end_mins(const uint8_t end_mins) {
    if (end_mins == 60) {
      daterange.end_mins = 0;
    } else if (end_mins > kMaxDateRangeMins) {
      throw std::runtime_error("Exceeding max end mins value. Skipping");
    } else {
      daterange.end_mins = end_mins;
    }
  }

  /**
   * Set the month when this time domain ends
   * @param end_month  the end_month.
   */
  void set_end_month(const uint8_t end_month) {
    if (end_month > kMaxDateRangeMonth) {
      throw std::runtime_error("Exceeding max end month value. Skipping");
    } else {
      daterange.end_month = end_month;
    }
  }

  /**
   * Set the day when this time domain ends
   * @param end_day_dow    the end_day_dow.
   *                       end_day_dow = end day or dow enum (i.e. 1st Sunday in some month)
   */
  void set_end_day_dow(const uint8_t end_day_dow) {
    if (daterange.type == kYMD && end_day_dow > kMaxDateRangeDay) {
      throw std::runtime_error("Exceeding max end day value. Skipping");
    } else if (daterange.type == kNthDow && end_day_dow > kMaxDateRangeDow) {
      throw std::runtime_error("Exceeding max end dow value. Skipping");
    } else {
      daterange.end_day_dow = end_day_dow;
    }
  }

  /**
   * Set the week when this time domain ends
   * @param end_week   the end_week.
   *                   which week(1-5) does this end. (i.e. 1st week in Oct)
   */
  void set_end_week(const uint8_t end_week) {
    if (end_week > kMaxDateRangeWeek) {
      throw std::runtime_error("Exceeding max end week value. Skipping");
    } else {
      daterange.end_week = end_week;
    }
  }

  /**
   * Gets the value.
   * @return   Returns the value.
   */
  uint64_t td_value() const {
    return value;
  }

  /**
   * Gets the type.
   * @return   Returns the type.
   */
  uint8_t type() const {
    return daterange.type;
  }

  /**
   * Gets the days of week this time domain is active.
   * @return   Returns the dow.  This is a mask (e.g., Mo-Fr = 62)
   */
  uint8_t dow() const {
    return daterange.dow;
  }

  /**
   * Gets the hour when this time domain starts
   * @return   Returns the begin_hrs.
   */
  uint8_t begin_hrs() const {
    return daterange.begin_hrs;
  }

  /**
   * Gets the min when this time domain starts
   * @return   Returns the begin_mins.
   */
  uint8_t begin_mins() const {
    return daterange.begin_mins;
  }

  /**
   * Gets the month when this time domain starts
   * @return   Returns the begin_month.
   */
  uint8_t begin_month() const {
    return daterange.begin_month;
  }

  /**
   * Gets the day when this time domain starts
   * @return   Returns the begin_day_dow.
   *           begin_day_dow = begin day or dow enum (i.e. 1st Sunday in some month)
   */
  uint8_t begin_day_dow() const {
    return daterange.begin_day_dow;
  }

  /**
   * Gets the week when this time domain starts
   * @return   Returns the begin_week.
   *           which week(1-5) does this start. (i.e. 1st week in Oct)
   */
  uint8_t begin_week() const {
    return daterange.begin_week;
  }

  /**
   * Gets the hour when this time domain ends
   * @return   Returns the end_hrs.
   */
  uint8_t end_hrs() const {
    return daterange.end_hrs;
  }

  /**
   * Gets the min when this time domain ends
   * @return   Returns the end_mins.
   */
  uint8_t end_mins() const {
    return daterange.end_mins;
  }

  /**
   * Gets the month when this time domain ends
   * @return   Returns the end_month.
   */
  uint8_t end_month() const {
    return daterange.end_month;
  }

  /**
   * Gets the day when this time domain ends
   * @return   Returns the end_day_dow.
   *           end_day_dow = end day or dow enum (i.e. 1st Sunday in some month)
   */
  uint8_t end_day_dow() const {
    return daterange.end_day_dow;
  }

  /**
   * Gets the week when this time domain ends
   * @return   Returns the end_week.
   *           which week(1-5) does this end. (i.e. 1st week in Oct)
   */
  uint8_t end_week() const {
    return daterange.end_week;
  }

  // Operator EqualTo.
  bool operator==(const TimeDomain& td) const {
    return value == td.value;
  }

  // Operator not equal
  bool operator!=(const TimeDomain& td) const {
    return value != td.value;
  }

  // cast operator
  operator uint64_t() const {
    return value;
  }

  /**
   * Provides a string representation of a condition in format close to the
   * [opening hours](https://wiki.openstreetmap.org/wiki/Key:opening_hours/specification)
   */
  std::string to_string() const;

protected:
  // represents a single date/time range from https://wiki.openstreetmap.org/wiki/Key:opening_hours
  struct DateRange {
    uint64_t type : 1;          // type of day_dow, 0 - day of month [1,31], 1 - nth day of week [1,7]
    uint64_t dow : 7;           // day of week mask, e.g. 0b0111110 for Mo-Fr as week starts from Su
    uint64_t begin_hrs : 5;     // begin hours, 0 if not set
    uint64_t begin_mins : 6;    // begin minutes, 0 if not set
    uint64_t begin_month : 4;   // begin month, from 1 (January) to 12 (December), 0 if not set
    uint64_t begin_day_dow : 5; // begin day of month or nth dow, i.e. 1st Sunday
    uint64_t begin_week : 3;    // which week does this start, i.e. 1st week in Oct
    uint64_t end_hrs : 5;       // end hours, 0 if not set
    uint64_t end_mins : 6;      // end minutes, 0 if not set
    uint64_t end_month : 4;     // end month, from 1 (January) to 12 (December), 0 if not set
    uint64_t end_day_dow : 5;   // end day of month or nth dow, i.e. last Sunday
    uint64_t end_week : 3;      // which week does this end, i.e. last week in Oct
    uint64_t spare : 10;

  } daterange;
  uint64_t value; // Single 64 bit value representing the date range.
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_TIMEDOMAIN_H_
