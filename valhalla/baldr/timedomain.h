#ifndef VALHALLA_BALDR_TIMEDOMAIN_H_
#define VALHALLA_BALDR_TIMEDOMAIN_H_

#include <valhalla/baldr/graphconstants.h>
#include <vector>

namespace valhalla {
namespace baldr {

constexpr uint32_t kYMD    = 0;
constexpr uint32_t kNthDow = 1;

union TimeDomain {
 public:
  /**
   * Default constructor
   */
  TimeDomain()
      : value(0) {
  }

  /**
   * Constructor
   * @param value all the datetime bits as a value
   */
  explicit TimeDomain(const uint64_t value)
      : value(value) {
  }

  /**
   * Gets the type.
   * @return   Returns the type.
   */
  uint32_t type() const {
    return daterange.type;
  }

  /**
   * Gets the days of week this time domain is active.
   * @return   Returns the dow.  This is a mask (e.g., Mo-Fr = 62)
   */
  uint32_t dow() const {
    return daterange.dow;
  }

  /**
   * Gets the hour when this time domain starts
   * @return   Returns the begin_hrs.
   */
  uint32_t begin_hrs() const {
    return daterange.begin_hrs;
  }

  /**
   * Gets the min when this time domain starts
   * @return   Returns the begin_mins.
   */
  uint32_t begin_mins() const {
    return daterange.begin_mins;
  }

  /**
   * Gets the month when this time domain starts
   * @return   Returns the begin_month.
   */
  uint32_t begin_month() const {
    return daterange.begin_month;
  }

  /**
   * Gets the day when this time domain starts
   * @return   Returns the begin_day_dow.
   *           begin_day_dow = begin day or dow enum (i.e. 1st Sunday in some month)
   */
  uint32_t begin_day() const {
    return daterange.begin_day_dow;
  }

  /**
   * Gets the dow when this time domain starts
   * @return   Returns the begin_day_dow.
   *           begin_day_dow = begin day or dow enum (i.e. 1st Sunday in some month)
   */
  uint32_t begin_dow() const {
    return daterange.begin_day_dow;
  }

  /**
   * Gets the year when this time domain starts
   * @return   Returns the begin_year_week.
   *           begin_year_week = begin year or which week(1-5) does this
   *           start. (i.e. 1st week in Oct)
   */
  uint32_t begin_year() const {
    return daterange.begin_year_week;
  }

  /**
   * Gets the week when this time domain starts
   * @return   Returns the begin_year_week.
   *           begin_year_week = begin year or which week(1-5) does this
   *           start. (i.e. 1st week in Oct)
   */
  uint32_t begin_week() const {
    return daterange.begin_year_week;
  }

  /**
   * Gets the hour when this time domain ends
   * @return   Returns the end_hrs.
   */
  uint32_t end_hrs() const {
    return daterange.end_hrs;
  }

  /**
   * Gets the min when this time domain ends
   * @return   Returns the end_mins.
   */
  uint32_t end_mins() const {
    return daterange.end_mins;
  }

  /**
   * Gets the month when this time domain ends
   * @return   Returns the end_month.
   */
  uint32_t end_month() const {
    return daterange.end_month;
  }

  /**
   * Gets the day when this time domain ends
   * @return   Returns the end_day_dow.
   *           end_day_dow = end day or dow enum (i.e. 1st Sunday in some month)
   */
  uint32_t end_day() const {
    return daterange.end_day_dow;
  }

  /**
   * Gets the dow when this time domain ends
   * @return   Returns the end_day_dow.
   *           end_day_dow = end day or dow enum (i.e. 1st Sunday in some month)
   */
  uint32_t end_dow() const {
    return daterange.end_day_dow;
  }

  /**
   * Gets the year when this time domain ends
   * @return   Returns the end_year_week.
   *           end_year_week = end year or which week(1-5) does this
   *           end. (i.e. 1st week in Oct)
   */
  uint32_t end_year() const {
    return daterange.end_year_week;
  }

  /**
   * Gets the week when this time domain ends
   * @return   Returns the end_year_week.
   *           end_year_week = end year or which week(1-5) does this
   *           end. (i.e. 1st week in Oct)
   */
  uint32_t end_week() const {
    return daterange.end_year_week;
  }

  // Operator EqualTo.
  bool operator ==(const TimeDomain& td) const {
    return value == td.value;
  }

  // Operator not equal
  bool operator !=(const TimeDomain& td) const {
    return value != td.value;
  }

  // cast operator
  operator uint64_t() const {
    return value;
  }

  struct DateRange {
    uint64_t type             :  1;
    uint64_t dow              :  7; // day of week for this restriction
    uint64_t begin_hrs        :  5; // begin hours
    uint64_t begin_mins       :  6; // begin minutes
    uint64_t begin_month      :  4; // begin month
    uint64_t begin_day_dow    :  5; // begin day or dow enum i.e. 1st Sunday
    uint64_t begin_year_week  :  8; // begin year or which week does this start.  i.e. 1st week in Oct
    uint64_t end_hrs          :  5; // end hours
    uint64_t end_mins         :  6; // end minutes
    uint64_t end_month        :  4; // end month
    uint64_t end_day_dow      :  5; // end day or dow enum i.e. last Sunday
    uint64_t end_year_week    :  8; // end year or which week does this end.  i.e. last week in Oct

  } daterange;
  uint64_t value; // Single 64 bit value representing the date range.

};

}
}

#endif // VALHALLA_BALDR_TIMEDOMAIN_H_
