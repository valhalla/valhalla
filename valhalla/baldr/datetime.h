#ifndef VALHALLA_BALDR_DATETIME_H_
#define VALHALLA_BALDR_DATETIME_H_

#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/date_time/local_time/local_time_io.hpp>
#include <boost/date_time/local_time/tz_database.hpp>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {
namespace DateTime {

struct tz_db_t : public boost::local_time::tz_database {
  tz_db_t();
  size_t to_index(const std::string& region) const;
  boost::shared_ptr<time_zone_base_type> from_index(size_t index) const;

protected:
  std::vector<std::string> regions;
};

/**
 * Get the timezone database singleton
 * @return  timezone database
 */
const tz_db_t& get_tz_db();

/**
 * Get a formatted testing date.  Currently, next Tuesday @ 08:00.
 * @return  Returns the formatted date string.
 */
std::string get_testing_date_time();

/**
 * Get a formatted date from a string.
 * @param date in the format of 20150516 or 2015-05-06T08:00
 * @return  Returns the formatted date.
 */
boost::gregorian::date get_formatted_date(const std::string& date);

/**
 * Get a local_date_time with support for dst.
 * @param date            Date
 * @param time_duration   Time
 * @param time_zone       Timezone
 *
 */
boost::local_time::local_date_time get_ldt(const boost::gregorian::date& date,
                                           const boost::posix_time::time_duration& time_duration,
                                           const boost::local_time::time_zone_ptr& time_zone);

/**
 * Get the days that this transit service is running in 60 days or less
 * @param   start_date start date
 * @param   end_date end date
 * @param   tile_date seconds from epoch
 * @param   dow_mask that this service runs.
 * @return  Returns the number of days.
 */
uint64_t get_service_days(boost::gregorian::date& start_date,
                          boost::gregorian::date& end_date,
                          const uint32_t tile_date,
                          const uint32_t dow_mask);

/**
 * Adds a service day to the days.
 * @param   days supported by the gtfs feed/service
 * @param   end_date end date
 * @param   tile_date seconds from epoch
 * @param   added_date in the format of 20150516 or 2015-05-06T08:00
 * @return  Returns the updated days.  Days will only be updated if the added date
 *          is in the start and end date range.
 */
uint64_t add_service_day(const uint64_t& days,
                         const boost::gregorian::date& end_date,
                         const uint32_t tile_date,
                         const boost::gregorian::date& added_date);

/**
 * Removes a service day to the days.
 * @param   days supported by the gtfs feed/service
 * @param   end_date end date
 * @param   tile_date seconds from epoch
 * @param   removed_date in the format of 20150516 or 2015-05-06T08:00
 * @return  Returns the updated days.  Days will only be updated if the removed date
 *          is in the start and end date range.
 */
uint64_t remove_service_day(const uint64_t& days,
                            const boost::gregorian::date& end_date,
                            const uint32_t tile_date,
                            const boost::gregorian::date& removed_date);

/**
 * Check if service is available for a date.
 * @param   days supported by the gtfs feed/service
 * @param   start_date in the format of days since pivot
 * @param   date the date in question...in the format of days since pivot.
 * @param   end_date in the format of days since pivot
 */
bool is_service_available(const uint64_t days,
                          const uint32_t start_date,
                          const uint32_t date,
                          const uint32_t end_date);

/**
 * Get the number of days elapsed from the pivot date until
 * inputed date.
 * @param   date_time date
 * @return  Returns the number of days.
 */
uint32_t days_from_pivot_date(const boost::gregorian::date& date_time);

/**
 * Get the iso date and time from a DOW mask and time.
 * @param   dow_mask    Day of the week mask.
 * @param   time        Time in the format of 08:00
 * @param   time_zone   Timezone.
 * @return  Returns the formatted date 2015-05-06.
 */
std::string iso_date_time(const uint8_t dow_mask,
                          const std::string& time,
                          const boost::local_time::time_zone_ptr& time_zone);

/**
 * Get the iso date and time from the current date and time.
 * @param   time_zone        Timezone.
 * @return  Returns the formated date 2015-05-06.
 *
 */
std::string iso_date_time(const boost::local_time::time_zone_ptr& time_zone);

/**
 * Get the seconds from epoch based on timezone.
 * @param   time_zone        Timezone.
 *
 * @return  Returns the seconds from epoch based on timezone.
 */
uint64_t seconds_since_epoch(const boost::local_time::time_zone_ptr& time_zone);

/**
 * Get the seconds from epoch for a date_time string
 * @param   date_time   date_time.
 * @param   time_zone   Timezone.
 *
 * @return  Returns the seconds from epoch.
 */
uint64_t seconds_since_epoch(const std::string& date_time,
                             const boost::local_time::time_zone_ptr& time_zone);

/**
 * Get the difference between two timezone using the seconds from epoch
 * (taking into account the timezones and dst) and add the difference to the seconds
 * @param   is_depart_at  is this a depart at or arrive by
 * @param   seconds       seconds since epoch for
 * @param   origin_tz     timezone for origin
 * @param   dest_tz       timezone for dest
 *
 */
void timezone_diff(const bool is_depart_at,
                   uint64_t& seconds,
                   const boost::local_time::time_zone_ptr& origin_tz,
                   const boost::local_time::time_zone_ptr& dest_tz);

std::string seconds_to_date(const uint64_t seconds, const boost::local_time::time_zone_ptr& tz);

/**
 * Get the iso date time from seconds since epoch and timezone.
 * @param   is_depart_at        is this a depart at or arrive by
 * @param   origin_seconds      seconds since epoch for origin
 * @param   dest_seconds        seconds since epoch for dest
 * @param   origin_tz           timezone for origin
 * @param   dest_tz             timezone for dest
 * @param   iso_origin          origin string that will be updated
 * @param   iso_dest            dest string that will be updated
 */
void seconds_to_date(const bool is_depart_at,
                     const uint64_t origin_seconds,
                     const uint64_t dest_seconds,
                     const boost::local_time::time_zone_ptr& origin_tz,
                     const boost::local_time::time_zone_ptr& dest_tz,
                     std::string& iso_origin,
                     std::string& iso_dest);

/**
 * Get the dow mask.
 * @param   date_time in the format of 20150516 or 2015-05-06T08:00
 * @return  Returns the dow mask.
 */
uint32_t day_of_week_mask(const std::string& date_time);

/**
 * Get the number of seconds elapsed from midnight.
 * Hours can be greater than 24.
 * @param   date_time in the format of 01:34:15 or 2015-05-06T08:00
 * @return  Returns the seconds from midnight.
 */
uint32_t seconds_from_midnight(const std::string& date_time);

/**
 * Add x seconds to a date_time and return a ISO date_time string.
 * @param   date_time   in the format of 01:34:15 or 2015-05-06T08:00
 * @param   seconds     seconds to add to the date.
 * @param   tz          timezone
 * @return  Returns ISO formatted string
 */
std::string get_duration(const std::string& date_time,
                         const uint32_t seconds,
                         const boost::local_time::time_zone_ptr& tz);

/**
 * checks if a date is restricted within a begin and end range.
 * @param   type          type of restriction kYMD or kNthDow
 * @param   begin_hrs     begin hours
 * @param   begin_mins    begin minutes
 * @param   end_hrs       end hours
 * @param   end_mins      end minutes
 * @param   dow           days of the week to apply this restriction
 * @param   begin_week    only set for kNthDow.  which week in the month
 * @param   begin_month   begin month
 * @param   begin_day_dow if kNthDow, then which dow to start the restriction.
 *                        if kYMD then it is the day of the month
 * @param   end_week      only set for kNthDow.  which week in the month
 * @param   end_month     end month
 * @param   end_day_dow   if kNthDow, then which dow to end the restriction.
 *                        if kYMD then it is the day of the month
 * @param   current_time  seconds since epoch
 * @param   time_zone     timezone for the date_time
 * @return true or false
 */

bool is_restricted(const bool type,
                   const uint8_t begin_hrs,
                   const uint8_t begin_mins,
                   const uint8_t end_hrs,
                   const uint8_t end_mins,
                   const uint8_t dow,
                   const uint8_t begin_week,
                   const uint8_t begin_month,
                   const uint8_t begin_day_dow,
                   const uint8_t end_week,
                   const uint8_t end_month,
                   const uint8_t end_day_dow,
                   const uint64_t current_time,
                   const boost::local_time::time_zone_ptr& time_zone);

/**
 * get the dow mask from user inputed string.  try to handle most inputs
 * @param   dow entered by a user
 * @return dow mask
 */
uint8_t get_dow_mask(const std::string& dow);

/**
 * get the dow from user inputed string.  try to handle most inputs
 * @param   dow entered by a user
 * @return DOW
 */
DOW get_dow(const std::string& dow);

/**
 * get the month from user inputed string.  try to handle most inputs
 * @param   month entered by a user
 * @return MONTH
 */
MONTH get_month(const std::string& month);

std::vector<uint64_t> get_time_range(const std::string& condition);

/**
 * Convert ISO 8601 time into std::tm.
 * @param iso  ISO time string (YYYY-mm-ddTmi:sec")
 * @return Returns std::tm time structure. If the input string is not valid this method
 *         sets tm_year to 0.
 */
static std::tm iso_to_tm(const std::string& iso) {
  // Create an invalid tm, then populate it from the ISO string using get_time
  std::tm t = {0, -1, -1, -1, -1, 0, 0, 0};

  // Check for invalid string (not the right separators and sizes)
  if (iso.size() != 16 || iso.at(4) != '-' || iso.at(7) != '-' || iso.at(10) != 'T' ||
      iso.at(13) != ':') {
    return t;
  }

  std::istringstream ss(iso);
  ss.imbue(std::locale(std::locale()));
  ss >> std::get_time(&t, "%Y-%m-%dT%H:%M");

  // Validate fields. Set tm_year to 0 if any of the year,month,day,hour,minute are invalid.
  if (t.tm_year > 200 || t.tm_mon < 0 || t.tm_mon > 11 || t.tm_mday < 0 || t.tm_mday > 31 ||
      t.tm_hour < 0 || t.tm_hour > 23 || t.tm_min < 0 || t.tm_min > 59) {
    t.tm_year = 0;
  }
  return t;
}

/**
 * Checks if string is in the format of %Y-%m-%dT%H:%M
 * @param   date_time should be in the format of 2015-05-06T08:00
 * @return true or false
 */
static bool is_iso_valid(const std::string& date_time) {
  return iso_to_tm(date_time).tm_year > 0;
}

/**
 * Get the day of the week given a time string
 * @param dt Date time string.
 */
static uint32_t day_of_week(const std::string& dt) {
  // Get the std::tm struct given the ISO string
  std::tm t = iso_to_tm(dt);

  // Use std::mktime to fill in day of week
  std::mktime(&t);
  return t.tm_wday;
}

} // namespace DateTime
} // namespace baldr
} // namespace valhalla
#endif // VALHALLA_BALDR_DATETIME_H_
