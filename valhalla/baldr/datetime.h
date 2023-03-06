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
#include <unordered_map>
#include <vector>

// date emits a warning otherwise for C++17, see
// https://github.com/valhalla/valhalla/pull/3878#issuecomment-1365487437
#define HAS_UNCAUGHT_EXCEPTIONS 1

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <date/date.h>
#include <date/tz.h>
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#endif

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/midgard/constants.h>

#include <valhalla/proto/common.pb.h>

namespace valhalla {
namespace baldr {
namespace DateTime {

// tz db
struct tz_db_t {
  tz_db_t();
  size_t to_index(const std::string& zone) const;
  const date::time_zone* from_index(size_t index) const;

protected:
  std::unordered_map<std::string, size_t> names;
  const date::tzdb& db;
};

/**
 * Get the timezone database singleton
 * @return  timezone database
 */
const tz_db_t& get_tz_db();

/**
 * Get a formatted date from a string.
 * @param date       in the format of 2015-05-06T08:00
 * @param can_throw  if true and the input is malformed invalid_argument is thrown
 *                   TODO: remove the option to disallow throwing
 * @return  Returns the formatted date.
 */
date::local_seconds get_formatted_date(const std::string& date, bool can_throw = false);

/**
 * Get a local_date_time with support for dst.
 * @param date            Date
 * @param time_duration   Time
 * @param time_zone       Timezone
 * @return Returns local date time.
 */
date::zoned_seconds get_ldt(const date::local_seconds& date, const date::time_zone* time_zone);

/**
 * Get the number of days elapsed from the pivot date until the input date.
 * @param   date_time date
 * @return  Returns the number of days.
 */
uint32_t days_from_pivot_date(const date::local_seconds& seconds);

/**
 * Get the iso date and time from the current date and time.
 * @param   time_zone        Timezone.
 * @return  Returns the formated date 2015-05-06.
 */
std::string iso_date_time(const date::time_zone* time_zone);

/**
 * Get the seconds from epoch for a date_time string
 * @param   date_time   date_time.
 * @param   time_zone   Timezone.
 * @return  Returns the seconds from epoch.
 */
uint64_t seconds_since_epoch(const std::string& date_time, const date::time_zone* time_zone);

/**
 * Get the difference between two timezones using the current time (seconds from epoch
 * so that DST can be take into account).
 * @param   seconds       seconds since epoch
 * @param   origin_tz     timezone for origin
 * @param   dest_tz       timezone for dest
 * @param   cache         a cache for timezone sys_info lookup (since its expensive)
 * @return Returns the seconds difference between the 2 timezones.
 */
using tz_sys_info_cache_t = std::unordered_map<const date::time_zone*, std::vector<date::sys_info>>;
int timezone_diff(const uint64_t seconds,
                  const date::time_zone* origin_tz,
                  const date::time_zone* dest_tz,
                  tz_sys_info_cache_t* cache = nullptr);

/**
 * Get the iso date time from seconds since epoch and timezone.
 * @param   seconds      seconds since epoch
 * @param   tz           timezone
 * @param   tz_format    whether or not to include the tz in the formatted string
 */
std::string seconds_to_date(const uint64_t seconds, const date::time_zone* tz, bool tz_format = true);

/**
 * Get the iso date time from seconds since epoch and timezone.
 * @param   origin_seconds      seconds since epoch for origin
 * @param   dest_seconds        seconds since epoch for dest
 * @param   origin_tz           timezone for origin
 * @param   dest_tz             timezone for dest
 * @param   iso_origin          origin string that will be updated
 * @param   iso_dest            dest string that will be updated
 */
void seconds_to_date(const uint64_t origin_seconds,
                     const uint64_t dest_seconds,
                     const date::time_zone* origin_tz,
                     const date::time_zone* dest_tz,
                     std::string& iso_origin,
                     std::string& iso_dest);

/**
 * Get utc formatted timestamp, skip using zoned times since we have utc special case
 * @param seconds since epoch in UTC zone
 * @return formated string like: 2020-08-12T14:17:09Z
 */
inline std::string seconds_to_date_utc(const uint64_t seconds) {
  std::stringstream ss;
  std::chrono::seconds secs(seconds);
  date::sys_seconds timestamp(secs);
  auto t = std::chrono::system_clock::to_time_t(timestamp);
  ss << std::put_time(std::gmtime(&t), "%FT%TZ");
  return ss.str();
}

/**
 * Get the dow mask.
 * @param   date_time in the format of 20150516 or 2015-05-06T08:00
 * @return  Returns the dow mask.
 */
uint32_t day_of_week_mask(const std::string& date_time);

/**
 * Add x seconds to a date_time and return a ISO date_time string.
 * @param   date_time   in the format of 01:34:15 or 2015-05-06T08:00
 * @param   seconds     seconds to add to the date.
 * @param   tz          timezone
 * @return  Returns ISO formatted string
 */
std::string
get_duration(const std::string& date_time, const uint32_t seconds, const date::time_zone* tz);

/**
 * Checks if a date is restricted within a begin and end range.
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
bool is_conditional_active(const bool type,
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
                           const date::time_zone* time_zone);

/**
 * Gets the second of the week in local time from an epoch time and timezone
 * @param epoch_time   the time from which to offset
 * @param time_zone
 * @return the second of the week accounting for timezone transformation from epoch time
 */
uint32_t second_of_week(uint32_t epoch_time, const date::time_zone* time_zone);

/**
 * Convert ISO 8601 time into std::tm.
 * @param iso  ISO time string (YYYY-mm-ddTHH:MM)
 * @return Returns std::tm time structure. If the input string is not valid this method
 *         sets tm_year to 0.
 */
static inline std::tm iso_to_tm(const std::string& iso) {
  // Create an invalid tm, then populate it from the ISO string using get_time
  std::tm t = {};

  // Check for invalid string (not the right separators and sizes)
  if (iso.size() != 16 || iso.at(4) != '-' || iso.at(7) != '-' || iso.at(10) != 'T' ||
      iso.at(13) != ':') {
    return t;
  }

  std::istringstream ss(iso);
  ss.imbue(std::locale("C"));
  ss >> std::get_time(&t, "%Y-%m-%dT%H:%M");

  // If parsing failed zero 0 the struct
  if (ss.fail()) {
    return {};
  }
  return t;
}

/**
 * Checks if string is in the format of %Y-%m-%dT%H:%M
 * @param   date_time should be in the format of 2015-05-06T08:00
 * @return true or false
 */
static inline bool is_iso_valid(const std::string& date_time) {
  return iso_to_tm(date_time).tm_year > 0;
}

/**
 * Get the day of the week given a time string
 * Time string must be of the format: YYYY-mm-ddTHH:MM
 * @param dt Date time string.
 */
static inline uint32_t day_of_week(const std::string& dt) {
  // Get the std::tm struct given the ISO string
  std::tm t = iso_to_tm(dt);

  // Use std::mktime to fill in day of week
  std::mktime(&t);
  return t.tm_wday;
}

/**
 * Get the number of seconds elapsed from midnight. Hours can be greater than 24
 * to allow support for transit schedules. See GTFS spec:
 * https://developers.google.com/transit/gtfs/reference#stop_times_fields
 * @param   date_time in the format HH:MM:SS or HH:MM or YYYY-MM-DDTHH:MM
 *          (examples: 01:34:15 or 2015-05-06T08:00)
 * @return  Returns the seconds from midnight.
 */
static inline uint32_t seconds_from_midnight(const std::string& date_time) {
  std::string str;
  std::size_t found = date_time.find('T'); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos) {
    str = date_time.substr(found + 1);
  } else {
    str = date_time;
  }

  // Split the string by the delimiter ':'
  int secs = 0;
  int multiplier = static_cast<int>(midgard::kSecondsPerHour);
  std::string item;
  std::stringstream ss(str);
  while (std::getline(ss, item, ':')) {
    secs += std::stoi(item) * multiplier;
    multiplier = (multiplier == midgard::kSecondsPerHour) ? midgard::kSecondsPerMinute : 1;
  }
  return secs;
}

/**
 * Returns seconds of week within the range [0, kSecondsPerWeek]
 * @param  secs  Seconds within the week.
 * @return Returns the seconds within the week within the valid range.
 */
static inline int32_t normalize_seconds_of_week(const int32_t secs) {
  if (secs < 0) {
    return secs + midgard::kSecondsPerWeek;
  } else if (secs > int32_t(midgard::kSecondsPerWeek)) {
    return secs - midgard::kSecondsPerWeek;
  } else {
    return secs;
  }
}

const date::local_seconds pivot_date_ = get_formatted_date(kPivotDate + "T00:00");

} // namespace DateTime
} // namespace baldr
} // namespace valhalla
#endif // VALHALLA_BALDR_DATETIME_H_
