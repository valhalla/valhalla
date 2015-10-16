#ifndef VALHALLA_BALDR_DATETIME_H_
#define VALHALLA_BALDR_DATETIME_H_

#include <string>
#include <memory>
#include <vector>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <boost/date_time/local_time/tz_database.hpp>

namespace valhalla {
namespace baldr {
namespace DateTime {

  struct tz_db_t : public boost::local_time::tz_database {
    tz_db_t();
    std::vector<std::string> regions;
  };

  /**
   * Get the timezone database singleton
   * @return  timezone database
   */
  const tz_db_t& get_tz_db();

  /**
   *
   * Get the list of regions.
   */
  std::vector<std::string> get_region_list();

  /**
   * Get a formatted date from a string.
   * @param date
   * @return  Returns the formatted date.
   */
  boost::gregorian::date get_formatted_date(const std::string& date);

  /**
   * Get the days that this transit service is running in 60 days or less
   * @param   start_date in the format of 20150516 or 2015-05-06T08:00
   * @param   end_date in the format of 20150516 or 2015-05-06T08:00
   * @param   tz timezone which is used to get the current time.
   * @param   dow_mask that this service runs.
   * @return  Returns the number of days.
   */
  uint64_t get_service_days(std::string& start_date, std::string& end_date,
                            const std::string& tz, const uint32_t& dow_mask);

  /**
   * Adds a service day to the days.
   * @param   days supported by the gtfs feed/service
   * @param   start_date in the format of 20150516 or 2015-05-06T08:00
   * @param   end_date in the format of 20150516 or 2015-05-06T08:00
   * @param   added_date in the format of 20150516 or 2015-05-06T08:00
   * @return  Returns the updated days.  Days will only be updated if the added date
   *          is in the start and end date range.
   */
  uint64_t add_service_day(const uint64_t& days, const std::string& start_date,
                           const std::string& end_date, const std::string& added_date);

  /**
   * Removes a service day to the days.
   * @param   days supported by the gtfs feed/service
   * @param   start_date in the format of 20150516 or 2015-05-06T08:00
   * @param   end_date in the format of 20150516 or 2015-05-06T08:00
   * @param   removed_date in the format of 20150516 or 2015-05-06T08:00
   * @return  Returns the updated days.  Days will only be updated if the removed date
   *          is in the start and end date range.
   */
  uint64_t remove_service_day(const uint64_t& days, const std::string& start_date,
                              const std::string& end_date, const std::string& removed_date);

  /**
   * Check if service is available for a date.
   * @param   days supported by the gtfs feed/service
   * @param   start_date in the format of days since pivot
   * @param   date the date in question...in the format of days since pivot.
   * @param   end_date in the format of days since pivot
   */
  bool is_service_available(const uint64_t& days, const uint32_t& start_date,
                            const uint32_t& date, const uint32_t& end_date);

  /**
   * Get the number of days elapsed from the pivot date until
   * inputed date.
   * @param   date_time in the format of 20150516 or 2015-05-06T08:00
   * @return  Returns the number of days.
   */
  uint32_t days_from_pivot_date(const std::string& date_time);

  /**
   * Get the time from the inputed date.
   * date_time is in the format of 2015-05-06T08:00
   * @param   date_time in the format of 2015-05-06T08:00
   * @return  Returns the formated time 8:00 AM.
   */
  std::string time(const std::string& date_time);

  /**
   * Get the date from the inputed date.
   * date_time is in the format of 2015-05-06T08:00
   * @param   date_time in the format of 2015-05-06T08:00
   * @return  Returns the formated date 2015-05-06.
   */
  std::string date(const std::string& date_time);

  /**
   * Get the iso date and time from a DOW mask and time.
   * @param   dow_mask  Day of the week mask.
   * @param   time      Time in the format of 08:00
   * @param   tz        Timezone.  Currently only support 2 timezones.
   *                    America/New_York and America/Los_Angeles and defaults
   *                    to America/New_York.
   */
  std::string iso_date_time(const uint8_t dow_mask, const std::string& time,
                            const std::string& tz = "America/New_York");

  /**
   * Get the iso date and time from the current date and time.
   * @param   tz        Timezone.  Currently only support 2 timezones.
   *                    America/New_York and America/Los_Angeles and defaults
   *                    to America/New_York.
   */
  std::string iso_date_time(const std::string& tz = "America/New_York");

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
   * @param   date_time in the format of 01:34:15 or 2015-05-06T08:00
   * @param   seconds to add to the date.
   * @return  Returns ISO formatted string
   */
  std::string get_duration(const std::string& date_time, const uint32_t seconds);

}
}
}
#endif  // VALHALLA_BALDR_DATETIME_H_
