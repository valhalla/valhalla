#ifndef VALHALLA_BALDR_DATETIME_H_
#define VALHALLA_BALDR_DATETIME_H_

#include <string>
#include <memory>
#include <vector>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/local_time/tz_database.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/date_time/local_time/local_time_io.hpp>


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
   * @param   start_date
   * @param   end_date
   * @param   tile_date seconds from epoch
   * @param   tz timezone which is used to get the current time.
   * @param   dow_mask that this service runs.
   * @return  Returns the number of days.
   */
  uint64_t get_service_days(boost::gregorian::date& start_date, boost::gregorian::date& end_date,
                            const uint32_t tile_date, const uint32_t dow_mask);

  /**
   * Adds a service day to the days.
   * @param   days supported by the gtfs feed/service
   * @param   start_date
   * @param   end_date
   * @param   added_date in the format of 20150516 or 2015-05-06T08:00
   * @return  Returns the updated days.  Days will only be updated if the added date
   *          is in the start and end date range.
   */
  uint64_t add_service_day(const uint64_t& days, const boost::gregorian::date& start_date,
                           const boost::gregorian::date& end_date, const boost::gregorian::date& added_date);

  /**
   * Removes a service day to the days.
   * @param   days supported by the gtfs feed/service
   * @param   start_date
   * @param   end_date
   * @param   removed_date in the format of 20150516 or 2015-05-06T08:00
   * @return  Returns the updated days.  Days will only be updated if the removed date
   *          is in the start and end date range.
   */
  uint64_t remove_service_day(const uint64_t& days, const boost::gregorian::date& start_date,
                              const boost::gregorian::date& end_date, const boost::gregorian::date& removed_date);

  /**
   * Check if service is available for a date.
   * @param   days supported by the gtfs feed/service
   * @param   start_date in the format of days since pivot
   * @param   date the date in question...in the format of days since pivot.
   * @param   end_date in the format of days since pivot
   */
  bool is_service_available(const uint64_t days, const uint32_t start_date,
                            const uint32_t date, const uint32_t end_date);

  /**
   * Get the number of days elapsed from the pivot date until
   * inputed date.
   * @param   date_time
   * @return  Returns the number of days.
   */
  uint32_t days_from_pivot_date(const boost::gregorian::date& date_time);

  /**
   * Get the iso date and time from a DOW mask and time.
   * @param   dow_mask    Day of the week mask.
   * @param   time        Time in the format of 08:00
   * @param   time_zone   Timezone.
   * @return  Returns the formated date 2015-05-06.
   */
  std::string iso_date_time(const uint8_t dow_mask, const std::string& time,
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
   * Get the seconds from epoch
   * @param   date_time        date_time.  Time is already adjusted to TZ
   *
   * @return  Returns the seconds from epoch.
   */
  uint64_t seconds_since_epoch(const std::string& date_time, const boost::local_time::time_zone_ptr& time_zone);

  /**
   * Get the iso date time from seconds since epoch and timezone.
   * @param   origin_seconds      seconds since epoch for origin
   * @param   dest_seconds        seconds since epoch for dest
   * @param   origin_tz           timezone for origin
   * @param   dest_tz             timezone for dest
   * @param   iso_origin          origin string that will be updated
   * @param   iso_dest            dest string that will be updated
   */
  void seconds_to_date(const bool is_depart_at,
                       const uint64_t origin_seconds, const uint64_t dest_seconds,
                       const boost::local_time::time_zone_ptr& origin_tz,
                       const boost::local_time::time_zone_ptr& dest_tz,
                       std::string& iso_origin, std::string& iso_dest);

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

  /**
   * checks if string is in the format of %Y-%m-%dT%H:%M
   * @param   date_time should be in the format of 2015-05-06T08:00
   * @return true or false
   */
  bool is_iso_local(const std::string& date_time);
}
}
}
#endif  // VALHALLA_BALDR_DATETIME_H_
