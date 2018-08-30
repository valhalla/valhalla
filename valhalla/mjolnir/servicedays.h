#ifndef VALHALLA_MJOLNIR_SERVICEDAYS_H
#define VALHALLA_MJOLNIR_SERVICEDAYS_H

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

namespace {

const boost::gregorian::date sv_pivot_date_ =
    boost::gregorian::from_simple_string(valhalla::baldr::kPivotDate);
}

namespace valhalla {
namespace mjolnir {

struct boost_tz_db_t : public boost::local_time::tz_database {
  boost_tz_db_t();
  size_t to_index(const std::string& region) const;
  boost::shared_ptr<time_zone_base_type> from_index(size_t index) const;

protected:
  std::vector<std::string> regions;
};

/**
 * Get the timezone database singleton
 * @return  timezone database
 */
const boost_tz_db_t& get_tz_db();

/**
 * Get a formatted date from a string.
 * @param date in the format of 20150516 or 2015-05-06T08:00
 * @return  Returns the formatted date.
 */
boost::gregorian::date get_formatted_date(const std::string& date);

/**
 * Get the number of days elapsed from the pivot date until the input date.
 * @param   date_time date
 * @return  Returns the number of days.
 */
uint32_t days_from_pivot_date(const boost::gregorian::date& date_time);

/**
 * Get the iso date and time from the current date and time.
 * @param   time_zone        Timezone.
 * @return  Returns the formated date 2015-05-06.
 */
std::string iso_date_time(const boost::local_time::time_zone_ptr& time_zone);

/**
 * Get a formatted testing date.  Currently, next Tuesday @ 08:00.
 * @return  Returns the formatted date string.
 */
std::string get_testing_date_time();

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
 * Get the month from the input string.Try to handle most inputs.
 * @param   month entered by a user
 * @return MONTH
 */
baldr::MONTH get_month(const std::string& month);

std::vector<uint64_t> get_time_range(const std::string& condition);

} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_SERVICEDAYS_H
