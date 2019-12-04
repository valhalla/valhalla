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

#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

std::string to_iso_extended_string(const date::sys_days& d);

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
uint64_t get_service_days(date::sys_days& start_date,
                          date::sys_days& end_date,
                          const uint32_t& tile_date,
                          const uint32_t& dow_mask);
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
                         const date::sys_days& end_date,
                         const uint32_t& tile_date,
                         const date::sys_days& added_date);

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
                            const date::sys_days& end_date,
                            const uint32_t& tile_date,
                            const date::sys_days& removed_date);

/**
 * Shift all days by one in the futur, Friday 11th -> Saturday 12th
 * @param   days supported by the gtfs feed/service
 * @return  Returns the updated days.
 */
uint64_t shift_service_day(const uint64_t& days);

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
