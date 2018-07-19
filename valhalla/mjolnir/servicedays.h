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
#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphconstants.h>

namespace {
const boost::gregorian::date pivot_date_ =
    boost::gregorian::from_undelimited_string(valhalla::baldr::kPivotDate);
}

namespace valhalla {
namespace mjolnir {

/**
 * Get a formatted testing date.  Currently, next Tuesday @ 08:00.
 * @return  Returns the formatted date string.
 */
static std::string get_testing_date_time() {
  auto tz = baldr::DateTime::get_tz_db().from_index(
      baldr::DateTime::get_tz_db().to_index("America/New_York"));
  boost::gregorian::date d = baldr::DateTime::get_formatted_date(baldr::DateTime::iso_date_time(tz));

  while (d.day_of_week() != boost::date_time::Tuesday) {
    d += boost::gregorian::days(1);
  }

  return to_iso_extended_string(d) + "T08:00";
}

/**
 * Get the days that this transit service is running in 60 days or less
 * @param   start_date start date
 * @param   end_date end date
 * @param   tile_date seconds from epoch
 * @param   dow_mask that this service runs.
 * @return  Returns the number of days.
 */
static uint64_t get_service_days(boost::gregorian::date& start_date,
                                 boost::gregorian::date& end_date,
                                 const uint32_t tile_date,
                                 const uint32_t dow_mask);
// Get service days
// Start from the tile_date or start date to end date or 60 days (whichever is less)
// start_date will be updated to the tile creation date if the start date is in the past
// set the bits based on the dow.
uint64_t get_service_days(boost::gregorian::date& start_date,
                          boost::gregorian::date& end_date,
                          const uint32_t tile_date,
                          const uint32_t dow_mask) {

  boost::gregorian::date tile_header_date = pivot_date_ + boost::gregorian::days(tile_date);

  // if our start date is more than 60 days out, reject.
  if (start_date > (tile_header_date + boost::gregorian::days(59))) {
    return 0;
  }

  if (start_date <= tile_header_date && tile_header_date <= end_date) {
    start_date = tile_header_date;
  } else if (tile_header_date > end_date) { // reject.
    return 0;
  }

  // only support 60 days out.  (59 days and include the end_date = 60)
  boost::gregorian::date enddate = tile_header_date + boost::gregorian::days(59);

  if (enddate <= end_date) {
    end_date = enddate;
  }

  uint32_t days = 0;
  boost::gregorian::day_iterator itr(tile_header_date + boost::gregorian::days(days));

  uint32_t x = days;
  uint64_t bit_set = 0;

  // TODO: we dont have to loop over all days, we could take the week mask, shift it by the start
  // date then use that weekly mask and shift it how ever many times a week fits into the 60 day
  // interval loop over all the days
  while (itr <= end_date) {

    // figure out what day of the week we are at
    uint8_t dow;
    switch ((*itr).day_of_week().as_enum()) {
      case boost::date_time::Sunday:
        dow = baldr::kSunday;
        break;
      case boost::date_time::Monday:
        dow = baldr::kMonday;
        break;
      case boost::date_time::Tuesday:
        dow = baldr::kTuesday;
        break;
      case boost::date_time::Wednesday:
        dow = baldr::kWednesday;
        break;
      case boost::date_time::Thursday:
        dow = baldr::kThursday;
        break;
      case boost::date_time::Friday:
        dow = baldr::kFriday;
        break;
      case boost::date_time::Saturday:
        dow = baldr::kSaturday;
        break;
    }

    // were we supposed to be on for this day?
    // and are we at or after the start date?
    if ((dow_mask & dow) && (itr >= start_date)) {
      bit_set |= static_cast<uint64_t>(1) << x;
    }

    ++itr;
    ++x;
  }
  return bit_set;
}

/**
 * Adds a service day to the days.
 * @param   days supported by the gtfs feed/service
 * @param   end_date end date
 * @param   tile_date seconds from epoch
 * @param   added_date in the format of 20150516 or 2015-05-06T08:00
 * @return  Returns the updated days.  Days will only be updated if the added date
 *          is in the start and end date range.
 */
static uint64_t add_service_day(const uint64_t& days,
                                const boost::gregorian::date& end_date,
                                const uint32_t tile_date,
                                const boost::gregorian::date& added_date) {

  // adding a service day must start at the tile header date.
  boost::gregorian::date start_date = pivot_date_ + boost::gregorian::days(tile_date);
  boost::gregorian::date enddate = start_date + boost::gregorian::days(59);

  if (enddate > end_date) {
    enddate = end_date;
  }

  if (start_date <= added_date && added_date <= enddate) {
    boost::gregorian::date_period range(start_date, added_date);
    uint32_t length = range.length().days();
    return days | (static_cast<uint64_t>(1) << length);
  }
  return days;
}

/**
 * Removes a service day to the days.
 * @param   days supported by the gtfs feed/service
 * @param   end_date end date
 * @param   tile_date seconds from epoch
 * @param   removed_date in the format of 20150516 or 2015-05-06T08:00
 * @return  Returns the updated days.  Days will only be updated if the removed date
 *          is in the start and end date range.
 */
static uint64_t remove_service_day(const uint64_t& days,
                                   const boost::gregorian::date& end_date,
                                   const uint32_t tile_date,
                                   const boost::gregorian::date& removed_date) {
  // removing a service day must start at the tile header date.
  boost::gregorian::date start_date = pivot_date_ + boost::gregorian::days(tile_date);
  boost::gregorian::date enddate = start_date + boost::gregorian::days(59);

  if (enddate > end_date) {
    enddate = end_date;
  }

  if (start_date <= removed_date && removed_date <= enddate) {
    boost::gregorian::date_period range(start_date, removed_date);
    uint32_t length = range.length().days();
    return ~((~days) | (static_cast<uint64_t>(1) << length));
  }
  return days;
}

} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_SERVICEDAYS_H
