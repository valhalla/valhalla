#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "baldr/datetime.h"
#include "mjolnir/servicedays.h"

using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace valhalla {
namespace mjolnir {

// Get a formatted testing date.  Currently, next Tuesday @ 08:00.
std::string get_testing_date_time() {
  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
  boost::gregorian::date d = DateTime::get_formatted_date(DateTime::iso_date_time(tz));

  while (d.day_of_week() != boost::date_time::Tuesday) {
    d += boost::gregorian::days(1);
  }

  return to_iso_extended_string(d) + "T08:00";
}

// Get the days that this transit service is running in 60 days or less
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
        dow = kSunday;
        break;
      case boost::date_time::Monday:
        dow = kMonday;
        break;
      case boost::date_time::Tuesday:
        dow = kTuesday;
        break;
      case boost::date_time::Wednesday:
        dow = kWednesday;
        break;
      case boost::date_time::Thursday:
        dow = kThursday;
        break;
      case boost::date_time::Friday:
        dow = kFriday;
        break;
      case boost::date_time::Saturday:
        dow = kSaturday;
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

// Adds a service day to the days.
uint64_t add_service_day(const uint64_t& days,
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

// Removes a service day to the days.
uint64_t remove_service_day(const uint64_t& days,
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
