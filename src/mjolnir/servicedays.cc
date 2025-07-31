#include "mjolnir/servicedays.h"
#include "baldr/datetime.h"

using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace valhalla {
namespace mjolnir {

std::string to_iso_extended_string(const date::sys_days& d) {
  std::ostringstream iso_date_time;
  iso_date_time << date::format("%F", d);
  return iso_date_time.str();
}

// Get a formatted testing date.  Currently, next Tuesday @ 08:00.
std::string get_testing_date_time() {
  auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
  auto date = DateTime::iso_date_time(tz);
  std::istringstream in{date};

  // needs to be local_days
  date::local_days tp;
  if (date.find('T') != std::string::npos)
    in >> date::parse("%FT%R", tp);
  else if (date.find('-') != std::string::npos)
    in >> date::parse("%F", tp);

  while (date::weekday(tp) != date::Tuesday) {
    tp += date::days(1);
  }

  std::ostringstream iso_date_time;
  const auto d = date::make_zoned(tz, tp, date::choose::latest);
  iso_date_time << date::format("%FT%R", d);
  return iso_date_time.str();
}

// Get the days that this transit service is running in 60 days or less
uint64_t get_service_days(date::sys_days& start_date,
                          date::sys_days& end_date,
                          const uint32_t& tile_date,
                          const uint32_t& dow_mask) {

  auto d = date::floor<date::days>(DateTime::pivot_date_);
  date::sys_days tile_header_date = date::sys_days(date::year_month_day(d + date::days(tile_date)));

  // if our start date is more than 60 days out, reject.
  if (start_date > (tile_header_date + date::days(59))) {
    return 0;
  }

  if (start_date <= tile_header_date && tile_header_date <= end_date) {
    start_date = tile_header_date;
  } else if (tile_header_date > end_date) { // reject.
    return 0;
  }

  // only support 60 days out.  (59 days and include the end_date = 60)
  date::sys_days enddate = tile_header_date + date::days(59);

  if (enddate <= end_date) {
    end_date = enddate;
  }

  date::sys_days itr = tile_header_date;

  uint32_t x = 0;
  uint64_t bit_set = 0;

  // TODO: we dont have to loop over all days, we could take the week mask, shift it by the start
  // date then use that weekly mask and shift it how ever many times a week fits into the 60 day
  // interval loop over all the days
  while (itr <= end_date) {

    // figure out what day of the week we are at
    uint8_t dow;
    uint8_t wd = (date::weekday(itr) - date::Sunday).count();

    switch (wd) {
      case 0:
        dow = kSunday;
        break;
      case 1:
        dow = kMonday;
        break;
      case 2:
        dow = kTuesday;
        break;
      case 3:
        dow = kWednesday;
        break;
      case 4:
        dow = kThursday;
        break;
      case 5:
        dow = kFriday;
        break;
      case 6:
        dow = kSaturday;
        break;
    }

    // were we supposed to be on for this day?
    // and are we at or after the start date?
    if ((dow_mask & dow) && (itr >= start_date)) {
      bit_set |= static_cast<uint64_t>(1) << x;
    }

    itr += date::days(1);
    ++x;
  }
  return bit_set;
}

// Adds a service day to the days.
uint64_t add_service_day(const uint64_t& days,
                         const date::sys_days& end_date,
                         const uint32_t& tile_date,
                         const date::sys_days& added_date) {
  // adding a service day must start at the tile header date.
  auto d = date::floor<date::days>(DateTime::pivot_date_);
  date::sys_days start_date = date::sys_days(date::year_month_day(d + date::days(tile_date)));
  date::sys_days enddate = start_date + date::days(59);

  if (enddate > end_date) {
    enddate = end_date;
  }

  if (start_date <= added_date && added_date <= enddate) {
    uint32_t length = static_cast<uint32_t>((added_date - start_date).count());
    return days | (static_cast<uint64_t>(1) << length);
  }
  return days;
}

// Removes a service day to the days.
uint64_t remove_service_day(const uint64_t& days,
                            const date::sys_days& end_date,
                            const uint32_t& tile_date,
                            const date::sys_days& removed_date) {
  // removing a service day must start at the tile header date.
  auto d = date::floor<date::days>(DateTime::pivot_date_);
  date::sys_days start_date = date::sys_days(date::year_month_day(d + date::days(tile_date)));
  date::sys_days enddate = start_date + date::days(59);

  if (enddate > end_date) {
    enddate = end_date;
  }

  if (start_date <= removed_date && removed_date <= enddate) {
    uint32_t length = static_cast<uint32_t>((removed_date - start_date).count());
    return days & ~(static_cast<uint64_t>(1) << length);
  }
  return days;
}

// Use when shifting all service and exception due to past midnight result
uint64_t shift_service_day(const uint64_t& days) {
  auto new_days = days << 1;
  // remove the day after the last day
  return ~((~new_days) | (static_cast<uint64_t>(1) << 60));
}

} // namespace mjolnir
} // namespace valhalla
