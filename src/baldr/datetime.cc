#include <bitset>
#include <fstream>
#include <iostream>
#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/range/algorithm/remove_if.hpp>

#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
//#include "baldr/reutil.h"
#include "baldr/timedomain.h"
#include "midgard/logging.h"

#include "date_time_zonespec.h"

using namespace valhalla::baldr;

namespace {

const boost::gregorian::date pivot_date_ = boost::gregorian::from_undelimited_string(kPivotDate);
}

namespace valhalla {
namespace baldr {
namespace DateTime {

tz_db_t::tz_db_t() {
  // load up the tz data
  std::string tz_data(date_time_zonespec_csv, date_time_zonespec_csv + date_time_zonespec_csv_len);
  std::stringstream ss(tz_data);
  load_from_stream(ss);
  // unfortunately boosts object has its map marked as private... so we have to keep our own
  regions = region_list();
}

size_t tz_db_t::to_index(const std::string& region) const {
  auto it = std::find(regions.cbegin(), regions.cend(), region);
  if (it == regions.cend()) {
    return 0;
  }
  return (it - regions.cbegin()) + 1;
}

boost::shared_ptr<boost::local_time::tz_database::time_zone_base_type>
tz_db_t::from_index(size_t index) const {
  if (index < 1 || index > regions.size()) {
    return {};
  };
  return time_zone_from_region(regions[index - 1]);
}

const tz_db_t& get_tz_db() {
  // thread safe static initialization of global singleton
  static const tz_db_t tz_db;
  return tz_db;
}

// get a formatted date.
boost::gregorian::date get_formatted_date(const std::string& date) {
  boost::gregorian::date d;
  if (date.find('T') != std::string::npos) {
    std::string dt = date;
    dt.erase(boost::remove_if(dt, boost::is_any_of("-,:")), dt.end());
    d = boost::gregorian::date_from_iso_string(dt);
  } else if (date.find('-') != std::string::npos) {
    std::string dt = date;
    dt.erase(boost::remove_if(dt, boost::is_any_of("-")), dt.end());
    d = boost::gregorian::from_undelimited_string(dt);
  } else {
    d = boost::gregorian::from_undelimited_string(date);
  }
  return d;
}

// get a local_date_time with support for dst.
// 2016-11-06T02:00 ---> 2016-11-06T01:00
boost::local_time::local_date_time get_ldt(const boost::gregorian::date& date,
                                           const boost::posix_time::time_duration& time_duration,
                                           const boost::local_time::time_zone_ptr& time_zone) {

  boost::posix_time::time_duration td = time_duration;
  boost::local_time::local_date_time
      in_local_time(date, td, time_zone, boost::local_time::local_date_time::NOT_DATE_TIME_ON_ERROR);

  // create not-a-date-time if invalid (eg: in dst transition)
  if (in_local_time.is_not_a_date_time()) {

    if (time_zone->dst_local_start_time(date.year()).date() == date) {
      td += time_zone->dst_offset(); // clocks ahead.
      in_local_time = boost::local_time::
          local_date_time(date, td, time_zone,
                          boost::local_time::local_date_time::NOT_DATE_TIME_ON_ERROR);
    } else {
      // Daylight Savings Results are ambiguous: time given: 2016-Nov-06 01:00:00
      boost::posix_time::time_duration time_dur = time_zone->dst_offset();

      in_local_time = boost::local_time::
          local_date_time(date, td + time_dur, time_zone,
                          boost::local_time::local_date_time::NOT_DATE_TIME_ON_ERROR);
      in_local_time -= time_dur;
    }
  }
  return in_local_time;
}

// Get the number of days that have elapsed from the pivot date for the inputed date.
// date_time is in the format of 20150516 or 2015-05-06T08:00
uint32_t days_from_pivot_date(const boost::gregorian::date& date_time) {
  if (date_time <= pivot_date_) {
    return 0;
  }
  boost::gregorian::date_period range(pivot_date_, date_time);
  return static_cast<uint32_t>(range.length().days());
}

// Get the current iso date and time.
std::string iso_date_time(const boost::local_time::time_zone_ptr& time_zone) {
  std::string iso_date_time;
  if (!time_zone) {
    return iso_date_time;
  }

  try {
    boost::posix_time::ptime pt = boost::posix_time::second_clock::universal_time();
    boost::local_time::local_date_time local_date_time(pt, time_zone);

    pt = local_date_time.local_time();
    boost::gregorian::date date = pt.date();

    std::stringstream ss_time;
    ss_time << pt.time_of_day();
    std::string time = ss_time.str();

    std::size_t found = time.find_last_of(':'); // remove seconds.
    if (found != std::string::npos) {
      time = time.substr(0, found);
    }

    iso_date_time = to_iso_extended_string(date) + "T" + time;
  } catch (std::exception& e) {}
  return iso_date_time;
}

// Get the seconds since epoch based on timezone.
uint64_t seconds_since_epoch(const boost::local_time::time_zone_ptr& time_zone) {

  if (!time_zone) {
    return 0;
  }

  try {
    boost::posix_time::ptime pt = boost::posix_time::second_clock::universal_time();
    boost::local_time::local_date_time local_date_time(pt, time_zone);

    const boost::posix_time::ptime time_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::time_duration diff = local_date_time.utc_time() - time_epoch;

    return diff.total_seconds();

  } catch (std::exception& e) {}
  return 0;
}

// Get the seconds since epoch time is already adjusted based on TZ
uint64_t seconds_since_epoch(const std::string& date_time,
                             const boost::local_time::time_zone_ptr& time_zone) {
  if (date_time.empty()) {
    return 0;
  }

  try {
    boost::gregorian::date date;
    boost::posix_time::time_duration td;

    std::size_t found = date_time.find('T'); // YYYY-MM-DDTHH:MM
    if (found != std::string::npos) {
      std::string dt = date_time;
      dt.erase(boost::remove_if(dt, boost::is_any_of("-,:")), dt.end());
      date = boost::gregorian::date_from_iso_string(dt);
      td = boost::posix_time::duration_from_string(date_time.substr(found + 1));
    } else if (date_time.find('-') != std::string::npos) { // YYYY-MM-DD
      std::string dt = date_time;
      dt.erase(boost::remove_if(dt, boost::is_any_of("-")), dt.end());
      date = boost::gregorian::date_from_iso_string(dt);
      td = boost::posix_time::duration_from_string("0000");

    } else { // YYYYMMDD
      // No time on date.  Make it midnight.
      date = boost::gregorian::date_from_iso_string(date_time);
      td = boost::posix_time::duration_from_string("0000");
    }

    boost::local_time::local_date_time in_local_time = get_ldt(date, td, time_zone);

    boost::local_time::time_zone_ptr tz_utc(new boost::local_time::posix_time_zone("UTC"));
    boost::local_time::local_date_time local_date_time = in_local_time.local_time_in(tz_utc);
    boost::posix_time::ptime const time_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::time_duration diff = local_date_time.utc_time() - time_epoch;

    return diff.total_seconds();

  } catch (std::exception& e) {}
  return 0;
}

// Get the difference between two timezone using the seconds from epoch
// (taking into account the timezones and dst) and add the difference to the seconds
void timezone_diff(const bool is_depart_at,
                   uint64_t& seconds,
                   const boost::local_time::time_zone_ptr& origin_tz,
                   const boost::local_time::time_zone_ptr& dest_tz) {

  if ((origin_tz == dest_tz) || (seconds == 0) || (!origin_tz || !dest_tz)) {
    return;
  }

  try {
    std::string tz_string;
    const boost::posix_time::ptime time_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::ptime origin_pt = time_epoch + boost::posix_time::seconds(seconds);
    boost::local_time::local_date_time origin_date_time(origin_pt, origin_tz);

    boost::posix_time::ptime dest_pt = time_epoch + boost::posix_time::seconds(seconds);
    boost::local_time::local_date_time dest_date_time(dest_pt, dest_tz);

    boost::gregorian::date o_date = origin_date_time.local_time().date();
    boost::gregorian::date d_date = dest_date_time.local_time().date();

    if (is_depart_at && dest_date_time.is_dst()) {
      boost::gregorian::date dst_date = dest_tz->dst_local_end_time(d_date.year()).date();
      bool in_range = (o_date <= dst_date && dst_date <= d_date);

      if (in_range) { // in range meaning via the dates.
        if (o_date == dst_date) {
          // must start before dst end time - the offset otherwise the time is ambiguous
          in_range =
              origin_date_time.local_time().time_of_day() <
              (dest_tz->dst_local_end_time(d_date.year()).time_of_day() - dest_tz->dst_offset());

          if (in_range) {
            // starts and ends on the same day.
            if (o_date == d_date) {
              in_range = dest_tz->dst_local_end_time(d_date.year()).time_of_day() <=
                         dest_date_time.local_time().time_of_day();
            }
          }
        } else if (dst_date == d_date) {
          in_range = dest_tz->dst_local_end_time(d_date.year()).time_of_day() <=
                     dest_date_time.local_time().time_of_day();
        }
      }
      if (in_range) {
        dest_date_time -= dest_tz->dst_offset();
      }
    }

    if (!is_depart_at) {
      boost::gregorian::date dst_date = origin_tz->dst_local_end_time(o_date.year()).date();
      bool in_range = (o_date <= dst_date && dst_date <= d_date);

      if (in_range) { // in range meaning via the dates.
        if (o_date == dst_date) {
          // must start before dst end time
          in_range = origin_date_time.local_time().time_of_day() <=
                     (origin_tz->dst_local_end_time(o_date.year()).time_of_day());

          if (in_range) {
            // starts and ends on the same day.
            if (o_date == d_date) {
              in_range = origin_tz->dst_local_end_time(o_date.year()).time_of_day() >
                         dest_date_time.local_time().time_of_day();
            }
          }
        } else if (dst_date == d_date) {
          in_range = origin_tz->dst_local_end_time(o_date.year()).time_of_day() >
                     dest_date_time.local_time().time_of_day();
        }
      }

      if (in_range) {
        origin_date_time -= origin_tz->dst_offset();
      }
    }

    origin_pt = origin_date_time.local_time();
    dest_pt = dest_date_time.local_time();

    boost::posix_time::time_duration td = origin_pt - dest_pt;
    if (origin_tz->base_utc_offset() < dest_tz->base_utc_offset()) {
      seconds += abs(td.total_seconds());
    } else {
      // should never happen
      if (seconds - abs(td.total_seconds()) < 0) {
        return;
      }
      seconds -= abs(td.total_seconds());
    }
  } catch (std::exception& e) {}
}

std::string seconds_to_date(const uint64_t seconds, const boost::local_time::time_zone_ptr& tz) {

  std::string iso_date;
  if (seconds == 0 || !tz) {
    return iso_date;
  }

  try {
    std::string tz_string;
    const boost::posix_time::ptime time_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::ptime pt = time_epoch + boost::posix_time::seconds(seconds);
    boost::local_time::local_date_time date_time(pt, tz);
    pt = date_time.local_time();

    boost::gregorian::date date = pt.date();
    std::stringstream ss_time;
    ss_time << pt.time_of_day();
    std::string time = ss_time.str();

    std::size_t found = time.find_last_of(':'); // remove seconds.
    if (found != std::string::npos) {
      time = time.substr(0, found);
    }

    ss_time.str("");
    if (date_time.is_dst()) {
      ss_time << tz->dst_offset() + tz->base_utc_offset();
    } else {
      ss_time << tz->base_utc_offset();
    }

    // positive tz
    if (ss_time.str().find('+') == std::string::npos &&
        ss_time.str().find('-') == std::string::npos) {
      iso_date = to_iso_extended_string(date) + "T" + time + "+" + ss_time.str();
    } else {
      iso_date = to_iso_extended_string(date) + "T" + time + ss_time.str();
    }

    found = iso_date.find_last_of(':'); // remove seconds.
    if (found != std::string::npos) {
      iso_date = iso_date.substr(0, found);
    }

  } catch (std::exception& e) {}
  return iso_date;
}

// Get the date from seconds and timezone.
void seconds_to_date(const bool is_depart_at,
                     const uint64_t origin_seconds,
                     const uint64_t dest_seconds,
                     const boost::local_time::time_zone_ptr& origin_tz,
                     const boost::local_time::time_zone_ptr& dest_tz,
                     std::string& iso_origin,
                     std::string& iso_dest) {

  iso_origin = "";
  iso_dest = "";

  if (!origin_tz || !dest_tz) {
    return;
  }

  try {
    std::string tz_string;
    const boost::posix_time::ptime time_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::ptime origin_pt = time_epoch + boost::posix_time::seconds(origin_seconds);
    boost::local_time::local_date_time origin_date_time(origin_pt, origin_tz);

    boost::posix_time::ptime dest_pt = time_epoch + boost::posix_time::seconds(dest_seconds);
    boost::local_time::local_date_time dest_date_time(dest_pt, dest_tz);

    boost::gregorian::date o_date = origin_date_time.local_time().date();
    boost::gregorian::date d_date = dest_date_time.local_time().date();

    if (is_depart_at && dest_date_time.is_dst()) {
      boost::gregorian::date dst_date = dest_tz->dst_local_end_time(d_date.year()).date();
      bool in_range = (o_date <= dst_date && dst_date <= d_date);

      if (in_range) { // in range meaning via the dates.
        if (o_date == dst_date) {
          // must start before dst end time - the offset otherwise the time is ambiguous
          in_range =
              origin_date_time.local_time().time_of_day() <
              (dest_tz->dst_local_end_time(d_date.year()).time_of_day() - dest_tz->dst_offset());

          if (in_range) {
            // starts and ends on the same day.
            if (o_date == d_date) {
              in_range = dest_tz->dst_local_end_time(d_date.year()).time_of_day() <=
                         dest_date_time.local_time().time_of_day();
            }
          }
        } else if (dst_date == d_date) {
          in_range = dest_tz->dst_local_end_time(d_date.year()).time_of_day() <=
                     dest_date_time.local_time().time_of_day();
        }
      }
      if (in_range) {
        dest_date_time -= dest_tz->dst_offset();
      }
    }

    if (!is_depart_at) {
      boost::gregorian::date dst_date = origin_tz->dst_local_end_time(o_date.year()).date();
      bool in_range = (o_date <= dst_date && dst_date <= d_date);

      if (in_range) { // in range meaning via the dates.
        if (o_date == dst_date) {
          // must start before dst end time
          in_range = origin_date_time.local_time().time_of_day() <=
                     (origin_tz->dst_local_end_time(o_date.year()).time_of_day());

          if (in_range) {
            // starts and ends on the same day.
            if (o_date == d_date) {
              in_range = origin_tz->dst_local_end_time(o_date.year()).time_of_day() >
                         dest_date_time.local_time().time_of_day();
            }
          }
        } else if (dst_date == d_date) {
          in_range = origin_tz->dst_local_end_time(o_date.year()).time_of_day() >
                     dest_date_time.local_time().time_of_day();
        }
      }

      if (in_range) {
        origin_date_time -= origin_tz->dst_offset();
      }
    }

    origin_pt = origin_date_time.local_time();
    boost::gregorian::date date = origin_pt.date();
    std::stringstream ss_time;
    ss_time << origin_pt.time_of_day();
    std::string time = ss_time.str();

    std::size_t found = time.find_last_of(':'); // remove seconds.
    if (found != std::string::npos) {
      time = time.substr(0, found);
    }

    ss_time.str("");
    if (origin_date_time.is_dst()) {
      ss_time << origin_tz->dst_offset() + origin_tz->base_utc_offset();
    } else {
      ss_time << origin_tz->base_utc_offset();
    }

    // positive tz
    if (ss_time.str().find('+') == std::string::npos &&
        ss_time.str().find('-') == std::string::npos) {
      iso_origin = to_iso_extended_string(date) + "T" + time + "+" + ss_time.str();
    } else {
      iso_origin = to_iso_extended_string(date) + "T" + time + ss_time.str();
    }

    found = iso_origin.find_last_of(':'); // remove seconds.
    if (found != std::string::npos) {
      iso_origin = iso_origin.substr(0, found);
    }

    dest_pt = dest_date_time.local_time();
    date = dest_pt.date();
    ss_time.str("");
    ss_time << dest_pt.time_of_day();
    time = ss_time.str();

    found = time.find_last_of(':'); // remove seconds.
    if (found != std::string::npos) {
      time = time.substr(0, found);
    }

    ss_time.str("");
    if (dest_date_time.is_dst()) {
      ss_time << dest_tz->dst_offset() + dest_tz->base_utc_offset();
    } else {
      ss_time << dest_tz->base_utc_offset();
    }

    // positive tz
    if (ss_time.str().find('+') == std::string::npos &&
        ss_time.str().find('-') == std::string::npos) {
      iso_dest = to_iso_extended_string(date) + "T" + time + "+" + ss_time.str();
    } else {
      iso_dest = to_iso_extended_string(date) + "T" + time + ss_time.str();
    }

    found = iso_dest.find_last_of(':'); // remove seconds.
    if (found != std::string::npos) {
      iso_dest = iso_dest.substr(0, found);
    }

  } catch (std::exception& e) {}
}

// Get the dow mask
// date_time is in the format of 20150516 or 2015-05-06T08:00
uint32_t day_of_week_mask(const std::string& date_time) {
  boost::gregorian::date date;
  date = get_formatted_date(date_time);

  if (date < pivot_date_) {
    return kDOWNone;
  }

  boost::gregorian::greg_weekday wd = date.day_of_week();

  switch (wd.as_enum()) {
    case boost::date_time::Sunday:
      return kSunday;
      break;
    case boost::date_time::Monday:
      return kMonday;
      break;
    case boost::date_time::Tuesday:
      return kTuesday;
      break;
    case boost::date_time::Wednesday:
      return kWednesday;
      break;
    case boost::date_time::Thursday:
      return kThursday;
      break;
    case boost::date_time::Friday:
      return kFriday;
      break;
    case boost::date_time::Saturday:
      return kSaturday;
      break;
  }
  return kDOWNone;
}

// Get the number of seconds midnight that have elapsed.
uint32_t seconds_from_midnight(const std::string& date_time) {
  // date_time is in the format of HH:MM:SS or HH:MM or YYYY-MM-DDTHH:MM
  // hours can be greater than 24.
  // please see GTFS spec:
  // https://developers.google.com/transit/gtfs/reference#stop_times_fields

  boost::posix_time::time_duration td;
  std::size_t found = date_time.find('T'); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos) {
    td = boost::posix_time::duration_from_string(date_time.substr(found + 1));
  } else {
    td = boost::posix_time::duration_from_string(date_time);
  }

  return static_cast<uint32_t>(td.total_seconds());
}

// add x seconds to a date_time and return a ISO date_time string.
// date_time is in the format of 20150516 or 2015-05-06T08:00
std::string get_duration(const std::string& date_time,
                         const uint32_t seconds,
                         const boost::local_time::time_zone_ptr& tz) {
  std::string formatted_date_time;
  boost::posix_time::ptime start;
  boost::gregorian::date date;
  if (date_time.find('T') != std::string::npos) {
    std::string dt = date_time;
    dt.erase(boost::remove_if(dt, boost::is_any_of("-,:")), dt.end());
    start = boost::posix_time::from_iso_string(dt);
    date = boost::gregorian::date_from_iso_string(dt);
  } else if (date_time.find('-') != std::string::npos) {
    std::string dt = date_time;
    dt.erase(boost::remove_if(dt, boost::is_any_of("-")), dt.end());
    start = boost::posix_time::from_iso_string(dt + "T0000");
    date = boost::gregorian::from_undelimited_string(dt);
  } else {
    // No time on date.  Make it midnight.
    start = boost::posix_time::from_iso_string(date_time + "T0000");
    date = boost::gregorian::from_undelimited_string(date_time);
  }

  if (date < pivot_date_) {
    return formatted_date_time;
  }

  boost::posix_time::ptime end = start + boost::posix_time::seconds(seconds);
  formatted_date_time = boost::posix_time::to_iso_extended_string(end);

  boost::local_time::local_date_time dt(end, tz);

  std::size_t found = formatted_date_time.find_last_of(':'); // remove seconds.
  if (found != std::string::npos) {
    formatted_date_time = formatted_date_time.substr(0, found);
  }

  std::stringstream ss;
  std::string tz_abbrev;
  if (dt.is_dst()) {
    ss << tz->dst_offset() + tz->base_utc_offset();
    // positive tz
    if (ss.str().find('+') == std::string::npos && ss.str().find('-') == std::string::npos) {
      ss.str("");
      ss << "+" << tz->dst_offset() + tz->base_utc_offset();
    }
    tz_abbrev = tz->dst_zone_abbrev();
  } else {
    ss << tz->base_utc_offset();
    // positive tz
    if (ss.str().find('+') == std::string::npos && ss.str().find('-') == std::string::npos) {
      ss.str("");
      ss << "+" << tz->base_utc_offset();
    }
    tz_abbrev = tz->std_zone_abbrev();
  }

  formatted_date_time += ss.str();

  found = formatted_date_time.find_last_of(':'); // remove seconds.
  if (found != std::string::npos) {
    formatted_date_time = formatted_date_time.substr(0, found);
  }

  formatted_date_time += " " + tz_abbrev;

  return formatted_date_time;
}

// does this date fall in the begin and end date range?
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
                   const boost::local_time::time_zone_ptr& time_zone) {
  bool dow_in_range = true;
  bool dt_in_range = false;

  try {
    boost::gregorian::date begin_date, end_date;
    boost::posix_time::time_duration b_td = boost::posix_time::hours(0),
                                     e_td = boost::posix_time::hours(23) +
                                            boost::posix_time::minutes(59);

    const boost::posix_time::ptime time_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::ptime origin_pt = time_epoch + boost::posix_time::seconds(current_time);
    boost::local_time::local_date_time in_local_time(origin_pt, time_zone);
    boost::gregorian::date d = in_local_time.date();
    boost::posix_time::time_duration td = in_local_time.local_time().time_of_day();

    // we have dow
    if (dow) {

      uint8_t local_dow = 0;
      switch (d.day_of_week()) {
        case boost::date_time::Sunday:
          local_dow = kSunday;
          break;
        case boost::date_time::Monday:
          local_dow = kMonday;
          break;
        case boost::date_time::Tuesday:
          local_dow = kTuesday;
          break;
        case boost::date_time::Wednesday:
          local_dow = kWednesday;
          break;
        case boost::date_time::Thursday:
          local_dow = kThursday;
          break;
        case boost::date_time::Friday:
          local_dow = kFriday;
          break;
        case boost::date_time::Saturday:
          local_dow = kSaturday;
          break;
        default:
          return false; // should never happen
          break;
      }
      dow_in_range = (dow & local_dow);
    }

    uint8_t b_month = begin_month;
    uint8_t e_month = end_month;
    uint8_t b_day_dow = begin_day_dow;
    uint8_t e_day_dow = end_day_dow;
    uint8_t b_week = begin_week;
    uint8_t e_week = end_week;

    if (type == kNthDow && begin_week && !begin_day_dow && !begin_month) { // Su[-1]
      b_month = d.month().as_enum();
    }
    if (type == kNthDow && end_week && !end_day_dow && !end_month) { // Su[-1]
      e_month = d.month().as_enum();
    }

    if (type == kNthDow && begin_week && !begin_day_dow && !begin_month && !end_week &&
        !end_day_dow && !end_month) { // only Su[-1] set in begin.
      // First Sunday of every month only.
      e_month = b_month;
      b_day_dow = e_day_dow = dow;
      e_week = b_week;
    } else if (type == kYMD && (b_month && e_month) &&
               (!b_day_dow && !e_day_dow)) { // Sep-Jun We 08:15-08:45

      b_day_dow = 1;
      boost::gregorian::date e_d = boost::gregorian::date(d.year(), e_month, 1);
      e_day_dow = e_d.end_of_month().day();
    }

    // month only
    if (type == kYMD && (b_month && e_month) && (!b_day_dow && !e_day_dow && !b_week && !b_week) &&
        b_month == e_month) {

      dt_in_range = (b_month <= d.month().as_enum() && d.month().as_enum() <= e_month);

      if (begin_hrs || begin_mins || end_hrs || end_mins) {
        b_td = boost::posix_time::hours(begin_hrs) + boost::posix_time::minutes(begin_mins);
        e_td = boost::posix_time::hours(end_hrs) + boost::posix_time::minutes(end_mins);
      }

      dt_in_range = (dt_in_range && (b_td <= td && td <= e_td));
      return (dow_in_range && dt_in_range);

    } else if (type == kYMD && b_month && b_day_dow) {

      uint32_t e_year = d.year(), b_year = d.year();
      if (b_month == e_month) {
        if (b_day_dow > e_day_dow) { // Mar 15 - Mar 1
          e_year = d.year() + 1;
        }
      } else if (b_month > e_month) { // Oct 10 - Mar 3
        if (b_month > d.month().as_enum()) {
          b_year = d.year() - 1;
        } else {
          e_year = d.year() + 1;
        }
      }

      begin_date = boost::gregorian::date(b_year, b_month, b_day_dow);
      end_date = boost::gregorian::date(e_year, e_month, e_day_dow);

    } else if (type == kNthDow && b_month && b_day_dow && e_month &&
               e_day_dow) { // kNthDow types can have a mix of ymd and nthdow. (e.g. Dec Su[-1]-Mar
                            // 3 Sat 15:00-17:00)

      uint32_t e_year = d.year(), b_year = d.year();
      if (b_month == e_month) {
        if (b_day_dow > e_day_dow) { // Mar 15 - Mar 1
          e_year = d.year() + 1;
        }
      } else if (b_month > e_month) { // Oct 10 - Mar 3
        if (b_month > d.month().as_enum()) {
          b_year = d.year() - 1;
        } else {
          e_year = d.year() + 1;
        }
      }

      if (b_week && b_week <= 5) { // kNthDow
        boost::gregorian::nth_day_of_the_week_in_month
            nthdow(static_cast<boost::gregorian::nth_day_of_the_week_in_month::week_num>(b_week),
                   b_day_dow - 1, b_month);
        begin_date = nthdow.get_date(b_year);
      } else { // YMD
        begin_date = boost::gregorian::date(b_year, b_month, b_day_dow);
      }

      if (e_week && e_week <= 5) { // kNthDow
        boost::gregorian::nth_day_of_the_week_in_month
            nthdow(static_cast<boost::gregorian::nth_day_of_the_week_in_month::week_num>(e_week),
                   e_day_dow - 1, e_month);
        end_date = nthdow.get_date(e_year);
      } else {                                                         // YMD
        end_date = boost::gregorian::date(e_year, e_month, e_day_dow); // Dec 5 to Mar 3
      }
    } else { // do we have just time?

      if (begin_hrs || begin_mins || end_hrs || end_mins) {
        b_td = boost::posix_time::hours(begin_hrs) + boost::posix_time::minutes(begin_mins);
        e_td = boost::posix_time::hours(end_hrs) + boost::posix_time::minutes(end_mins);

        if (begin_hrs > end_hrs) { // 19:00 - 06:00
          dt_in_range = !(e_td <= td && td <= b_td);
        } else {
          dt_in_range = (b_td <= td && td <= e_td);
        }
      }
      return (dow_in_range && dt_in_range);
    }

    if (begin_hrs || begin_mins || end_hrs || end_mins) {
      b_td = boost::posix_time::hours(begin_hrs) + boost::posix_time::minutes(begin_mins);
      e_td = boost::posix_time::hours(end_hrs) + boost::posix_time::minutes(end_mins);
    }

    boost::local_time::local_date_time b_in_local_time = get_ldt(begin_date, b_td, time_zone);
    boost::local_time::local_date_time e_in_local_time = get_ldt(end_date, e_td, time_zone);

    dt_in_range = (b_in_local_time.date() <= in_local_time.date() &&
                   in_local_time.date() <= e_in_local_time.date());

    bool time_in_range = false;

    if (begin_hrs > end_hrs) { // 19:00 - 06:00
      time_in_range = !(e_td <= td && td <= b_td);
    } else {
      time_in_range = (b_td <= td && td <= e_td);
    }

    dt_in_range = (dt_in_range && time_in_range);
  } catch (std::exception& e) {}
  return (dow_in_range && dt_in_range);
}

// get the dow mask from user inputed string.  try to handle most inputs
uint8_t get_dow_mask(const std::string& dow) {

  std::string str = dow;
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
  str.erase(boost::remove_if(str, boost::is_any_of(":")), str.end());

  if (str == "SUNDAY" || str == "SUN" || str == "SU") {
    return kSunday;

  } else if (str == "MONDAY" || str == "MON" || str == "MO") {
    return kMonday;

  } else if (str == "TUESDAY" || str == "TUES" || str == "TUE" || str == "TU") {
    return kTuesday;

  } else if (str == "WEDNESDAY" || str == "WEDS" || str == "WED" || str == "WE") {
    return kWednesday;

  } else if (str == "THURSDAY" || str == "THURS" || str == "THUR" || str == "TH") {
    return kThursday;

  } else if (str == "FRIDAY" || str == "FRI" || str == "FR") {
    return kFriday;

  } else if (str == "SATURDAY" || str == "SAT" || str == "SA") {
    return kSaturday;
  }
  return kDOWNone;
}

// get the dow from user inputed string.  try to handle most inputs
DOW get_dow(const std::string& dow) {

  std::string str = dow;
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
  str.erase(boost::remove_if(str, boost::is_any_of(":")), str.end());

  if (str == "SUNDAY" || str == "SUN" || str == "SU") {
    return DOW::kSunday;

  } else if (str == "MONDAY" || str == "MON" || str == "MO") {
    return DOW::kMonday;

  } else if (str == "TUESDAY" || str == "TUES" || str == "TUE" || str == "TU") {
    return DOW::kTuesday;

  } else if (str == "WEDNESDAY" || str == "WEDS" || str == "WED" || str == "WE") {
    return DOW::kWednesday;

  } else if (str == "THURSDAY" || str == "THURS" || str == "THUR" || str == "TH") {
    return DOW::kThursday;

  } else if (str == "FRIDAY" || str == "FRI" || str == "FR") {
    return DOW::kFriday;

  } else if (str == "SATURDAY" || str == "SAT" || str == "SA") {
    return DOW::kSaturday;
  }
  return DOW::kNone;
}

} // namespace DateTime
} // namespace baldr
} // namespace valhalla
