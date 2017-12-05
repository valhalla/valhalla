#include <iostream>
#include <sstream>
#include <bitset>
#include <fstream>
#include <regex>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>

#include "baldr/datetime.h"
#include "baldr/timedomain.h"
#include "baldr/graphconstants.h"

#include "date_time_zonespec.h"

using namespace valhalla::baldr;

namespace {

const boost::gregorian::date pivot_date_ = boost::gregorian::from_undelimited_string(kPivotDate);

}

namespace valhalla {
namespace baldr {
namespace DateTime {

tz_db_t::tz_db_t() {
  //load up the tz data
  std::string tz_data(date_time_zonespec_csv, date_time_zonespec_csv + date_time_zonespec_csv_len);
  std::stringstream ss(tz_data);
  load_from_stream(ss);
  //unfortunately boosts object has its map marked as private... so we have to keep our own
  regions = region_list();
}

size_t tz_db_t::to_index(const std::string& region) const {
  auto it = std::find(regions.cbegin(), regions.cend(), region);
  if(it == regions.cend())
    return 0;
  return (it - regions.cbegin()) + 1;
}

boost::shared_ptr<boost::local_time::tz_database::time_zone_base_type> tz_db_t::from_index(size_t index) const {
  if(index < 1 || index > regions.size())
    return {};
  return time_zone_from_region(regions[index - 1]);
}

const tz_db_t& get_tz_db() {
  //thread safe static initialization of global singleton
  static const tz_db_t tz_db;
  return tz_db;
}

//default testing date and time is the next Tues @ 08:00
std::string get_testing_date_time() {
  auto tz = get_tz_db().from_index(get_tz_db().to_index("America/New_York"));
  boost::gregorian::date d = get_formatted_date(iso_date_time(tz));

  while (d.day_of_week() != boost::date_time::Tuesday)
    d += boost::gregorian::days(1);

  return to_iso_extended_string(d) + "T08:00";
}

//get a formatted date.
boost::gregorian::date get_formatted_date(const std::string& date) {
  boost::gregorian::date d;
  if (date.find("T") != std::string::npos) {
    std::string dt = date;
    dt.erase(boost::remove_if(dt, boost::is_any_of("-,:")), dt.end());
    d = boost::gregorian::date_from_iso_string(dt);
  } else if (date.find("-") != std::string::npos) {
    std::string dt = date;
    dt.erase(boost::remove_if(dt, boost::is_any_of("-")), dt.end());
    d = boost::gregorian::from_undelimited_string(dt);
  } else
    d = boost::gregorian::from_undelimited_string(date);
  return d;
}

// get a local_date_time with support for dst.
// 2016-11-06T02:00 ---> 2016-11-06T01:00
boost::local_time::local_date_time get_ldt(const boost::gregorian::date& date,
                                           const boost::posix_time::time_duration& time_duration,
                                           const boost::local_time::time_zone_ptr& time_zone) {

  boost::posix_time::time_duration td = time_duration;
  boost::local_time::local_date_time in_local_time(date, td, time_zone,
                                                       boost::local_time::local_date_time::NOT_DATE_TIME_ON_ERROR);

    // create not-a-date-time if invalid (eg: in dst transition)
    if (in_local_time.is_not_a_date_time()) {

      if (time_zone->dst_local_start_time(date.year()).date() == date) {
        td += time_zone->dst_offset(); //clocks ahead.
        in_local_time = boost::local_time::local_date_time(date, td, time_zone,
                                                           boost::local_time::local_date_time::NOT_DATE_TIME_ON_ERROR);
      }
      else {
//Daylight Savings Results are ambiguous: time given: 2016-Nov-06 01:00:00
        boost::posix_time::time_duration time_dur = time_zone->dst_offset();

        in_local_time = boost::local_time::local_date_time(date, td + time_dur, time_zone,
                                                           boost::local_time::local_date_time::NOT_DATE_TIME_ON_ERROR);
        in_local_time -= time_dur;
      }
    }
    return in_local_time;
}

//Get service days
//Start from the tile_date or start date to end date or 60 days (whichever is less)
//start_date will be updated to the tile creation date if the start date is in the past
//set the bits based on the dow.
uint64_t get_service_days(boost::gregorian::date& start_date, boost::gregorian::date& end_date,
                          const uint32_t tile_date, const uint32_t dow_mask) {

  boost::gregorian::date tile_header_date = pivot_date_ + boost::gregorian::days(tile_date);

  // if our start date is more than 60 days out, reject.
  if (start_date > (tile_header_date + boost::gregorian::days(59)))
    return 0;

  if (start_date <= tile_header_date && tile_header_date <= end_date)
    start_date = tile_header_date;
  else if (tile_header_date > end_date) //reject.
    return 0;

  // only support 60 days out.  (59 days and include the end_date = 60)
  boost::gregorian::date enddate = tile_header_date + boost::gregorian::days(59);

  if (enddate <= end_date)
    end_date = enddate;

  uint32_t days = 0;
  boost::gregorian::day_iterator itr(tile_header_date + boost::gregorian::days(days));

  uint32_t x = days;
  uint64_t bit_set = 0;

  //TODO: we dont have to loop over all days, we could take the week mask, shift it by the start date
  //then use that weekly mask and shift it how ever many times a week fits into the 60 day interval
  //loop over all the days
  while (itr <= end_date) {

    //figure out what day of the week we are at
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

    //were we supposed to be on for this day?
    //and are we at or after the start date?
    if ((dow_mask & dow) && (itr >= start_date))
      bit_set |= static_cast<uint64_t>(1) << x;

    ++itr;
    ++x;
  }
  return bit_set;
}

//add a service day to the days if it is in range.
uint64_t add_service_day(const uint64_t& days,
                         const boost::gregorian::date& end_date,
                         const uint32_t tile_date,
                         const boost::gregorian::date& added_date) {

  // adding a service day must start at the tile header date.
  boost::gregorian::date start_date = pivot_date_ + boost::gregorian::days(tile_date);
  boost::gregorian::date enddate = start_date + boost::gregorian::days(59);

  if (enddate > end_date)
    enddate = end_date;

  if (start_date <= added_date && added_date <= enddate) {
    boost::gregorian::date_period range(start_date, added_date);
    uint32_t length = range.length().days();
    return days | (static_cast<uint64_t>(1) << length);
  }
  return days;
}

//remove a service day to the days if it is in range.
uint64_t remove_service_day(const uint64_t& days,
                            const boost::gregorian::date& end_date,
                            const uint32_t tile_date,
                            const boost::gregorian::date& removed_date) {

  // removing a service day must start at the tile header date.
  boost::gregorian::date start_date = pivot_date_ + boost::gregorian::days(tile_date);
  boost::gregorian::date enddate = start_date + boost::gregorian::days(59);

  if (enddate > end_date)
    enddate = end_date;

  if (start_date <= removed_date && removed_date <= enddate) {
    boost::gregorian::date_period range(start_date, removed_date);
    uint32_t length = range.length().days();
    return ~((~days) | (static_cast<uint64_t>(1) << length));
  }
  return days;
}

// check if service is available for a date.
bool is_service_available(const uint64_t days, const uint32_t start_date, const uint32_t date, const uint32_t end_date) {

  if (start_date <= date && date <= end_date) {
    boost::gregorian::date start = pivot_date_ + boost::gregorian::days(start_date);
    boost::gregorian::date d = pivot_date_ + boost::gregorian::days(date);

    std::bitset<64> bit_set(days);
    boost::gregorian::date_period range(start, d);
    uint32_t length = range.length().days();
    return bit_set.test(length);
  }
  return false;
}

//Get the number of days that have elapsed from the pivot date for the inputed date.
//date_time is in the format of 20150516 or 2015-05-06T08:00
uint32_t days_from_pivot_date(const boost::gregorian::date& date_time) {
  if (date_time <= pivot_date_)
    return 0;
  boost::gregorian::date_period range(pivot_date_, date_time);
  return static_cast<uint32_t>(range.length().days());
}

//Get the iso date and time from a DOW mask and time.
std::string iso_date_time(const uint8_t dow_mask, const std::string& time,
                          const boost::local_time::time_zone_ptr& time_zone) {


  std::string iso_date_time;
  std::stringstream ss("");
  if (time.empty() || time.find(":") == std::string::npos || !time_zone)
    return iso_date_time;

  uint8_t dow;
  switch (dow_mask) {
    case kSunday:
      dow = boost::date_time::Sunday;
      break;
    case kMonday:
      dow = boost::date_time::Monday;
      break;
    case kTuesday:
      dow = boost::date_time::Tuesday;
      break;
    case kWednesday:
      dow = boost::date_time::Wednesday;
      break;
    case kThursday:
      dow = boost::date_time::Thursday;
      break;
    case kFriday:
      dow = boost::date_time::Friday;
      break;
    case kSaturday:
      dow = boost::date_time::Saturday;
      break;
    default:
      return iso_date_time;
      break;
  }

  try {
    boost::local_time::local_time_input_facet* input_facet = new boost::local_time::local_time_input_facet();
    input_facet->format("%H:%M");
    ss.imbue(std::locale(ss.getloc(), input_facet));

    boost::local_time::local_date_time desired_time(boost::local_time::not_a_date_time);
    ss.str(time);
    ss >> desired_time;

    boost::posix_time::ptime pt = boost::posix_time::second_clock::universal_time();
    boost::local_time::local_date_time local_date_time(pt,time_zone);

    pt = local_date_time.local_time();
    boost::gregorian::date date = pt.date();
    uint8_t desired_tod = (3600 * desired_time.time_of_day().hours()) +
        (60 * desired_time.time_of_day().minutes());
    uint8_t current_tod = (3600 * pt.time_of_day().hours()) +
        (60 * pt.time_of_day().minutes());

    //will today work?
    if (date.day_of_week().as_enum() == dow) {
      //is the desired time in the past?
      if (desired_tod < current_tod)
        date += boost::gregorian::days(7);
    } else {
      while (date.day_of_week().as_enum() != dow)
        date += boost::gregorian::days(1);
    }
    iso_date_time = to_iso_extended_string(date) + "T" + time;
  } catch (std::exception& e){}
  return iso_date_time;
}

//Get the current iso date and time.
std::string iso_date_time(const boost::local_time::time_zone_ptr& time_zone) {
  std::string iso_date_time;
  if (!time_zone)
    return iso_date_time;

  try {
    boost::posix_time::ptime pt = boost::posix_time::second_clock::universal_time();
    boost::local_time::local_date_time local_date_time(pt,time_zone);

    pt = local_date_time.local_time();
    boost::gregorian::date date = pt.date();

    std::stringstream ss_time;
    ss_time << pt.time_of_day();
    std::string time = ss_time.str();

    std::size_t found = time.find_last_of(":"); // remove seconds.
    if (found != std::string::npos)
      time = time.substr(0,found);

    iso_date_time = to_iso_extended_string(date) + "T" + time;
  } catch (std::exception& e){}
  return iso_date_time;
}

// Get the seconds since epoch based on timezone.
uint64_t seconds_since_epoch(const boost::local_time::time_zone_ptr& time_zone) {

    if (!time_zone)
      return 0;

    try {
      boost::posix_time::ptime pt = boost::posix_time::second_clock::universal_time();
      boost::local_time::local_date_time local_date_time(pt,time_zone);

      boost::posix_time::ptime const time_epoch(boost::gregorian::date(1970, 1, 1));
      boost::posix_time::time_duration diff = local_date_time.utc_time() - time_epoch;

      return diff.total_seconds();

    } catch (std::exception& e){}
    return 0;
}

// Get the seconds since epoch time is already adjusted based on TZ
uint64_t seconds_since_epoch(const std::string& date_time,
                             const boost::local_time::time_zone_ptr& time_zone) {
  if (date_time.empty())
    return 0;

  try {
    boost::posix_time::ptime pt;
    boost::gregorian::date date;
    boost::posix_time::time_duration td;

    std::size_t found = date_time.find("T"); // YYYY-MM-DDTHH:MM
    if (found != std::string::npos) {
      std::string dt = date_time;
      dt.erase(boost::remove_if(dt, boost::is_any_of("-,:")), dt.end());
      pt = boost::posix_time::from_iso_string(dt);
      date = boost::gregorian::date_from_iso_string(dt);
      td = boost::posix_time::duration_from_string(date_time.substr(found+1));
    }
    else if (date_time.find("-") != std::string::npos) {
      std::string dt = date_time;
      dt.erase(boost::remove_if(dt, boost::is_any_of("-")), dt.end());
      pt = boost::posix_time::from_iso_string(dt + "T0000");
      date = boost::gregorian::date_from_iso_string(dt);
      td = boost::posix_time::duration_from_string("0000");

    } else {
      //No time on date.  Make it midnight.
      pt = boost::posix_time::from_iso_string(date_time + "T0000");
      date = boost::gregorian::date_from_iso_string("0000");

    }

    boost::local_time::local_date_time in_local_time = get_ldt(date,td,time_zone);

    boost::local_time::time_zone_ptr tz_utc(new boost::local_time::posix_time_zone("UTC"));
    boost::local_time::local_date_time local_date_time = in_local_time.local_time_in(tz_utc);
    boost::posix_time::ptime const time_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::time_duration diff = local_date_time.utc_time() - time_epoch;

    return diff.total_seconds();

  } catch (std::exception& e){}
  return 0;
}

// Get the date from seconds and timezone.
void seconds_to_date(const bool is_depart_at,
                     const uint64_t origin_seconds, const uint64_t dest_seconds,
                     const boost::local_time::time_zone_ptr& origin_tz,
                     const boost::local_time::time_zone_ptr& dest_tz,
                     std::string& iso_origin, std::string& iso_dest) {

  iso_origin = "";
  iso_dest = "";

  if (!origin_tz || !dest_tz)
    return;

  try {
    std::string tz_string;
    boost::posix_time::ptime const time_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::ptime origin_pt = time_epoch + boost::posix_time::seconds(origin_seconds);
    boost::local_time::local_date_time origin_date_time(origin_pt,origin_tz);

    boost::posix_time::ptime dest_pt = time_epoch + boost::posix_time::seconds(dest_seconds);
    boost::local_time::local_date_time dest_date_time(dest_pt,dest_tz);

    boost::gregorian::date o_date = origin_date_time.local_time().date();
    boost::gregorian::date d_date = dest_date_time.local_time().date();

    if (is_depart_at && dest_date_time.is_dst()) {
      boost::gregorian::date dst_date = dest_tz->dst_local_end_time(d_date.year()).date();
      bool in_range = (o_date <= dst_date && dst_date <= d_date);

      if (in_range) { // in range meaning via the dates.
        if (o_date == dst_date) {
          // must start before dst end time - the offset otherwise the time is ambiguous
          in_range = origin_date_time.local_time().time_of_day() <
              (dest_tz->dst_local_end_time(d_date.year()).time_of_day() - dest_tz->dst_offset());

          if (in_range) {
            // starts and ends on the same day.
            if (o_date == d_date)
              in_range = dest_tz->dst_local_end_time(d_date.year()).time_of_day() <= dest_date_time.local_time().time_of_day();
          }
        }
        else if (dst_date == d_date)
          in_range = dest_tz->dst_local_end_time(d_date.year()).time_of_day() <= dest_date_time.local_time().time_of_day();
      }
      if (in_range)
        dest_date_time -= dest_tz->dst_offset();
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
            if (o_date == d_date)
              in_range = origin_tz->dst_local_end_time(o_date.year()).time_of_day() > dest_date_time.local_time().time_of_day();
          }
        } else if (dst_date == d_date)
          in_range = origin_tz->dst_local_end_time(o_date.year()).time_of_day() > dest_date_time.local_time().time_of_day();
      }

      if (in_range)
        origin_date_time -= origin_tz->dst_offset();
    }

    origin_pt = origin_date_time.local_time();
    boost::gregorian::date date = origin_pt.date();
    std::stringstream ss_time;
    ss_time << origin_pt.time_of_day();
    std::string time = ss_time.str();

    std::size_t found = time.find_last_of(":"); // remove seconds.
    if (found != std::string::npos)
      time = time.substr(0,found);

    ss_time.str("");
    if (origin_date_time.is_dst())
      ss_time << origin_tz->dst_offset() + origin_tz->base_utc_offset();
    else ss_time << origin_tz->base_utc_offset();

    //postive tz
    if (ss_time.str().find("+") == std::string::npos && ss_time.str().find("-") == std::string::npos)
      iso_origin = to_iso_extended_string(date) + "T" + time + "+" + ss_time.str();
    else iso_origin = to_iso_extended_string(date) + "T" + time + ss_time.str();

    found = iso_origin.find_last_of(":"); // remove seconds.
    if (found != std::string::npos)
      iso_origin = iso_origin.substr(0,found);

    dest_pt = dest_date_time.local_time();
    date = dest_pt.date();
    ss_time.str("");
    ss_time << dest_pt.time_of_day();
    time = ss_time.str();

    found = time.find_last_of(":"); // remove seconds.
    if (found != std::string::npos)
      time = time.substr(0,found);

    ss_time.str("");
    if (dest_date_time.is_dst())
      ss_time << dest_tz->dst_offset() + dest_tz->base_utc_offset();
    else ss_time << dest_tz->base_utc_offset();

    //postive tz
    if (ss_time.str().find("+") == std::string::npos && ss_time.str().find("-") == std::string::npos)
      iso_dest = to_iso_extended_string(date) + "T" + time + "+" + ss_time.str();
    else iso_dest = to_iso_extended_string(date) + "T" + time + ss_time.str();

    found = iso_dest.find_last_of(":"); // remove seconds.
    if (found != std::string::npos)
      iso_dest = iso_dest.substr(0,found);

  } catch (std::exception& e){}
}

//Get the dow mask
//date_time is in the format of 20150516 or 2015-05-06T08:00
uint32_t day_of_week_mask(const std::string& date_time) {
  boost::gregorian::date date;
  date = get_formatted_date(date_time);

  if (date < pivot_date_)
    return kDOWNone;

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

//Get the number of seconds midnight that have elapsed.
uint32_t seconds_from_midnight(const std::string& date_time) {
  //date_time is in the format of HH:MM:SS or HH:MM or YYYY-MM-DDTHH:MM
  //hours can be greater than 24.
  //please see GTFS spec:
  //https://developers.google.com/transit/gtfs/reference#stop_times_fields

  boost::posix_time::time_duration td;
  std::size_t found = date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    td = boost::posix_time::duration_from_string(date_time.substr(found+1));
  else
    td = boost::posix_time::duration_from_string(date_time);

  return static_cast<uint32_t>(td.total_seconds());
}

//add x seconds to a date_time and return a ISO date_time string.
//date_time is in the format of 20150516 or 2015-05-06T08:00
std::string get_duration(const std::string& date_time, const uint32_t seconds,
                         const boost::local_time::time_zone_ptr& tz) {
  std::string formatted_date_time;
  boost::posix_time::ptime start;
  boost::gregorian::date date;
  if (date_time.find("T") != std::string::npos) {
    std::string dt = date_time;
    dt.erase(boost::remove_if(dt, boost::is_any_of("-,:")), dt.end());
    start = boost::posix_time::from_iso_string(dt);
    date = boost::gregorian::date_from_iso_string(dt);
  }
  else if (date_time.find("-") != std::string::npos) {
    std::string dt = date_time;
    dt.erase(boost::remove_if(dt, boost::is_any_of("-")), dt.end());
    start = boost::posix_time::from_iso_string(dt + "T0000");
    date = boost::gregorian::from_undelimited_string(dt);
  } else {
    //No time on date.  Make it midnight.
    start = boost::posix_time::from_iso_string(date_time + "T0000");
    date = boost::gregorian::from_undelimited_string(date_time);
  }

  if (date < pivot_date_)
    return formatted_date_time;

  boost::posix_time::ptime end = start + boost::posix_time::seconds(seconds);
  formatted_date_time = boost::posix_time::to_iso_extended_string(end);

  boost::local_time::local_date_time dt(end,tz);

  std::size_t found = formatted_date_time.find_last_of(":"); // remove seconds.
  if (found != std::string::npos)
    formatted_date_time = formatted_date_time.substr(0,found);

  std::stringstream ss;
  std::string tz_abbrev;
  if (dt.is_dst()) {
    ss << tz->dst_offset() + tz->base_utc_offset();
    //positive tz
    if (ss.str().find("+") == std::string::npos && ss.str().find("-") == std::string::npos) {
      ss.str("");
      ss << "+" << tz->dst_offset() + tz->base_utc_offset();
    }
    tz_abbrev = tz->dst_zone_abbrev();
  } else {
    ss << tz->base_utc_offset();
    //positive tz
    if (ss.str().find("+") == std::string::npos && ss.str().find("-") == std::string::npos) {
      ss.str("");
      ss << "+" << tz->base_utc_offset();
    }
    tz_abbrev = tz->std_zone_abbrev();
  }

  formatted_date_time += ss.str();

  found = formatted_date_time.find_last_of(":"); // remove seconds.
  if (found != std::string::npos)
    formatted_date_time = formatted_date_time.substr(0,found);

  formatted_date_time += " " + tz_abbrev;

  return formatted_date_time;
}

// checks if string is in the format of %Y-%m-%dT%H:%M
bool is_iso_local(const std::string& date_time) {

  std::stringstream ss("");
  bool is_ok = true;

  if (date_time.size() != 16)//YYYY-MM-DDTHH:MM
    return false;

  if (date_time.at(4) != '-' || date_time.at(7) != '-' ||
      date_time.at(10) != 'T' || date_time.at(13) != ':')
    return false;

  try {
    boost::local_time::local_time_input_facet* input_facet = new boost::local_time::local_time_input_facet();
    input_facet->format("%Y-%m-%dT%H:%M");

    ss.imbue(std::locale(ss.getloc(), input_facet));
    boost::posix_time::ptime pt;
    ss.str(date_time);
    is_ok = static_cast<bool>(ss >> pt);

    std::size_t found = date_time.find("T"); //YYYY-MM-DDTHH:MM
    std::string time = date_time.substr(found+1);
    uint32_t hour = std::stoi(time.substr(0,2));
    uint32_t min = std::stoi(time.substr(3));

    if (hour > 23 || min > 59)
      return false;

  } catch (std::exception& e){
    return false;
  }
  return is_ok;
}

//get the dow mask from user inputed string.  try to handle most inputs
uint8_t get_dow_mask(const std::string& dow) {

  std::string str = dow;
  std::transform(str.begin(), str.end(),str.begin(), ::toupper);
  str.erase(boost::remove_if(str, boost::is_any_of(":")), str.end());

  if (str == "SUNDAY" ||
      str == "SUN" || str == "SU")
    return kSunday;

  else if (str == "MONDAY" ||
           str == "MON" || str == "MO")
    return kMonday;

  else if (str == "TUESDAY" || str == "TUES" ||
           str == "TUE" || str == "TU")
    return kTuesday;

  else if (str == "WEDNESDAY" || str == "WEDS" ||
           str == "WED" || str == "WE")
    return kWednesday;

  else if (str == "THURSDAY" || str == "THURS" ||
           str == "THUR" || str == "TH")
    return kThursday;

  else if (str == "FRIDAY" ||
           str == "FRI" || str == "FR")
    return kFriday;

  else if (str == "SATURDAY" ||
           str == "SAT" || str == "SA")
    return kSaturday;
  return kDOWNone;
}

//get the dow from user inputed string.  try to handle most inputs
DOW get_dow(const std::string& dow) {

  std::string str = dow;
  std::transform(str.begin(), str.end(),str.begin(), ::toupper);
  str.erase(boost::remove_if(str, boost::is_any_of(":")), str.end());

  if (str == "SUNDAY" ||
      str == "SUN" || str == "SU")
    return DOW::kSunday;

  else if (str == "MONDAY" ||
           str == "MON" || str == "MO")
    return DOW::kMonday;

  else if (str == "TUESDAY" || str == "TUES" ||
           str == "TUE" || str == "TU")
    return DOW::kTuesday;

  else if (str == "WEDNESDAY" || str == "WEDS" ||
           str == "WED" || str == "WE")
    return DOW::kWednesday;

  else if (str == "THURSDAY" || str == "THURS" ||
           str == "THUR" || str == "TH")
    return DOW::kThursday;

  else if (str == "FRIDAY" ||
           str == "FRI" || str == "FR")
    return DOW::kFriday;

  else if (str == "SATURDAY" ||
           str == "SAT" || str == "SA")
    return DOW::kSaturday;
  return DOW::kNone;
}

//get the month from user inputed string.  try to handle most inputs
baldr::MONTH get_month(const std::string& month) {

  std::string str = month;
  std::transform(str.begin(), str.end(),str.begin(), ::toupper);
  str.erase(boost::remove_if(str, boost::is_any_of(":")), str.end());

  if (str == "JANUARY" || str == "JAN")
    return MONTH::kJan;

  else if (str == "FEBRUARY" || str == "FEB")
    return MONTH::kFeb;

  else if (str == "MARCH" || str == "MAR")
    return MONTH::kMar;

  else if (str == "APRIL" || str == "APR")
    return MONTH::kApr;

  else if (str == "MAY")
    return MONTH::kMay;

  else if (str == "JUNE" || str == "JUN")
    return MONTH::kJun;

  else if (str == "JULY" || str == "JUL")
    return MONTH::kJul;

  else if (str == "AUGUST" || str == "AUG")
    return MONTH::kAug;

  else if (str == "SEPTEMBER" || str == "SEP" ||
           str == "SEPT")
    return MONTH::kSep;

  else if (str == "OCTOBER" || str == "OCT")
    return MONTH::kOct;

  else if (str == "NOVEMBER" || str == "NOV")
    return MONTH::kNov;

  else if (str == "DECEMBER" || str == "DEC")
    return MONTH::kDec;
  return MONTH::kNone;
}

std::vector<std::string> GetTokens(const std::string& tag_value,
                                      char delim) {
  std::vector<std::string> tokens;
  boost::algorithm::split(tokens, tag_value,
                          std::bind1st(std::equal_to<char>(), delim),
                          boost::algorithm::token_compress_on);
  return tokens;
}

bool RegexFound(const std::string& source, const std::regex& regex) {
  auto begin = std::sregex_iterator(source.begin(), source.end(), regex);
  auto end = std::sregex_iterator();
  return std::distance(begin, end);
}

std::string FormatCondition(const std::string& source, const std::regex& regex,
                            const std::string& pattern) {
  return std::regex_replace(source, regex, pattern);
}

std::vector<uint64_t> get_time_range(const std::string& str) {

  std::vector<uint64_t> time_domains;

  TimeDomain timedomain;
  //rm ()
  std::string condition = str;
  condition.erase(boost::remove_if(condition, boost::is_any_of("()")), condition.end());

  //rm white space at both ends
  boost::algorithm::trim(condition);

  //Holidays and school hours skip for now
  if (condition.size() >= 2 && (condition.substr(0,2) == "PH" ||
      condition.substr(0,2) == "SH"))
    return time_domains;

  //Dec Su[-1]-Mar 3
  std::regex regex = std::regex("(?:(January|February|March|April|May|June|July|"
      "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
      "Sep|Sept|Oct|Nov|Dec)) (?:(Monday|Tuesday|Wednesday|Thursday|Friday|Saturday|"
      "Sunday|Mon|Mo|Tues|Tue|Tu|Weds|Wed|We|Thurs|Thur|Th|Fri|Fr|Sat|Sa|Sun|Su)(\\[-?[0-9]\\])-"
      "(?:(January|February|March|April|May|June|July|August|September|October|November"
      "|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Sept|Oct|Nov|Dec)) (\\d{1,2}))",std::regex_constants::icase);

  if (RegexFound(condition,regex)) {
    condition = FormatCondition(condition, regex, "$1#$2#$3-$4#$5");
    //fifth is the equivalent of last week in month (-1)
    condition = FormatCondition(condition, std::regex("\\[-1\\]"), "[5]");
  } else {

    //Mar 3-Dec Su[-1]
    std::regex regex = std::regex("(?:(January|February|March|April|May|June|July|August|"
        "September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Sept|Oct|"
        "Nov|Dec)) (\\d{1,2})-(?:(January|February|March|April|May|June|July|"
        "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
        "Sep|Sept|Oct|Nov|Dec)) (?:(Monday|Tuesday|Wednesday|Thursday|Friday|Saturday|"
        "Sunday|Mon|Mo|Tues|Tue|Tu|Weds|Wed|We|Thurs|Thur|Th|Fri|Fr|Sat|Sa|Sun|Su)(\\[-?[0-9]\\]))"
        ,std::regex_constants::icase);

    if (RegexFound(condition,regex)) {
      condition = FormatCondition(condition, regex, "$1#$2-$3#$4#$5");
      //fifth is the equivalent of last week in month (-1)
      condition = FormatCondition(condition, std::regex("\\[-1\\]"), "[5]");
    } else {

      //Dec Su[-1]
      regex = std::regex("(?:(January|February|March|April|May|June|July|"
          "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
          "Sep|Sept|Oct|Nov|Dec)) (?:(Monday|Tuesday|Wednesday|Thursday|Friday|Saturday|"
          "Sunday|Mon|Mo|Tues|Tue|Tu|Weds|Wed|We|Thurs|Thur|Th|Fri|Fr|Sat|Sa|Sun|Su)(\\[-?[0-9]\\]))",std::regex_constants::icase);

      if (RegexFound(condition,regex)) {
        condition = FormatCondition(condition, regex, "$1#$2#$3");
        //fifth is the equivalent of last week in month (-1)
        condition = FormatCondition( condition, std::regex("\\[-1\\]"), "[5]");
      } else {

        regex = std::regex("(?:(Monday|Tuesday|Wednesday|Thursday|Friday|Saturday|"
            "Sunday|Mon|Mo|Tues|Tue|Tu|Weds|Wed|We|Thurs|Thur|Th|Fri|Fr|Sat|Sa|Sun|Su)(\\[-?[0-9]\\]))",std::regex_constants::icase);

        if (RegexFound(condition,regex)) {
          condition = FormatCondition(condition, regex, "$1#$2");
          //fifth is the equivalent of last week in month (-1)
          condition = FormatCondition( condition, std::regex("\\[-1\\]"), "[5]");
        } else {

          //Feb 16-Oct 15 09:00-18:30
          regex = std::regex("(?:(January|February|March|April|May|June|July|"
              "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
              "Sep|Sept|Oct|Nov|Dec)) (\\d{1,2})",std::regex_constants::icase);

          if (RegexFound(condition,regex)) {
            condition = FormatCondition(condition, regex, "$1#$2");
          } else {
            //Feb 2-14
            regex = std::regex("(?:(January|February|March|April|May|June|July|"
                "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
                "Sep|Sept|Oct|Nov|Dec)) (\\d{1,2})-(\\d{1,2})",std::regex_constants::icase);

            if (RegexFound(condition,regex)) {
              condition = FormatCondition(condition, regex, "$1#$2-$1#$3");
            }
          }
        }
      }
    }
  }

  std::vector<std::string> months_dow_times = GetTokens(condition,' ');

  if (months_dow_times.size() == 1 && condition.find('#') != std::string::npos &&
      std::count(condition.begin(), condition.end(), '#') == 1) {
    months_dow_times = GetTokens(condition,'#');
  }

  if (months_dow_times.size() == 1) {
    //no dow just times
    //06:00-11:00,17:00-19:00
    if (months_dow_times.at(0).find('-') != std::string::npos &&
        months_dow_times.at(0).find(':') != std::string::npos) {

      std::vector<std::string> times = GetTokens(months_dow_times.at(0),',');
      //is this data looking good enough to try to process?
      if (times.size()) {

      } else return time_domains;

      //multiple times are saved as multiple restrictions
      for (const auto& t : times) {

        std::vector<std::string> on_off = GetTokens(t,'-');

        //do we have an hour on and hour off?
        if (on_off.size() == 2) {

          // process the hour on
          std::size_t found = on_off.at(0).find(":");
          if (found == std::string::npos)
            return time_domains;

          std::stringstream stream(on_off.at(0));
          uint32_t hour, min;

          stream >> hour;
          stream.ignore();
          stream >> min;

          timedomain.daterange.begin_hrs = hour;
          timedomain.daterange.begin_mins = min;

          //process the hour off
          found = on_off.at(1).find(":");
          if (found == std::string::npos)
            return time_domains;

          stream.str("");
          stream.clear();
          stream.str(on_off.at(1));

          stream >> hour;
          stream.ignore();
          stream >> min;

          timedomain.daterange.end_hrs = hour;
          timedomain.daterange.end_mins = min;

          time_domains.push_back(timedomain.value);

        }
      }
      return time_domains;
    }
  }
  //Mo-Fr 06:00-11:00,17:00-19:00; Sa 03:30-19:00
  //Apr-Sep: Mo-Fr 09:00-13:00,14:00-18:00; Apr-Sep: Sa 10:00-13:00
  //Mo,We,Th,Fr 12:00-18:00; Sa-Su 12:00-17:00
  //Feb#16-Oct#15 09:00-18:30; Oct#16-Nov#15: 09:00-17:30; Nov#16-Feb#15: 09:00-16:30
  // and etc.
  for (auto& mdt : months_dow_times) {
    //rm white space at both ends
    boost::algorithm::trim(mdt);

    std::vector<std::string> months_dow;
    bool is_range = false;
    bool is_date = false;
    bool is_nth_week = false;
    bool ends_nth_week = false;

    if (mdt.find(',') != std::string::npos)
      months_dow = GetTokens(mdt,',');
    else if (mdt.find('-') != std::string::npos) {
      months_dow = GetTokens(mdt,'-');
      is_range = true;

      if (months_dow.size() && mdt.find('#') != std::string::npos &&
          mdt.find('[') != std::string::npos && mdt.find(']') != std::string::npos) {
        is_date = true;
        is_nth_week = true;

        std::vector<std::string> tmp, result;
        for (auto& md : months_dow) {
          tmp = GetTokens(md,'#');
          result.insert(std::end(result), std::begin(tmp), std::end(tmp));
        }
        months_dow = result;
      }
      //Feb#16-Oct#15
      else if (months_dow.size() && mdt.find('#') != std::string::npos) {
        is_date = true;
        std::vector<std::string> tmp, result;
        for (auto& md : months_dow) {
          tmp = GetTokens(md,'#');
          result.insert(std::end(result), std::begin(tmp), std::end(tmp));
        }
        months_dow = result;
      }
    //Dec Su[-1] Su-Sa 15:00-17:00
    }else if (mdt.find('#') != std::string::npos &&
        mdt.find('[') != std::string::npos && mdt.find(']') != std::string::npos) {
      is_date = true;
      is_nth_week = true;
      months_dow = GetTokens(mdt,'#');
    } else if (mdt.find('#') != std::string::npos) { // May#15
        is_date = true;
        months_dow = GetTokens(mdt,'#');
    } else months_dow.push_back(mdt); //just one day: Th or month

    // dealing with months?
    if (get_month(months_dow.at(0)) != MONTH::kNone) {
      for (auto& md : months_dow) {

        //Feb#16-Oct#15
        if (months_dow.size() == 4 && is_date && is_range) {
          timedomain.daterange.begin_month = static_cast<uint8_t>(get_month(months_dow.at(0)));
          timedomain.daterange.begin_day_dow = std::stoi(months_dow.at(1));

          timedomain.daterange.end_month = static_cast<uint8_t>(get_month(months_dow.at(2)));
          timedomain.daterange.end_day_dow = std::stoi(months_dow.at(3));

          timedomain.daterange.type = kYMD;
          break;
        } //May 16-31
        else if (months_dow.size() == 3 && is_date && is_range) {
          timedomain.daterange.begin_month = static_cast<uint8_t>(get_month(months_dow.at(0)));
          timedomain.daterange.begin_day_dow = std::stoi(months_dow.at(1));

          timedomain.daterange.end_month = timedomain.daterange.begin_month;
          timedomain.daterange.end_day_dow = std::stoi(months_dow.at(2));

          timedomain.daterange.type = kYMD;
          break;
        }
        //Apr-Sep or May 15
        else if (months_dow.size() == 2) {

          timedomain.daterange.begin_month = static_cast<uint8_t>(get_month(months_dow.at(0)));
          baldr::MONTH month = get_month(months_dow.at(1));

          if (month != MONTH::kNone) {
            timedomain.daterange.end_month = static_cast<uint8_t>(month);
            timedomain.daterange.type = kYMD;
          } else if (is_date) { // May 15
            timedomain.daterange.type = kYMD;
            timedomain.daterange.begin_day_dow = std::stoi(months_dow.at(1));
            timedomain.daterange.end_month = timedomain.daterange.begin_month;
            timedomain.daterange.end_day_dow = timedomain.daterange.begin_day_dow;
          } else {
            return time_domains;
          }

          break;
        } else if (months_dow.size() == 1) {//May
          timedomain.daterange.type = kYMD;
          timedomain.daterange.begin_month = static_cast<uint8_t>(get_month(months_dow.at(0)));
          timedomain.daterange.end_month = static_cast<uint8_t>(get_month(months_dow.at(0)));
          break;
        } else if (is_nth_week) {//Oct Su[-1]-Mar Su[4] Su 09:00-16:00
          if (get_month(md) != MONTH::kNone) {

            timedomain.daterange.type = kNthDow;
            if (timedomain.daterange.begin_month == 0) {

              //assume the restriction is the entire week.
              timedomain.daterange.dow = kAllDaysOfWeek;

              timedomain.daterange.begin_month = static_cast<uint8_t>(get_month(md));
              //assume no range.  Dec Su[-1] Su-Sa 15:00-17:00 starts on the last week
              //in Dec and ends in the last week in Dec
              if (!is_range)
                timedomain.daterange.end_month = timedomain.daterange.begin_month;
            } else {
              timedomain.daterange.end_month = static_cast<uint8_t>(get_month(md));

              if (is_range && is_date &&
                  md != months_dow.at(months_dow.size()-1)) { //Dec Su[-1]-Mar 3 Sat

                if (months_dow.at(months_dow.size()-1).find('[') == std::string::npos) {
                  timedomain.daterange.end_day_dow = std::stoi(months_dow.at(months_dow.size()-1));
                  break;
                } else ends_nth_week = true;
              }
            }

          } else if (get_dow(md) != DOW::kNone) {

            if (timedomain.daterange.begin_day_dow == 0)
              timedomain.daterange.begin_day_dow = static_cast<uint8_t>(get_dow(md));
            else timedomain.daterange.end_day_dow = static_cast<uint8_t>(get_dow(md));

          } else if (md.find('[') != std::string::npos && md.find(']') != std::string::npos) {
            md.erase(boost::remove_if(md, boost::is_any_of("[]")), md.end());

            if (timedomain.daterange.begin_year_week == 0 && !ends_nth_week) {
              timedomain.daterange.begin_year_week = std::stoi(md);
              //assume no range.  Dec Su[-1] Su-Sa 15:00-17:00 starts on the last week
              //in Dec and ends in the last week in Dec
              if (!is_range)
                timedomain.daterange.end_year_week = timedomain.daterange.begin_year_week;
            } else timedomain.daterange.end_year_week = std::stoi(md);
          } else if (is_date && is_range && timedomain.daterange.begin_month != 0 &&
              timedomain.daterange.end_month == 0) { //Mar 3-Dec Su[-1] Sat
              timedomain.daterange.begin_day_dow = std::stoi(md);
          }
        }
      }
    }
    // dealing with dow
    else if (get_dow(months_dow.at(0)) != DOW::kNone) {
      //Mo,We,Th,Fr
      if (!is_range) {
        //wipe out assumption that this restriction is for the entire week.
        if (timedomain.daterange.type == kNthDow)
          timedomain.daterange.dow = 0;

        for (auto& md : months_dow){
          timedomain.daterange.dow += get_dow_mask(md);
        }

        if (months_dow_times.size() == 2) {
          std::string week = months_dow_times.at(1);
          //Su[1] every 1st Sunday of every month.
          if (week.find('[') != std::string::npos && week.find(']') != std::string::npos) {
            timedomain.daterange.type = kNthDow;
            week.erase(boost::remove_if(week, boost::is_any_of("[]")), week.end());
            timedomain.daterange.begin_year_week = std::stoi(week);
            break;
          }
        }
      //Mo-Fr
      } else if (months_dow.size() == 2) {
        //wipe out assumption that this restriction is for the entire week.
        if (timedomain.daterange.type == kNthDow)
          timedomain.daterange.dow = 0;

        uint8_t b_index = static_cast<uint8_t>(get_dow(months_dow.at(0)));
        uint8_t e_index = static_cast<uint8_t>(get_dow(months_dow.at(1)));
        while (b_index <= e_index) {
          timedomain.daterange.dow += (1 << (b_index-1));
          b_index++;
        }
      } else return time_domains;
    } else {

      std::vector<std::string> on_off;

      for (const auto& time : months_dow) {
        //is this data looking good enough to try to process?
        if (time.find('-') != std::string::npos &&
            time.find(':') != std::string::npos) {

            //multiple times are saved as multiple restrictions
            on_off = GetTokens(time,'-');

        } else if (is_range && months_dow.size() == 2) {
          on_off.insert(std::end(on_off), std::begin(months_dow), std::end(months_dow));
        } else continue;

        //do we have an hour on and hour off?
        if (on_off.size() == 2) {

          // process the hour on
          std::size_t found = on_off.at(0).find(":");
          if (found == std::string::npos)
            return time_domains;

          std::stringstream stream(on_off.at(0));
          uint32_t hour, min;

          stream >> hour;
          stream.ignore();
          stream >> min;

          timedomain.daterange.begin_hrs = hour;
          timedomain.daterange.begin_mins = min;

          //process the hour off
          found = on_off.at(1).find(":");
          if (found == std::string::npos)
            return time_domains;

          stream.str("");
          stream.clear();
          stream.str(on_off.at(1));

          stream >> hour;
          stream.ignore();
          stream >> min;

          timedomain.daterange.end_hrs = hour;
          timedomain.daterange.end_mins = min;

          time_domains.push_back(timedomain.value);
        }
      }
    }
  }

  //no time.
  if (time_domains.size() == 0 && timedomain.value)
    time_domains.push_back(timedomain.value);

  return time_domains;
}

}
}
}
