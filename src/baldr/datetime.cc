#include <iostream>
#include <sstream>
#include <bitset>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/c_local_time_adjustor.hpp>

#include <boost/algorithm/string.hpp>

#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/date_time_zonespec.h>
#include <fstream>

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

//Get service days
//Start from the tile_date or start date to end date or 60 days (whichever is less)
//start_date will be updated to the tile creation date if the start date is in the past
//set the bits based on the dow.
uint64_t get_service_days(boost::gregorian::date& start_date, boost::gregorian::date& end_date,
                          const uint32_t tile_date, const uint32_t dow_mask) {

  boost::gregorian::date tile_header_date = pivot_date_ + boost::gregorian::days(tile_date);
  if (start_date <= tile_header_date && tile_header_date <= end_date)
    start_date = tile_header_date;
  else if (tile_header_date > end_date) //reject.
    return 0;

  // only support 60 days out.  (59 days and include the end_date = 60)
  boost::gregorian::date enddate = start_date + boost::gregorian::days(59);
  if (enddate <= end_date)
    end_date = enddate;

  boost::gregorian::day_iterator itr(start_date);
  uint32_t x = 0;
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

    //were we supposed to be on for this day
    if (dow_mask & dow)
      bit_set |= static_cast<uint64_t>(1) << x;

    ++itr;
    ++x;
  }
  return bit_set;
}

//add a service day to the days if it is in range.
uint64_t add_service_day(const uint64_t& days, const boost::gregorian::date& start_date,
                         const boost::gregorian::date& end_date, const boost::gregorian::date& added_date) {
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
uint64_t remove_service_day(const uint64_t& days, const boost::gregorian::date& start_date,
                            const boost::gregorian::date& end_date, const boost::gregorian::date& removed_date) {
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
bool is_service_available(const uint64_t& days, const uint32_t& start_date, const uint32_t& date, const uint32_t& end_date) {

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

//Get the time from the inputed date.
//date_time is in the format of 2015-05-06T08:00
std::string time(const std::string& date_time) {
  std::stringstream ss("");
  try {
    if (date_time.find("T") == std::string::npos)
      return ss.str();

    boost::local_time::local_time_input_facet* input_facet = new boost::local_time::local_time_input_facet();
    boost::local_time::local_time_facet* output_facet = new boost::local_time::local_time_facet();

    input_facet->format("%Y-%m-%dT%H:%M");
    output_facet->format("%l:%M %p");

    ss.imbue(std::locale(std::locale::classic(), output_facet));
    ss.imbue(std::locale(ss.getloc(), input_facet));

    boost::local_time::local_date_time ldt(boost::local_time::not_a_date_time);
    ss.str(date_time);
    ss >> ldt;
    ss.str("");
    ss << ldt;

  } catch (std::exception& e){}
  std::string result = ss.str();
  boost::algorithm::trim(result);
  return result;
}

//Get the date from the inputed date.
//date_time is in the format of 2015-05-06T08:00
std::string date(const std::string& date_time) {
  std::stringstream ss("");
  try {
    if (date_time.find("T") == std::string::npos)
      return ss.str();

    boost::local_time::local_time_input_facet* input_facet = new boost::local_time::local_time_input_facet();
    boost::local_time::local_time_facet* output_facet = new boost::local_time::local_time_facet();

    input_facet->format("%Y-%m-%dT%H:%M");
    output_facet->format("%Y%m%d");

    ss.imbue(std::locale(std::locale::classic(), output_facet));
    ss.imbue(std::locale(ss.getloc(), input_facet));

    boost::local_time::local_date_time ldt(boost::local_time::not_a_date_time);
    ss.str(date_time);
    ss >> ldt;
    ss.str("");
    ss << ldt;
  } catch (std::exception& e){}
  std::string result = ss.str();
  boost::algorithm::trim(result);
  return result;
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

    boost::local_time::local_date_time in_local_time(date, td, time_zone, false);
    boost::local_time::time_zone_ptr tz_utc(new boost::local_time::posix_time_zone("UTC"));
    boost::local_time::local_date_time local_date_time = in_local_time.local_time_in(tz_utc);
    boost::posix_time::ptime const time_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::time_duration diff = local_date_time.utc_time() - time_epoch;

    return diff.total_seconds();

  } catch (std::exception& e){}
  return 0;
}

// Get the date from seconds and timezone.
std::string seconds_to_date(uint64_t seconds, const boost::local_time::time_zone_ptr& time_zone) {

  std::string iso_date_time;
  if (!time_zone)
    return iso_date_time;

  try {
    std::string tz_string;

    boost::posix_time::ptime const time_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::ptime pt = time_epoch + boost::posix_time::seconds(seconds);
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
std::string get_duration(const std::string& date_time, const uint32_t seconds) {
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

  std::size_t found = formatted_date_time.find_last_of(":"); // remove seconds.
  if (found != std::string::npos)
    formatted_date_time = formatted_date_time.substr(0,found);

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
    is_ok = ss >> pt;

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

}
}
}
