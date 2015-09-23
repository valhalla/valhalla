#include <iostream>
#include <sstream>
#include <bitset>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/date_time/local_time/local_time_io.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/operations.hpp>

#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/date_time_zonespec.h>
#include <fstream>

namespace valhalla {
namespace baldr {
namespace DateTime {

namespace
{

const boost::gregorian::date pivot_date_ = boost::gregorian::from_undelimited_string(kPivotDate);

struct tz_db_t {
  tz_db_t() {
    std::string tz_data(date_time_zonespec_csv, date_time_zonespec_csv + date_time_zonespec_csv_len);
    std::stringstream ss(tz_data);
    db.load_from_stream(ss);
  }
  const boost::local_time::tz_database* operator->() const {
      return &db;
  }

  boost::local_time::tz_database db;
};

}

//Get the list of regions.
std::vector<std::string> get_region_list() {
  //thread safe static initialization of global singleton
  static tz_db_t tz_db;
  std::vector<std::string> v = tz_db->region_list();
  std::vector<std::string>::iterator it;
  it = v.begin();
  it = v.insert (it,"None");
  return v;
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
//Start from the start date to end date or 60 days, whichever is less.
//set the bits based on the dow.
uint64_t get_service_days(const std::string& start_date, std::string& end_date, const uint32_t& dow_mask) {

  //start_date is in the format of 20150516 or 2015-05-06T08:00
  boost::gregorian::date s_date;
  s_date = get_formatted_date(start_date);

  //end_date is in the format of 20150516 or 2015-05-06T08:00
  boost::gregorian::date e_date;
  e_date = get_formatted_date(end_date);

  boost::gregorian::date enddate = s_date + boost::gregorian::days(59);

  if (enddate <= e_date) {
    e_date = enddate;
    end_date = to_iso_extended_string(e_date);
  }

  boost::gregorian::day_iterator itr(s_date);
  uint32_t x = 0;
  std::bitset<64> bit_set;

  while (itr <= e_date) {

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

    if (dow_mask & dow)
      bit_set.set(x);

    ++itr;
    ++x;
  }
  return bit_set.to_ulong();
}

//add a service day to the days if it is in range.
uint64_t add_service_day(const uint64_t& days, const std::string& start_date,
                         const std::string& end_date, const std::string& added_date) {

  //start_date is in the format of 20150516 or 2015-05-06T08:00
  boost::gregorian::date s_date;
  s_date = get_formatted_date(start_date);

  //end_date is in the format of 20150516 or 2015-05-06T08:00
  boost::gregorian::date e_date;
  e_date = get_formatted_date(end_date);

  boost::gregorian::date enddate = s_date + boost::gregorian::days(59);

  if (enddate <= e_date)
    e_date = enddate;

  //added_date is in the format of 20150516 or 2015-05-06T08:00
  boost::gregorian::date a_date;
  a_date = get_formatted_date(added_date);

  if (s_date <= a_date && a_date <= e_date) {
    std::bitset<64> bit_set(days);
    boost::gregorian::date_period range(s_date, a_date);
    uint32_t length = range.length().days();
    bit_set.set(length);
    return bit_set.to_ulong();
  }
  return days;
}

//remove a service day to the days if it is in range.
uint64_t remove_service_day(const uint64_t& days, const std::string& start_date,
                            const std::string& end_date, const std::string& removed_date) {

  //start_date is in the format of 20150516 or 2015-05-06T08:00
  boost::gregorian::date s_date;
  s_date = get_formatted_date(start_date);

  //end_date is in the format of 20150516 or 2015-05-06T08:00
  boost::gregorian::date e_date;
  e_date = get_formatted_date(end_date);

  boost::gregorian::date enddate = s_date + boost::gregorian::days(59);

  if (enddate <= e_date)
    e_date = enddate;

  //removed_date is in the format of 20150516 or 2015-05-06T08:00
  boost::gregorian::date r_date;
  r_date = get_formatted_date(removed_date);

  if (s_date <= r_date && r_date <= e_date) {
    std::bitset<64> bit_set(days);
    boost::gregorian::date_period range(s_date, r_date);
    uint32_t length = range.length().days();
    bit_set.reset(length);
    return bit_set.to_ulong();
  }
  return days;
}

//Get the number of days that have elapsed from the pivot date for the inputed date.
//date_time is in the format of 20150516 or 2015-05-06T08:00
uint32_t days_from_pivot_date(const std::string& date_time) {
  boost::gregorian::date e_date;
  e_date = get_formatted_date(date_time);

  if (e_date <= pivot_date_)
    return 0;
  boost::gregorian::date_period range(pivot_date_, e_date);
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
                          const std::string& tz) {
  //thread safe static initialization of global singleton
  static tz_db_t tz_db;

  std::string iso_date_time;
  std::stringstream ss("");
  if (time.empty() || time.find(":") == std::string::npos || tz.empty())
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
    boost::local_time::time_zone_ptr time_zone = tz_db->time_zone_from_region(tz);
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
std::string iso_date_time(const std::string& tz) {
  //thread safe static initialization of global singleton
  static tz_db_t tz_db;

  std::string iso_date_time;
  if (tz.empty())
    return iso_date_time;

  try {
    boost::posix_time::ptime pt = boost::posix_time::second_clock::universal_time();
    std::string tz_string;
    boost::local_time::time_zone_ptr time_zone = tz_db->time_zone_from_region(tz);
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

}
}
}
