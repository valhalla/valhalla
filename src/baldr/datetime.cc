#include <iostream>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {
namespace DateTime {

boost::gregorian::date pivot_date_ = boost::gregorian::from_undelimited_string(kPivotDate);

//Get the number of days that have elapsed from the pivot date for the inputed date.
//Date in the format of 20150516 or 2015-05-06T08:00
uint32_t days_from_pivot_date(std::string date_time) {
  boost::gregorian::date e_date;
  if (date_time.find("T") != std::string::npos) {
    date_time.erase(boost::remove_if(date_time, boost::is_any_of("-,:")), date_time.end());
    e_date = boost::gregorian::date_from_iso_string(date_time);
  }
  else
    e_date = boost::gregorian::from_undelimited_string(date_time);

  if (e_date <= pivot_date_)
    return 0;
  boost::gregorian::date_period range(pivot_date_, e_date);
  return static_cast<uint32_t>(range.length().days());
}

//Get the dow mask
//Date in the format of 20150516 or 2015-05-06T08:00
uint32_t day_of_week_mask(std::string date_time) {
  boost::gregorian::date date;
  if (date_time.find("T") != std::string::npos) {
    date_time.erase(boost::remove_if(date_time, boost::is_any_of("-,:")), date_time.end());
    date = boost::gregorian::date_from_iso_string(date_time);
  }
  else
    date = boost::gregorian::from_undelimited_string(date_time);

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
uint32_t seconds_from_midnight(std::string date_time) {
  //date_time is in the format of HH:MM:SS or HH:MM or YYYY-MM-DDTHH:MM
  //hours can be greater than 24.
  //please see GTFS spec:
  //https://developers.google.com/transit/gtfs/reference#stop_times_fields

  std::size_t found = date_time.find("T"); // YYYY-MM-DDTHH:MM
  if (found != std::string::npos)
    date_time = date_time.substr(found+1);

  boost::posix_time::time_duration td(boost::posix_time::duration_from_string(date_time));
  return static_cast<uint32_t>(td.total_seconds());
}

}
}
}
