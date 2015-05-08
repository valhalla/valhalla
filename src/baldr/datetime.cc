#include <iostream>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"
#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {
namespace DateTime {

boost::gregorian::date pivot_date_ = boost::gregorian::from_undelimited_string(kPivotDate);

//Get the number of days that have elapsed from the pivot date for the inputed date.
uint32_t days_from_pivot_date(std::string date) {
  boost::gregorian::date e_date = boost::gregorian::from_undelimited_string(date);
  if (e_date <= pivot_date_)
    return 0;
  boost::gregorian::date_period range(pivot_date_, e_date);
  return static_cast<uint32_t>(range.length().days());
}

//Get the number of seconds midnight that have elapsed.
uint32_t seconds_from_midnight(std::string time) {
  //time is in the format of hh::mm::ss
  //hours can be greater than 24.
  //please see GTFS spec:
  //https://developers.google.com/transit/gtfs/reference#stop_times_fields
  boost::posix_time::time_duration td(boost::posix_time::duration_from_string(time));
  return static_cast<uint32_t>(td.total_seconds());
}

}
}
}
