#ifndef VALHALLA_BALDR_DATETIME_H_
#define VALHALLA_BALDR_DATETIME_H_

#include <string>
#include <memory>
#include "boost/date_time/gregorian/gregorian.hpp"

namespace valhalla {
namespace baldr {

//Date and time class for transit departure and arrival date and times.
class DateTime {
 public:
  DateTime();

  /**
   * Get the number of days elapsed from the pivot date until
   * inputed date.
   * @param   date in the format of 20150516
   * @return  Returns the number of days.
   */
  uint32_t getDaysFromPivotDate(std::string date);

  /**
   * Get the number of seconds elapsed from midnight.
   * Hours can be greater than 24.
   * @param   time in the format of 01:34:15
   * @return  Returns the seconds from midnight.
   */
  uint32_t getSecondsFromMidnight(std::string time);

 protected:
  //This is our pivot date for transit.  No dates will be older than this date.
  boost::gregorian::date pivot_date_;
};

}
}

#endif  // VALHALLA_BALDR_DATETIME_H_
