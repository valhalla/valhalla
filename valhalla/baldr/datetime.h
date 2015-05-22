#ifndef VALHALLA_BALDR_DATETIME_H_
#define VALHALLA_BALDR_DATETIME_H_

#include <string>
#include <memory>

namespace valhalla {
namespace baldr {
namespace DateTime {

  /**
   * Get the number of days elapsed from the pivot date until
   * inputed date.
   * @param   date in the format of 20150516 or 2015-05-06T08:00
   * @return  Returns the number of days.
   */
  uint32_t days_from_pivot_date(std::string date_time);

  /**
   * Get the number of seconds elapsed from midnight.
   * Hours can be greater than 24.
   * @param   time in the format of 01:34:15 or 2015-05-06T08:00
   * @return  Returns the seconds from midnight.
   */
  uint32_t seconds_from_midnight(std::string date_time);

}
}
}
#endif  // VALHALLA_BALDR_DATETIME_H_
