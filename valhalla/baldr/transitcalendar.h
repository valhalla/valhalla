#ifndef VALHALLA_BALDR_TRANSITCALENDAR_H_
#define VALHALLA_BALDR_TRANSITCALENDAR_H_

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Contains service exceptions indexed by service Id. "Normal" schedule
 * information is contained in TripDepartures - this just includes exceptions
 * from calendar_dates.txt.
 */
class TransitCalendar {
 public:
  // Constructor with args.
  TransitCalendar(const uint32_t serviceid, const uint32_t date,
                  const CalendarExceptionType type);

  /**
   * Get the service Id for this calendar exception.
   * @return  Returns the service Id.
   */
  uint32_t serviceid() const;

  /**
   * Get the date of the calendar exception.
   * @return  Returns the date (form TODO)
   */
 uint32_t date() const;

 /**
  * Gets the exception type (add or remove).
  * @return  Returns the exception type.
  */
 CalendarExceptionType type() const;

 /**
  * operator < - for sorting. Sort by service Id and date.
  * @param  other  Other transit calendar exception to compare to.
  * @return  Returns true if service Id < other service Id or
  *          service Ids are equal and date < other date.
  */
 bool operator < (const TransitCalendar& other) const;

 protected:
  // Service Id - used to index the calendar exceptions.
  uint32_t serviceid_;

  // Exception date and type
  struct CalendarException {
    uint32_t date   : 12;   // Date
    uint32_t type   : 4;    // Exception type
    uint32_t spare  : 16;
  };
  CalendarException exception_;
};

}
}

#endif  // VALHALLA_BALDR_TRANSITCALENDAR_H_
