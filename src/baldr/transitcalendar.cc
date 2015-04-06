#include "baldr/transitcalendar.h"

namespace valhalla {
namespace baldr {

TransitCalendar::TransitCalendar(const uint32_t serviceid,
                                 const uint32_t date,
                const CalendarExceptionType type)
    : serviceid_(serviceid) {
  // TODO - check limits once form is finalized
  exception_.date = date;
  exception_.type = static_cast<uint32_t>(type);
}

// Get the service Id for this calendar exception.
uint32_t TransitCalendar::serviceid() const {
  return serviceid_;
}

// Get the date of the calendar exception.
uint32_t TransitCalendar::date() const {
  return exception_.date;
}

// Gets the exception type (add or remove).
CalendarExceptionType TransitCalendar::type() const {
  return static_cast<CalendarExceptionType>(exception_.type);
}

}
}
