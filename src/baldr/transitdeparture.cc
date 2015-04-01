#include "baldr/transitdeparture.h"

namespace valhalla {
namespace baldr {

// Get the stop Id (internal) of the departure stop.
uint32_t TransitDeparture::departurestop() const {
  return departurestop_;
}

// Get the stop Id (internal) of the arrival stop.
uint32_t TransitDeparture::arrivalstop() const {
  return arrivalstop_;
}

// Get the internal trip Id for this departure.
uint32_t TransitDeparture::tripid() const {
  return tripid_;
}

// Get the route Id (internal) for this departure.
uint32_t TransitDeparture::routeid() const {
  return routeid_;
}

// Get the headsign offset into the names/text list.
uint32_t TransitDeparture::headsign() const {
  return headsign_;
}

// Get the departure time.
uint32_t TransitDeparture::departure_time_() const {
  return times_.departure;
}

// Get the elapsed time until arrival at the next stop.
uint32_t TransitDeparture::elapsed_time() const {
  return times_.elapsed;
}

// Get the start date of this scheduled departure.
uint32_t TransitDeparture::start_date() const {
  return dates_.start;
}

// Get the end date for this scheduled departure.
uint32_t TransitDeparture::end_date() const {
  return dates_.end;
}

// Gets the days of the week for this departure.
uint32_t TransitDeparture::days() const {
  return dates_.days;
}

// Get the service Id (internal) for this departure.
uint32_t TransitDeparture::serviceid() const {
  return serviceid_;
}

}
}
