#include "baldr/transitdeparture.h"

namespace valhalla {
namespace baldr {

// Get the edge Id - for lookup of all departures along this edge. Each edge
// represents a unique departure/arrival stop pair and route Id.
uint32_t TransitDeparture::edgeid() const {
  return edgeid_;
}

// Get the internal trip Id for this departure.
uint32_t TransitDeparture::tripid() const {
  return tripid_;
}

// Get the route Id (internal) for this departure.
uint32_t TransitDeparture::routeid() const {
  return routeid_;
}

// Get the block Id of this trip.
uint32_t TransitDeparture::blockid() const {
  return blockid_;
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
