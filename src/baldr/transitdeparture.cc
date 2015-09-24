#include "baldr/transitdeparture.h"

namespace valhalla {
namespace baldr {

// Construct with arguments
TransitDeparture::TransitDeparture(const uint32_t lineid,
                 const uint32_t tripid, const uint32_t routeid,
                 const uint32_t blockid,
                 const uint32_t headsign_offset,
                 const uint32_t departure_time,
                 const uint32_t elapsed_time,
                 const uint32_t start_date,
                 const uint32_t end_date,
                 const uint32_t days_of_week,
                 const uint64_t days)
    : lineid_(lineid),
      tripid_(tripid),
      routeid_(routeid),
      blockid_(blockid),
      headsign_offset_(headsign_offset),
      days_(days){
  // TODO - protect against max values...
  times_.departure = departure_time;
  uint32_t elapsed = (elapsed_time < 32767) ? elapsed_time : 32767;
  times_.elapsed      = elapsed;
  dates_.start        = start_date;
  dates_.end          = end_date;
  dates_.days_of_week = days_of_week;
}

// Get the line Id - for lookup of all departures along an edge. Each line Id
// represents a unique departure/arrival stop pair and route Id.
uint32_t TransitDeparture::lineid() const {
  return lineid_;
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
uint32_t TransitDeparture::headsign_offset() const {
  return headsign_offset_;
}

// Get the departure time.
uint32_t TransitDeparture::departure_time() const {
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
uint32_t TransitDeparture::days_of_week() const {
  return dates_.days_of_week;
}

// Gets the days for this departure.
uint64_t TransitDeparture::days() const {
  return days_;
}

// operator < - for sorting. Sort by line Id and departure time.
bool TransitDeparture::operator < (const TransitDeparture& other) const {
  if (lineid() == other.lineid()) {
    return departure_time() < other.departure_time();
  } else {
    return lineid() < other.lineid();
  }
}

}
}
