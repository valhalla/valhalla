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
                 const uint32_t end_day,
                 const uint32_t days_of_week,
                 const uint64_t days)
    : lineid_(lineid),
      tripid_(tripid),
      routeid_(routeid),
      blockid_(blockid),
      headsign_offset_(headsign_offset),
      departure_time_(departure_time),
      end_day_(end_day),
      days_of_week_(days_of_week),
      days_(days) {
  // TODO - protect against max values...blockId?
  uint32_t elapsed = (elapsed_time < 32767) ? elapsed_time : 32767;
  elapsed_time_    = elapsed;
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
  return departure_time_;
}

// Get the elapsed time until arrival at the next stop.
uint32_t TransitDeparture::elapsed_time() const {
  return elapsed_time_;
}

// Get the end day for this scheduled departure.
uint32_t TransitDeparture::end_day() const {
  return end_day_;
}

// Gets the days of the week for this departure.
uint32_t TransitDeparture::days_of_week() const {
  return days_of_week_;
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
