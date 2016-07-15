#include "baldr/transitdeparture.h"

#include <valhalla/midgard/logging.h>

namespace valhalla {
namespace baldr {

// Construct with arguments
TransitDeparture::TransitDeparture(const uint32_t lineid,
                 const uint32_t tripid, const uint32_t routeid,
                 const uint32_t blockid,
                 const uint32_t headsign_offset,
                 const uint32_t departure_time,
                 const uint32_t elapsed_time,
                 const uint32_t schedule_index)
    : tripid_(tripid),
      headsign_offset_(headsign_offset) {
  // Protect against exceeding max. values
  if (lineid > kMaxTransitLineId) {
    throw std::runtime_error("Exceeded maximum transit line Ids per tile");
  }
  lineid_ = lineid;

  if (routeid > kMaxTransitRoutes) {
    throw std::runtime_error("Exceeded maximum transit routes per tile");
  }
  routeid_ = routeid;

  if (blockid > kMaxTransitBlockId) {
    throw std::runtime_error("Exceeded maximum transit block Id");
  }
  blockid_ = blockid;

  if (schedule_index > kMaxTransitSchedules) {
    throw std::runtime_error("Exceeded maximum transit schedules per tile");
  }
  schedule_index_ = schedule_index;

  if (departure_time_ > kMaxTransitDepartureTime) {
    throw std::runtime_error("Exceeded maximum transit departure time");
  }
  departure_time_ = departure_time;

  if (elapsed_time > kMaxTransitElapsedTime) {
    LOG_ERROR("Elapsed time = " + std::to_string(elapsed_time));
    elapsed_time_ = kMaxTransitElapsedTime;
    //throw std::runtime_error("Exceeded maximum transit elapsed time");
  } else {
    elapsed_time_ = elapsed_time;
  }
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

// Get the schedule index.
uint32_t TransitDeparture::schedule_index() const {
  return schedule_index_;
}

// operator < - for sorting. Sort by line Id and departure time.
bool TransitDeparture::operator < (const TransitDeparture& other) const {
  if (lineid() == other.lineid()) {
    if (departure_time() == other.departure_time()) {
      return tripid_ < other.tripid_;
    } else {
      return departure_time() < other.departure_time();
    }
  } else {
    return lineid() < other.lineid();
  }
}

}
}
