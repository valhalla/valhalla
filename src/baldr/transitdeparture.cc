#include <stdexcept>

#include "baldr/transitdeparture.h"
#include "midgard/logging.h"

namespace valhalla {
namespace baldr {

// Constructor for a fixed schedule departure
TransitDeparture::TransitDeparture(const uint32_t lineid,
                                   const uint32_t tripid,
                                   const uint32_t routeindex,
                                   const uint32_t blockid,
                                   const uint32_t headsign_offset,
                                   const uint32_t departure_time,
                                   const uint32_t elapsed_time,
                                   const uint32_t schedule_index,
                                   const bool wheelchair_accessible,
                                   const bool bicycle_accessible) {
  // Set type to fixed schedule
  type_ = kFixedSchedule;
  spare_ = 0;

  // Protect against exceeding max. values
  if (lineid > kMaxTransitLineId) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit line Ids per tile");
  }
  lineid_ = lineid;

  if (routeindex > kMaxTransitRoutes) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit routes per tile");
  }
  routeindex_ = routeindex;

  if (tripid > kMaxTripId) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum trip Id");
  }
  tripid_ = tripid;

  if (headsign_offset > kMaxNameOffset) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum headsign offset");
  }
  headsign_offset_ = headsign_offset;

  if (blockid > kMaxTransitBlockId) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit block Id");
  }
  blockid_ = blockid;

  if (schedule_index > kMaxTransitSchedules) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit schedules per tile");
  }
  schedule_index_ = schedule_index;

  if (departure_time > kMaxTransitDepartureTime) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit departure time");
  }
  departure_times_.fixed_.departure_time_ = departure_time;

  if (elapsed_time > kMaxTransitElapsedTime) {
    LOG_ERROR("Elapsed time = " + std::to_string(elapsed_time));
    departure_times_.fixed_.elapsed_time_ = kMaxTransitElapsedTime;
  } else {
    departure_times_.fixed_.elapsed_time_ = elapsed_time;
  }

  wheelchair_accessible_ = wheelchair_accessible;
  bicycle_accessible_ = bicycle_accessible;
}

// Constructor for a frequency based schedule departure
TransitDeparture::TransitDeparture(const uint32_t lineid,
                                   const uint32_t tripid,
                                   const uint32_t routeindex,
                                   const uint32_t blockid,
                                   const uint32_t headsign_offset,
                                   const uint32_t departure_time,
                                   const uint32_t end_time,
                                   const uint32_t frequency,
                                   const uint32_t elapsed_time,
                                   const uint32_t schedule_index,
                                   const bool wheelchair_accessible,
                                   const bool bicycle_accessible) {
  // Set type to frequency schedule
  type_ = kFrequencySchedule;
  spare_ = 0;

  // Protect against exceeding max. values
  if (lineid > kMaxTransitLineId) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit line Ids per tile");
  }
  lineid_ = lineid;

  if (routeindex > kMaxTransitRoutes) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit routes per tile");
  }
  routeindex_ = routeindex;

  if (tripid > kMaxTripId) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum trip Id");
  }
  tripid_ = tripid;

  if (headsign_offset > kMaxNameOffset) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum name offset");
  }
  headsign_offset_ = headsign_offset;

  if (blockid > kMaxTransitBlockId) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit block Id");
  }
  blockid_ = blockid;

  if (schedule_index > kMaxTransitSchedules) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit schedules per tile");
  }
  schedule_index_ = schedule_index;

  if (departure_time > kMaxTransitDepartureTime) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit schedule departure time");
  }
  departure_times_.frequency_.departure_time_ = departure_time;

  if (end_time > kMaxEndTime) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit schedule end time");
  }
  departure_times_.frequency_.end_time_ = end_time;

  if (frequency > kMaxFrequency) {
    throw std::runtime_error("TransitDeparture: Exceeded maximum transit schedule frequency");
  }
  departure_times_.frequency_.frequency_ = frequency;

  if (elapsed_time > kMaxTransitElapsedTime) {
    LOG_ERROR("Elapsed time = " + std::to_string(elapsed_time));
    departure_times_.frequency_.elapsed_time_ = kMaxTransitElapsedTime;
  } else {
    departure_times_.frequency_.elapsed_time_ = elapsed_time;
  }

  wheelchair_accessible_ = wheelchair_accessible;
  bicycle_accessible_ = bicycle_accessible;
}

// operator < - for sorting. Sort by line Id and departure time.
bool TransitDeparture::operator<(const TransitDeparture& other) const {
  if (lineid() == other.lineid()) {
    if (type() == other.type()) {
      if (departure_time() == other.departure_time()) {
        return tripid() < other.tripid();
      } else {
        return departure_time() < other.departure_time();
      }
    } else {
      return type() < other.type();
    }
  } else {
    return lineid() < other.lineid();
  }
}

} // namespace baldr
} // namespace valhalla
