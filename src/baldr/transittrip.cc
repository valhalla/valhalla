#include <string.h>
#include "baldr/transittrip.h"

namespace valhalla {
namespace baldr {

// Constructor with arguments
TransitTrip::TransitTrip(const uint32_t tripid,
             const uint32_t routeid, const char* tl_tripid,
             const uint32_t short_name_offset, const uint32_t headsign_offset)
    : tripid_(tripid),
      routeid_(routeid),
      short_name_offset_(short_name_offset),
      headsign_offset_(headsign_offset) {
  strncpy(tl_tripid_, tl_tripid, kOneStopIdSize);
}

// Get the internal route Id.
uint32_t TransitTrip::tripid() const {
  return tripid_;
}

// Get the internal route Id.
uint32_t TransitTrip::routeid() const {
  return routeid_;
}

// Get the TransitLand one-stop Id for this trip.
const char* TransitTrip::tl_tripid() const {
  return tl_tripid_;
}

// Get the text/name offset for the short trip name.
uint32_t TransitTrip::short_name_offset() const {
  return short_name_offset_;
}

// Get the text/name offset for the trip headsign.
uint32_t TransitTrip::headsign_offset() const {
  return headsign_offset_;
}

// operator < - for sorting. Sort by trip Id.
bool TransitTrip::operator < (const TransitTrip& other) const {
  return tripid() < other.tripid();
}

}
}
