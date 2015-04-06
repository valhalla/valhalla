#include "baldr/transittrip.h"

namespace valhalla {
namespace baldr {

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

// Get the text/name index for the short trip name.
uint32_t TransitTrip::short_name_index() const {
  return short_name_index_;
}

// Get the text/name index for the trip headsign.
uint32_t TransitTrip::headsign_index() const {
  return headsign_index_;
}

}
}
