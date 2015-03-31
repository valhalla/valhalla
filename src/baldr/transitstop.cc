#include "baldr/transitstop.h"
#include <boost/functional/hash.hpp>
#include <cmath>

namespace valhalla {
namespace baldr {

// Default constructor
TransitStop::TransitStop()
    : stopid_(0),
      tl_stopid_{},
      name_index_(0),
      desc_index_(0),
      parent_stopid_(0),
      fare_zoneid_(0) {
}

// Get the internal stop Id.
uint32_t TransitStop::stopid() const {
  return stopid_;
}

// Get the TransitLand one stop Id.
const char* TransitStop::tl_stopid() const {
  return tl_stopid_;
}

// Get the text/name index for the stop name.
uint32_t TransitStop::name_index() const {
  return name_index_;
}

// Get the text/name index for the stop description.
uint32_t TransitStop::desc_index() const {
  return desc_index_;
}

// Get the internal stop Id of the parent stop.
uint32_t TransitStop::parent_stopid() const {
  return parent_stopid_;
}

// Get the fare zone Id.
uint32_t TransitStop::fare_zoneid() const {
  return fare_zoneid_;
}

}
}
