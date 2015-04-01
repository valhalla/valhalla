#include "baldr/transitstop.h"

namespace valhalla {
namespace baldr {

// Get the internal stop Id.
uint32_t TransitStop::stopid() const {
  return stopid_;
}

// Get the TransitLand one-stop Id.
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
