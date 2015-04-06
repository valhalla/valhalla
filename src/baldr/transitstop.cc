#include <string.h>
#include "baldr/transitstop.h"

namespace valhalla {
namespace baldr {

// Constructor with arguments
TransitStop::TransitStop(const uint32_t stopid, const char* tl_stopid,
            const uint32_t name_offset, const uint32_t desc_offset,
            const uint32_t parent_stopid, const uint32_t fare_zoneid)
    : stopid_(stopid),
      name_offset_(name_offset),
      desc_offset_(desc_offset),
      parent_stopid_(parent_stopid),
      fare_zoneid_(fare_zoneid) {
  strncpy(tl_stopid_, tl_stopid, kOneStopIdSize);
}

// Get the internal stop Id.
uint32_t TransitStop::stopid() const {
  return stopid_;
}

// Get the TransitLand one-stop Id.
const char* TransitStop::tl_stopid() const {
  return tl_stopid_;
}

// Get the text/name offset for the stop name.
uint32_t TransitStop::name_offset() const {
  return name_offset_;
}

// Get the text/name offset for the stop description.
uint32_t TransitStop::desc_offset() const {
  return desc_offset_;
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
