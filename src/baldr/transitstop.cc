#include <string.h>
#include "baldr/transitstop.h"

namespace valhalla {
namespace baldr {

// Constructor with arguments
TransitStop::TransitStop(const uint32_t one_stop_offset,
                         const uint32_t name_offset)
    : one_stop_offset_(one_stop_offset),
      name_offset_(name_offset) {
}

// Get the TransitLand one-stop Id.
uint32_t TransitStop::one_stop_offset() const {
  return one_stop_offset_;
}

// Get the text/name offset for the stop name.
uint32_t TransitStop::name_offset() const {
  return name_offset_;
}

}
}
