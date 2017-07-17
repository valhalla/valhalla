#include <string.h>
#include "baldr/transitstop.h"

namespace valhalla {
namespace baldr {

// Constructor with arguments. Validate input is within bounds of the
// name offset fields.
TransitStop::TransitStop(const uint32_t one_stop_offset,
                         const uint32_t name_offset,
                         const bool generated,
                         const uint32_t traversability)
    : spare_(0) {
  if (one_stop_offset > kMaxNameOffset) {
    throw std::runtime_error("TransitStop: Exceeded maximum name offset");
  }
  one_stop_offset_ = one_stop_offset;

  if (name_offset > kMaxNameOffset) {
    throw std::runtime_error("TransitStop: Exceeded maximum name offset");
  }
  name_offset_ = name_offset;

  generated_ = generated;
  traversability_ = traversability;
}

// Get the TransitLand one-stop Id.
uint32_t TransitStop::one_stop_offset() const {
  return one_stop_offset_;
}

// Get the text/name offset for the stop name.
uint32_t TransitStop::name_offset() const {
  return name_offset_;
}

// Get the generated flag.
bool TransitStop::generated() const {
  return generated_;
}

// Get the traversability
Traversability TransitStop::traversability() const {
  return static_cast<Traversability>(traversability_);
}

}
}
