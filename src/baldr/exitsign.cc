#include "baldr/exitsign.h"

namespace valhalla {
namespace baldr {

// Constructor given parameters.
ExitSign::ExitSign(const uint32_t idx, const ExitSign::Type& type,
                   const uint32_t text_offset)
    : edgeindex_(idx),
      type_(type),
      text_offset_(text_offset) {
}

// Get the directed edge index to which this sign applies.
uint32_t ExitSign::edgeindex() const {
  return edgeindex_;
}

// Get the exit sign type.
const ExitSign::Type& ExitSign::type() const {
  return type_;
}

// Get the offset within the text/names list for the sign text.
const uint32_t ExitSign::text_offset() const {
  return text_offset_;
}

}
}
