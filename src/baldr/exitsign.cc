#include "baldr/exitsign.h"

namespace valhalla {
namespace baldr {

const ExitSign::Type& ExitSign::type() const {
  return type_;
}

const uint32_t ExitSign::text_offset() const {
  return text_offset_;
}

ExitSign::ExitSign(const ExitSign::Type& type, uint32_t text_offset)
    : type_(type),
      text_offset_(text_offset) {
}

}
}
