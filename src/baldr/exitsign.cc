#include "baldr/exitsign.h"

namespace valhalla {
namespace baldr {

const ExitSign::Type& ExitSign::type() const {
  return type_;
}

const uint32_t ExitSign::text_index() const {
  return text_index_;
}

ExitSign::ExitSign(const ExitSign::Type& type, uint32_t text_index)
    : type_(type),
      text_index_(text_index) {
}

}
}
