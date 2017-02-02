#include "baldr/signinfo.h"

namespace valhalla {
namespace baldr {

// Get the sign type
const Sign::Type& SignInfo::type() const {
  return type_;
}

// Get the sign text
const std::string& SignInfo::text() const {
  return text_;
}

// Constructor
SignInfo::SignInfo(const Sign::Type& type, const std::string& text)
    : type_(type),
      text_(text) {
}

}
}
