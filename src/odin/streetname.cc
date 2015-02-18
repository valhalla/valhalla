#include <iostream>

#include "odin/streetname.h"

namespace valhalla {
namespace odin {

StreetName::StreetName(const std::string& value)
    : value_(value) {
}

const std::string& StreetName::value() const {
  return value_;
}

bool StreetName::operator ==(const StreetName& rhs) const {
  return (value_ == rhs.value_);
}

}
}
