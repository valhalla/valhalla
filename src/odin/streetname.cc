#include <iostream>

#include "odin/streetname.h"

namespace valhalla {
namespace odin {

StreetName::StreetName(const std::string& name)
    : name_(name) {
}

const std::string& StreetName::name() const {
  return name_;
}

bool StreetName::operator ==(const StreetName& rhs) const {
  return (name_ == rhs.name_);
}

}
}
