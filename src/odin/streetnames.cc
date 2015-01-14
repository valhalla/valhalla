#include <iostream>

#include "odin/streetnames.h"

namespace valhalla {
namespace odin {

StreetNames::StreetNames() {
}

const std::list<StreetName>& StreetNames::names() const {
  return names_;
}

bool StreetNames::empty() const {
  return names_.empty();
}

void StreetNames::push_back(const StreetName& street_name) {
  names_.push_back(street_name);
}

}
}
