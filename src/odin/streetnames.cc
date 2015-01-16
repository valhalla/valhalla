#include <iostream>

#include "odin/streetnames.h"

namespace valhalla {
namespace odin {

StreetNames::StreetNames()
    : std::list<StreetName>() {
}

std::string StreetNames::ToString() const {
  std::string name_string;
  if (empty())
    name_string = "unnamed";
  for (auto& street_name : *this) {
    if (!name_string.empty()) {
      name_string += "/";
    }
    name_string += street_name.name();
  }
  return name_string;
}

}
}
