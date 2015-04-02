#include <iostream>
#include <vector>

#include "baldr/streetnames.h"

namespace valhalla {
namespace baldr {

StreetNames::StreetNames()
    : std::list<std::unique_ptr<StreetName>>() {
}

StreetNames::~StreetNames() {
}

std::string StreetNames::ToString() const {
  std::string name_string;
  if (this->empty())
    name_string = "unnamed";
  for (auto& street_name : *this) {
    if (!name_string.empty()) {
      name_string += "/";
    }
    name_string += street_name->value();
  }
  return name_string;
}

std::string StreetNames::ToParameterString() const {
  std::string name_string;
  bool is_first = true;
  name_string += "{ ";
  for (auto& street_name : *this) {
    if (is_first)
      is_first = false;
    else
      name_string += ", ";
    name_string += "\"";
    name_string += street_name->value();
    name_string += "\"";
  }
  name_string += " }";
  return name_string;
}

}
}
