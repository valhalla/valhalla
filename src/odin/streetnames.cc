#include <iostream>

#include "odin/streetnames.h"

namespace valhalla {
namespace odin {

StreetNames::StreetNames()
    : std::list<StreetName>() {
}

StreetNames::StreetNames(
    const ::google::protobuf::RepeatedPtrField<::std::string>& names)
    : std::list<StreetName>() {
  for (const auto& name : names) {
    this->emplace_back(name);
  }

}

std::string StreetNames::ToString() const {
  std::string name_string;
  if (empty())
    name_string = "unnamed";
  for (auto& street_name : *this) {
    if (!name_string.empty()) {
      name_string += "/";
    }
    name_string += street_name.value();
  }
  return name_string;
}

StreetNames StreetNames::FindCommonStreetNames(StreetNames other_street_names) const {
  StreetNames common_street_names;
  for (const auto& street_name : *this) {
    for (const auto& other_street_name : other_street_names) {
      if (street_name == other_street_name) {
        common_street_names.push_back(street_name);
        break;
      }
    }
  }

  return common_street_names;
}

}
}
