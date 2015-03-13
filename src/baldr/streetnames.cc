#include <iostream>

#include "baldr/streetnames.h"

namespace valhalla {
namespace baldr {

StreetNames::StreetNames()
    : std::list<StreetName>() {
}

StreetNames::StreetNames(const std::vector<std::string>& names)
    : std::list<StreetName>() {
  for (const auto& name : names) {
    this->emplace_back(name);
  }

}

StreetNames::~StreetNames() {
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
    name_string += street_name.value();
    name_string += "\"";
  }
  name_string += " }";
  return name_string;
}

StreetNames StreetNames::FindCommonStreetNames(
    const StreetNames& other_street_names) const {
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

StreetNames StreetNames::FindCommonBaseNames(
    const StreetNames& other_street_names) const {
  StreetNames common_base_names;
  for (const auto& street_name : *this) {
    for (const auto& other_street_name : other_street_names) {
      if (street_name.HasSameBaseName(other_street_name)) {
        // Use the name with the cardinal directional suffix
        // thus, 'US 30 West' will be used instead of 'US 30'
        if (!street_name.GetPostCardinalDir().empty())
          common_base_names.push_back(street_name);
        else if (!other_street_name.GetPostCardinalDir().empty())
          common_base_names.push_back(other_street_name);
        // Use street_name by default
        else
          common_base_names.push_back(street_name);
        break;
      }
    }
  }

  return common_base_names;
}

}
}
