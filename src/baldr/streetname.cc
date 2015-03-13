#include <iostream>

#include "baldr/streetname.h"

#include <boost/algorithm/string/predicate.hpp>

namespace valhalla {
namespace baldr {

// TODO - locale
const std::vector<std::string> StreetName::pre_dirs_ { "North ", "East ",
    "South ", "West ", "Northeast ", "Southeast ", "Southwest ", "Northwest " };
const std::vector<std::string> StreetName::post_dirs_ { " North", " East",
    " South", " West", " Northeast", " Southeast", " Southwest", " Northwest" };
const std::vector<std::string> StreetName::post_cardinal_dirs_ { " North",
    " East", " South", " West" };

StreetName::StreetName(const std::string& value)
    : value_(value) {
}

const std::string& StreetName::value() const {
  return value_;
}

bool StreetName::operator ==(const StreetName& rhs) const {
  return (value_ == rhs.value_);
}

bool StreetName::StartsWith(const std::string& prefix) const {
  return boost::algorithm::starts_with(value_, prefix);
}

bool StreetName::EndsWith(const std::string& suffix) const {
  return boost::algorithm::ends_with(value_, suffix);
}

std::string StreetName::GetPreDir() const {
  for (const auto& pre_dir : StreetName::pre_dirs_) {
    if (StartsWith(pre_dir))
      return pre_dir;
  }
  return "";
}

std::string StreetName::GetPostDir() const {
  for (const auto& post_dir : StreetName::post_dirs_) {
    if (EndsWith(post_dir))
      return post_dir;
  }
  return "";
}

std::string StreetName::GetPostCardinalDir() const {
  for (const auto& post_cardinal_dir : StreetName::post_cardinal_dirs_) {
    if (EndsWith(post_cardinal_dir))
      return post_cardinal_dir;
  }
  return "";
}

std::string StreetName::GetBaseName() const {
  std::string pre_dir = GetPreDir();
  std::string post_dir = GetPostDir();

  return value_.substr(pre_dir.size(),
                       (value_.size() - pre_dir.size() - post_dir.size()));
}

bool StreetName::HasSameBaseName(const StreetName& rhs) const {
  return (GetBaseName() == rhs.GetBaseName());
}

}
}
