#include <iostream>

#include "baldr/streetname.h"

#include <boost/algorithm/string/predicate.hpp>

namespace valhalla {
namespace baldr {

StreetName::StreetName(const std::string& value)
    : value_(value) {
}

StreetName::~StreetName() {
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
  return "";
}

std::string StreetName::GetPostDir() const {
  return "";
}

std::string StreetName::GetPostCardinalDir() const {
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
