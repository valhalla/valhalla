#include <iostream>

#include "baldr/streetname.h"

namespace valhalla {
namespace baldr {

StreetName::StreetName(const std::string& value, const bool is_route_number)
    : value_(value), is_route_number_(is_route_number) {
}

StreetName::~StreetName() {
}

const std::string& StreetName::value() const {
  return value_;
}

bool StreetName::is_route_number() const {
  return is_route_number_;
}

bool StreetName::operator==(const StreetName& rhs) const {
  return ((value_ == rhs.value_) && (is_route_number_ == rhs.is_route_number_));
}

bool StreetName::StartsWith(const std::string& prefix) const {
  size_t n = prefix.size();
  return (value_.size() < n) ? false : prefix == value_.substr(0, n);
}

bool StreetName::EndsWith(const std::string& suffix) const {
  size_t n = suffix.size();
  return (value_.size() < n) ? false : suffix == value_.substr(value_.size() - n);
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

  return value_.substr(pre_dir.size(), (value_.size() - pre_dir.size() - post_dir.size()));
}

bool StreetName::HasSameBaseName(const StreetName& rhs) const {
  return (GetBaseName() == rhs.GetBaseName());
}

} // namespace baldr
} // namespace valhalla
