#include "baldr/streetname_us.h"
#include "baldr/streetname.h"

namespace valhalla {
namespace baldr {

const std::vector<std::string> StreetNameUs::pre_dirs_{"North ",     "East ",      "South ",
                                                       "West ",      "Northeast ", "Southeast ",
                                                       "Southwest ", "Northwest "};
const std::vector<std::string> StreetNameUs::post_dirs_{" North",     " East",      " South",
                                                        " West",      " Northeast", " Southeast",
                                                        " Southwest", " Northwest"};
const std::vector<std::string> StreetNameUs::post_cardinal_dirs_{" North", " East", " South",
                                                                 " West"};

StreetNameUs::StreetNameUs(const std::string& value,
                           const bool is_route_number,
                           const std::optional<baldr::Pronunciation>& pronunciation)
    : StreetName(value, is_route_number, pronunciation) {
}

std::string StreetNameUs::GetPreDir() const {
  for (const auto& pre_dir : StreetNameUs::pre_dirs_) {
    if (StartsWith(pre_dir)) {
      return pre_dir;
    }
  }
  return "";
}

std::string StreetNameUs::GetPostDir() const {
  for (const auto& post_dir : StreetNameUs::post_dirs_) {
    if (EndsWith(post_dir)) {
      return post_dir;
    }
  }
  return "";
}

std::string StreetNameUs::GetPostCardinalDir() const {
  for (const auto& post_cardinal_dir : StreetNameUs::post_cardinal_dirs_) {
    if (EndsWith(post_cardinal_dir)) {
      return post_cardinal_dir;
    }
  }
  return "";
}

std::string StreetNameUs::GetBaseName() const {
  std::string pre_dir = GetPreDir();
  std::string post_dir = GetPostDir();

  return value_.substr(pre_dir.size(), (value_.size() - pre_dir.size() - post_dir.size()));
}

bool StreetNameUs::HasSameBaseName(const StreetName& rhs) const {
  return (GetBaseName() == rhs.GetBaseName());
}

} // namespace baldr
} // namespace valhalla
