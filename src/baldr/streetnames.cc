#include <iostream>
#include <vector>

#include "baldr/streetnames.h"
#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

StreetNames::StreetNames() : std::list<std::unique_ptr<StreetName>>() {
}

StreetNames::StreetNames(const std::vector<std::string>& names) {
  for (auto& name : names) {
    this->emplace_back(midgard::make_unique<StreetName>(name));
  }
}

StreetNames::~StreetNames() {
}

std::string StreetNames::ToString(uint32_t max_count,
                                  const std::string& delim,
                                  const VerbalTextFormatter* verbal_formatter) const {
  std::string name_string;
  uint32_t count = 0;
  if (this->empty()) {
    name_string = "unnamed";
  }
  for (auto& street_name : *this) {
    // If supplied, limit by max count
    if ((max_count > 0) && (count == max_count)) {
      break;
    }
    if (!name_string.empty()) {
      name_string += delim;
    }
    name_string +=
        (verbal_formatter) ? verbal_formatter->Format(street_name->value()) : street_name->value();
    ++count;
  }
  return name_string;
}

#ifdef LOGGING_LEVEL_TRACE
std::string StreetNames::ToParameterString() const {
  std::string name_string;
  bool is_first = true;
  name_string += "{ ";
  for (auto& street_name : *this) {
    if (is_first) {
      is_first = false;
    } else {
      name_string += ", ";
    }
    name_string += "\"";
    name_string += street_name->value();
    name_string += "\"";
  }
  name_string += " }";
  return name_string;
}
#endif

std::unique_ptr<StreetNames> StreetNames::clone() const {
  std::unique_ptr<StreetNames> clone_street_names = midgard::make_unique<StreetNames>();
  for (const auto& street_name : *this) {
    clone_street_names->emplace_back(midgard::make_unique<StreetName>(street_name->value()));
  }

  return clone_street_names;
}

std::unique_ptr<StreetNames>
StreetNames::FindCommonStreetNames(const StreetNames& other_street_names) const {
  std::unique_ptr<StreetNames> common_street_names = midgard::make_unique<StreetNames>();
  for (const auto& street_name : *this) {
    for (const auto& other_street_name : other_street_names) {
      if (*street_name == *other_street_name) {
        common_street_names->emplace_back(midgard::make_unique<StreetName>(street_name->value()));
        break;
      }
    }
  }

  return common_street_names;
}

std::unique_ptr<StreetNames>
StreetNames::FindCommonBaseNames(const StreetNames& other_street_names) const {
  std::unique_ptr<StreetNames> common_base_names = midgard::make_unique<StreetNames>();
  for (const auto& street_name : *this) {
    for (const auto& other_street_name : other_street_names) {
      if (street_name->HasSameBaseName(*other_street_name)) {
        // Use the name with the cardinal directional suffix
        // thus, 'US 30 West' will be used instead of 'US 30'
        if (!street_name->GetPostCardinalDir().empty()) {
          common_base_names->emplace_back(midgard::make_unique<StreetName>(street_name->value()));
        } else if (!other_street_name->GetPostCardinalDir().empty()) {
          common_base_names->emplace_back(
              midgard::make_unique<StreetName>(other_street_name->value()));
          // Use street_name by default
        } else {
          common_base_names->emplace_back(midgard::make_unique<StreetName>(street_name->value()));
        }
        break;
      }
    }
  }

  return common_base_names;
}

} // namespace baldr
} // namespace valhalla
