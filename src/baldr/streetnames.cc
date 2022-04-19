#include <iostream>
#include <vector>

#include <boost/optional.hpp>

#include "baldr/streetname.h"
#include "baldr/streetnames.h"
#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"
#include "midgard/util.h"
#include "proto/common.pb.h"

namespace valhalla {
namespace baldr {

StreetNames::StreetNames() : std::list<std::unique_ptr<StreetName>>() {
}

StreetNames::StreetNames(const std::vector<std::pair<std::string, bool>>& names) {
  for (auto& name : names) {
    this->emplace_back(std::make_unique<StreetName>(name.first, name.second, boost::none));
  }
}

StreetNames::StreetNames(const google::protobuf::RepeatedPtrField<valhalla::StreetName>& names) {
  for (auto& name : names) {
    boost::optional<baldr::Pronunciation> pronunciation =
        boost::make_optional(name.has_pronunciation(),
                             baldr::Pronunciation{name.pronunciation().alphabet(),
                                                  name.pronunciation().value()});
    this->emplace_back(
        std::make_unique<StreetName>(name.value(), name.is_route_number(), pronunciation));
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
    name_string += (verbal_formatter) ? verbal_formatter->Format(street_name) : street_name->value();
    if (street_name->pronunciation()) {
      name_string += "(";
      name_string += street_name->pronunciation()->value;
      name_string += ")";
    }
    ++count;
  }
  return name_string;
}

#ifdef LOGGING_LEVEL_TRACE
std::string StreetNames::ToParameterString() const {
  std::string name_string;
  std::string param_list;

  name_string += "{ ";
  for (const auto& street_name : *this) {
    if (!param_list.empty()) {
      param_list += ", ";
    }
    param_list += "{ \"";
    param_list += street_name->value();
    param_list += "\", ";
    param_list += std::to_string(street_name->is_route_number());
    param_list += " }";
  }
  name_string += param_list;
  name_string += " }";

  return name_string;
}
#endif

std::unique_ptr<StreetNames> StreetNames::clone() const {
  std::unique_ptr<StreetNames> clone_street_names = std::make_unique<StreetNames>();
  for (const auto& street_name : *this) {
    clone_street_names->emplace_back(std::make_unique<StreetName>(street_name->value(),
                                                                  street_name->is_route_number(),
                                                                  street_name->pronunciation()));
  }

  return clone_street_names;
}

std::unique_ptr<StreetNames>
StreetNames::FindCommonStreetNames(const StreetNames& other_street_names) const {
  std::unique_ptr<StreetNames> common_street_names = std::make_unique<StreetNames>();
  for (const auto& street_name : *this) {
    for (const auto& other_street_name : other_street_names) {
      if (*street_name == *other_street_name) {
        common_street_names->emplace_back(std::make_unique<StreetName>(street_name->value(),
                                                                       street_name->is_route_number(),
                                                                       street_name->pronunciation()));
        break;
      }
    }
  }

  return common_street_names;
}

std::unique_ptr<StreetNames>
StreetNames::FindCommonBaseNames(const StreetNames& other_street_names) const {
  std::unique_ptr<StreetNames> common_base_names = std::make_unique<StreetNames>();
  for (const auto& street_name : *this) {
    for (const auto& other_street_name : other_street_names) {
      if (street_name->HasSameBaseName(*other_street_name)) {
        // Use the name with the cardinal directional suffix
        // thus, 'US 30 West' will be used instead of 'US 30'
        if (!street_name->GetPostCardinalDir().empty()) {
          common_base_names->emplace_back(std::make_unique<StreetName>(street_name->value(),
                                                                       street_name->is_route_number(),
                                                                       street_name->pronunciation()));
        } else if (!other_street_name->GetPostCardinalDir().empty()) {
          common_base_names->emplace_back(
              std::make_unique<StreetName>(other_street_name->value(),
                                           other_street_name->is_route_number(),
                                           other_street_name->pronunciation()));
          // Use street_name by default
        } else {
          common_base_names->emplace_back(std::make_unique<StreetName>(street_name->value(),
                                                                       street_name->is_route_number(),
                                                                       street_name->pronunciation()));
        }
        break;
      }
    }
  }

  return common_base_names;
}

std::unique_ptr<StreetNames> StreetNames::GetRouteNumbers() const {
  std::unique_ptr<StreetNames> route_numbers = std::make_unique<StreetNames>();
  for (const auto& street_name : *this) {
    if (street_name->is_route_number()) {
      route_numbers->emplace_back(std::make_unique<StreetName>(street_name->value(),
                                                               street_name->is_route_number(),
                                                               street_name->pronunciation()));
    }
  }

  return route_numbers;
}

std::unique_ptr<StreetNames> StreetNames::GetNonRouteNumbers() const {
  std::unique_ptr<StreetNames> non_route_numbers = std::make_unique<StreetNames>();
  for (const auto& street_name : *this) {
    if (!street_name->is_route_number()) {
      non_route_numbers->emplace_back(std::make_unique<StreetName>(street_name->value(),
                                                                   street_name->is_route_number(),
                                                                   street_name->pronunciation()));
    }
  }

  return non_route_numbers;
}

} // namespace baldr
} // namespace valhalla
