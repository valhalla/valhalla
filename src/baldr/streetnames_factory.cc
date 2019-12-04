#include <iostream>
#include <utility>
#include <vector>

#include "baldr/streetnames.h"
#include "baldr/streetnames_factory.h"
#include "baldr/streetnames_us.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

std::unique_ptr<StreetNames>
StreetNamesFactory::Create(const std::string& country_code,
                           const std::vector<std::pair<std::string, bool>>& names) {
  if (country_code == "US") {
    return std::make_unique<StreetNamesUs>(names);
  }

  return std::make_unique<StreetNames>(names);
}

} // namespace baldr
} // namespace valhalla
