#include <iostream>
#include <vector>

#include "baldr/streetnames.h"
#include "baldr/streetnames_us.h"
#include "baldr/streetnames_factory.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

std::unique_ptr<StreetNames> StreetNamesFactory::Create(
    const std::string& country_code, const std::vector<std::string>& names) {
  if (country_code == "US") {
    return midgard::make_unique<StreetNamesUs>(names);
  }

  return midgard::make_unique<StreetNames>(names);
}

}
}
