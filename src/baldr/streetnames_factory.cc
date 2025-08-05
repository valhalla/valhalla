#include "baldr/streetnames_factory.h"
#include "baldr/streetnames.h"
#include "baldr/streetnames_us.h"
#include "proto/common.pb.h"

#include <utility>
#include <vector>

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

std::unique_ptr<StreetNames>
StreetNamesFactory::Create(const std::string& country_code,
                           const google::protobuf::RepeatedPtrField<valhalla::StreetName>& names) {
  if (country_code == "US") {
    return std::make_unique<StreetNamesUs>(names);
  }

  return std::make_unique<StreetNames>(names);
}

} // namespace baldr
} // namespace valhalla
