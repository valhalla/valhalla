#include <cstdint>
#include <unordered_set>

#include "baldr/json.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace {

json::MapPtr serialize(const PathLocation& location, bool istransit) {
  // serialze all the edges
  auto json = json::map({{"input_lat", json::fixed_t{location.latlng_.lat(), 6}},
                         {"input_lon", json::fixed_t{location.latlng_.lng(), 6}},
                         {"radius", static_cast<uint64_t>(location.radius_)}});
  json->emplace("istransit", istransit);
  return json;
}
} // namespace

namespace valhalla {
namespace tyr {

std::string serializeTransitAvailable(const Api& /* request */,
                                      const std::vector<baldr::Location>& locations,
                                      const std::unordered_set<baldr::Location>& found) {
  auto json = json::array({});
  for (const auto& location : locations) {
    json->emplace_back(serialize(location, found.find(location) != found.cend()));
  }
  std::stringstream ss;
  ss << *json;
  return ss.str();
}

} // namespace tyr
} // namespace valhalla
