#include "baldr/pathlocation.h"
#include "baldr/rapidjson_utils.h"
#include "tyr/serializers.h"

#include <cstdint>
#include <unordered_set>

using namespace valhalla;
using namespace valhalla::baldr;

namespace {

void serialize(rapidjson::writer_wrapper_t& writer, const PathLocation& location, bool istransit) {
  // serialze all the edges
  writer.set_precision(tyr::kCoordinatePrecision);
  writer.start_object();
  writer("input_lat", location.latlng_.lat());
  writer("input_lon", location.latlng_.lng());
  writer("radius", static_cast<uint64_t>(location.radius_));
  writer("istransit", istransit);
  writer.end_object();
}
} // namespace

namespace valhalla {
namespace tyr {

std::string serializeTransitAvailable(const Api& /* request */,
                                      const std::vector<baldr::Location>& locations,
                                      const std::unordered_set<baldr::Location>& found) {
  rapidjson::writer_wrapper_t writer(4096);
  writer.start_array();
  for (const auto& location : locations) {
    serialize(writer, location, found.find(location) != found.cend());
  }
  writer.end_array();
  return writer.get_buffer();
}

} // namespace tyr
} // namespace valhalla
