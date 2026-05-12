#include "baldr/rapidjson_utils.h"
#include "tyr/serializers.h"

#include <cstdint>

using namespace valhalla;
using namespace valhalla::baldr;

namespace {

void serialize(rapidjson::writer_wrapper_t& writer, const Location& location) {
  // serialze all the edges
  writer.set_precision(tyr::kCoordinatePrecision);
  writer.start_object();
  writer("input_lat", location.ll().lat());
  writer("input_lon", location.ll().lng());
  writer("radius", static_cast<uint64_t>(location.radius()));
  writer("istransit", location.transit_available());
  writer.end_object();
}
} // namespace

namespace valhalla {
namespace tyr {

std::string serializeTransitAvailable(const Api& request) {
  rapidjson::writer_wrapper_t writer(4096);
  writer.start_array();
  for (const auto& location : request.options().locations()) {
    serialize(writer, location);
  }
  writer.end_array();
  return writer.get_buffer();
}

} // namespace tyr
} // namespace valhalla
