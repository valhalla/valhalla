#include "baldr/graphid.h"
#include "baldr/rapidjson_utils.h"

namespace valhalla {
namespace baldr {

// the json representation of the Id
void GraphId::json(rapidjson::writer_wrapper_t& writer) const {
  if (Is_Valid()) {
    writer("level", static_cast<uint64_t>(level()));
    writer("tile_id", static_cast<uint64_t>(tileid()));
    writer("id", static_cast<uint64_t>(id()));
    writer("value", value);
  }
}

// Stream output
std::ostream& operator<<(std::ostream& os, const GraphId& id) {
  return os << std::to_string(id);
}

} // namespace baldr
} // namespace valhalla
