#include "baldr/graphid.h"

namespace valhalla {
namespace baldr {

// The json representation of the Id
// used in valhalla/mjolnir/util.h
json::Value GraphId::json() const {
  if (Is_Valid()) {
    return json::map({
        {"level", static_cast<uint64_t>(level())},
        {"tile_id", static_cast<uint64_t>(tileid())},
        {"id", static_cast<uint64_t>(id())},
        {"value", value},
    });
  }
  return static_cast<std::nullptr_t>(nullptr);
}

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
