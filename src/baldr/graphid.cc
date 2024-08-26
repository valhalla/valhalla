#include "baldr/graphid.h"

namespace valhalla {
namespace baldr {

// The json representation of the Id
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

// Stream output
std::ostream& operator<<(std::ostream& os, const GraphId& id) {
  return os << std::to_string(id);
}

} // namespace baldr
} // namespace valhalla
