#include <limits>

#include "baldr/graphid.h"
#include "baldr/graphconstants.h"

namespace valhalla {
namespace baldr {

// Constructor with values for each field of the GraphId.
GraphId::GraphId(const uint32_t tileid, const uint32_t level,
                 const uint32_t id) {
  Set(tileid, level, id);
}

// Set the fields of the GraphId
void GraphId::Set(const uint32_t tileid, const uint32_t level,
                  const uint32_t id) {
  if(tileid <= kMaxGraphTileId) fields.tileid = tileid;
  else throw std::logic_error("Tile id out of valid range");
  if(level <= kMaxGraphHierarchy) fields.level = level;
  else throw std::logic_error("Level out of valid range");
  if(id <= kMaxGraphId) fields.id = id;
  else throw std::logic_error("Id out of valid range");
  fields.spare = 0;
}

// The json representation of the Id
json::Value GraphId::json() const {
  if(Is_Valid())
    return json::map({
      {"level", static_cast<uint64_t>(fields.level)},
      {"tile_id", static_cast<uint64_t>(fields.tileid)},
      {"id", static_cast<uint64_t>(fields.id)},
    });
  return static_cast<std::nullptr_t>(nullptr);
}

// Stream output
std::ostream& operator<<(std::ostream& os, const GraphId& id) {
  return os << id.fields.level << '/' << id.fields.tileid << '/' << id.fields.id;
}

}
}
