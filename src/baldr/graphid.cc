#include <limits>

#include "baldr/graphid.h"
#include "baldr/graphconstants.h"

namespace {
// Invalid graphid.
constexpr uint32_t kInvalidId = std::numeric_limits<uint32_t>::max();
}

namespace valhalla {
namespace baldr {

// Default constructor
GraphId::GraphId() {
  value = kInvalidId;
}

// Constructor with values for each field of the GraphId.
GraphId::GraphId(const uint32_t tileid, const uint32_t level,
                 const uint32_t id) {
  Set(tileid, level, id);
}

GraphId::GraphId(const uint64_t value): value(value){
}

// Get the tile Id
uint32_t GraphId::tileid() const {
  return fields.tileid;
}

// Get the hierarchy level
uint32_t GraphId::level() const {
  return fields.level;
}

// Get the Id
uint32_t GraphId::id() const {
  return fields.id;
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

bool GraphId::Is_Valid() const {
  return value != kInvalidId;
}

GraphId GraphId::Tile_Base() const {
  return GraphId( fields.tileid,  fields.level, 0);
}

json::Value GraphId::json() const {
  if(Is_Valid())
    return json::map({
      {"level", static_cast<uint64_t>(fields.level)},
      {"tile_id", static_cast<uint64_t>(fields.tileid)},
      {"id", static_cast<uint64_t>(fields.id)},
    });
  return static_cast<std::nullptr_t>(nullptr);
}

// Post increments the id.
void GraphId::operator ++(int) {
  fields.id++;
}

// Advance the id.
GraphId GraphId::operator+(uint64_t offset) const {
  return GraphId(fields.tileid, fields.level, fields.id + offset);
}

// Comparison for sorting
bool GraphId::operator <(const GraphId& rhs) const {
  return value < rhs.value;
}

// Equality operator
bool GraphId::operator ==(const GraphId& rhs) const {
  return value == rhs.value;
}

bool GraphId::operator !=(const GraphId& rhs) const {
  return value != rhs.value;
}

GraphId::operator uint64_t() const {
  return value;
}

std::ostream& operator<<(std::ostream& os, const GraphId& id)
{
    return os << id.fields.level << '/' << id.fields.tileid << '/' << id.fields.id;
}

}
}
