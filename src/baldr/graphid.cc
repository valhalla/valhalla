#include <limits>

#include "baldr/graphid.h"
#include <boost/functional/hash.hpp>

namespace {
// Invalid graphid.
constexpr uint32_t kInvalidId = std::numeric_limits<uint32_t>::max();

// Maximum number of tiles supported.
constexpr uint32_t kMaxGraphTileId = 16777215;

// Maximum of 8 (0-7) graph hierarchies are supported.
constexpr uint32_t kMaxGraphHierarchy = 7;

// Maximum unique identifier within a graph hierarchy.
constexpr uint64_t kMaxGraphId = 68719476735;
}

namespace valhalla {
namespace baldr {

// Default constructor
GraphId::GraphId() {
  value = kInvalidId;
}

// Constructor with values for each field of the GraphId.
GraphId::GraphId(const uint32_t tileid, const uint32_t level,
                 const uint64_t id) {
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
uint64_t GraphId::id() const {
  return fields.id;
}

// Set the fields of the GraphId
void GraphId::Set(const uint32_t tileid, const uint32_t level,
                  const uint64_t id) {
  fields.tileid = (tileid < kMaxGraphTileId) ? tileid : 0;
  fields.level = (level < kMaxGraphHierarchy) ? level : 0;
  fields.id = (id < kMaxGraphId) ? id : 0;
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

// Get the internal version
const uint64_t GraphId::internal_version() {

  GraphId id{};

  size_t seed = 0;

  id.fields.id = ~id.fields.id;
  boost::hash_combine(seed,ffs(id.fields.id+1)-1);
  id.fields.level = ~id.fields.level;
  boost::hash_combine(seed,ffs(id.fields.level+1)-1);
  id.fields.tileid = ~id.fields.tileid;
  boost::hash_combine(seed,ffs(id.fields.tileid+1)-1);

  boost::hash_combine(seed,sizeof(GraphId));

  return seed;

}

}
}
