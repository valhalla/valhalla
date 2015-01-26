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
  graphid_.v = kInvalidId;
}

// Constructor with values for each field of the GraphId.
GraphId::GraphId(const uint32_t tileid, const uint32_t level,
                 const uint64_t id) {
  Set(tileid, level, id);
}

// Copy constructor
GraphId::GraphId(const GraphId& g) {
  graphid_.v = g.graphid_.v;
}

// Get a 64 bit value of the composite GraphId
uint64_t GraphId::value() const {
  return graphid_.v;
}

// Get the tile Id
uint32_t GraphId::tileid() const {
  return graphid_.fields.tileid;
}

// Get the hierarchy level
uint32_t GraphId::level() const {
  return graphid_.fields.level;
}

// Get the Id
uint64_t GraphId::id() const {
  return graphid_.fields.id;
}

// Set the fields of the GraphId
void GraphId::Set(const uint32_t tileid, const uint32_t level,
                  const uint64_t id) {
  graphid_.fields.tileid = (tileid < kMaxGraphTileId) ? tileid : 0;
  graphid_.fields.level = (level < kMaxGraphHierarchy) ? level : 0;
  graphid_.fields.id = (id < kMaxGraphId) ? id : 0;
}

bool GraphId::Is_Valid() const {
  return graphid_.v != kInvalidId;
}

GraphId GraphId::Tile_Base() const {
  return GraphId( graphid_.fields.tileid,  graphid_.fields.level, 0);
}

// Post increments the id.
void GraphId::operator ++(int) {
  graphid_.fields.id++;
}

// Comparison for sorting
bool GraphId::operator <(const GraphId& rhs) const {
  return graphid_.v < rhs.graphid_.v;
}

// Equality operator
bool GraphId::operator ==(const GraphId& rhs) const {
  return graphid_.v == rhs.graphid_.v;
}

std::ostream& operator<<(std::ostream& os, const GraphId& id)
{
    return os << id.level() << '/' << id.tileid() << '/' << id.id();
}

// Get the hash_value
const uint64_t GraphId::hash_value() {

  uint64_t seed = 0;

  graphid_.fields.id = ~graphid_.fields.id;
  boost::hash_combine(seed,ffs(graphid_.fields.id+1)-1);
  graphid_.fields.level = ~graphid_.fields.level;
  boost::hash_combine(seed,ffs(graphid_.fields.level+1)-1);
  graphid_.fields.tileid = ~graphid_.fields.tileid;
  boost::hash_combine(seed,ffs(graphid_.fields.tileid+1)-1);

  boost::hash_combine(seed,sizeof(GraphId));

  return seed;

}

}
}
