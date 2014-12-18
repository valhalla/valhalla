#include <functional>
#include <boost/functional/hash.hpp>

#include "baldr/graphid.h"

namespace {

constexpr unsigned int kMaxGraphTileId = 16777215;

// Maximum of 8 (0-7) graph hierarchies are supported.
constexpr unsigned int kMaxGraphHierarchy = 7;

// Maximum unique identifier within a graph hierarchy.
constexpr uint64_t kMaxGraphId = 68719476735;

}

namespace valhalla {
namespace baldr {

GraphId::GraphId() {
  graphid_.v = 0;
}

GraphId::GraphId(const unsigned int tileid, const unsigned int level,
                 const unsigned int id) {
  Set(tileid, level, id);
}

GraphId::GraphId(const GraphId& g) {
  Set(g.tileid(), g.level(), g.id());
}

uint64_t GraphId::value() const {
  return graphid_.v;
}

unsigned int GraphId::tileid() const {
  return graphid_.fields.tileid;
}

unsigned int GraphId::level() const {
  return graphid_.fields.level;
}

unsigned int GraphId::id() const {
  return graphid_.fields.id;
}

void GraphId::Set(const unsigned int tileid, const unsigned int level,
                  const unsigned int id) {
  graphid_.fields.tileid = (tileid < kMaxGraphTileId) ? tileid : 0;
  graphid_.fields.level = (level < kMaxGraphHierarchy) ? level : 0;
  graphid_.fields.id = (id < kMaxGraphId) ? id : 0;
}

/**
 * Post increments the id.
 */
void GraphId::operator ++(int) {
  graphid_.fields.id++;
}

// TODO - could this be simplified using unions in the struct?
bool GraphId::operator <(const GraphId& rhs) const {
  return graphid_.v < rhs.graphid_.v;
}

bool GraphId::operator ==(const GraphId& rhs) const {
  return graphid_.v == rhs.graphid_.v;
}

std::size_t GraphId::HashCode() const {
  std::size_t seed = 13;
  boost::hash_combine(seed, graphid_.fields.tileid);
  boost::hash_combine(seed, graphid_.fields.level);
  boost::hash_combine(seed, graphid_.fields.id);

  return seed;
}

}
}
