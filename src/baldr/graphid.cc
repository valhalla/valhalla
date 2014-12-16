#include <functional>

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
  Set(0, 0, 0);
}

GraphId::GraphId(const unsigned int tileid, const unsigned int level,
                 const unsigned int id) {
  Set(tileid, level, id);
}

GraphId::GraphId(const GraphId& g) {
  Set(g.tileid(), g.level(), g.id());
}

unsigned int GraphId::tileid() const {
  return graphid_.tileid;
}

unsigned int GraphId::level() const {
  return graphid_.level;
}

unsigned int GraphId::id() const {
  return graphid_.id;
}

bool GraphId::IsValid() const {
  return (graphid_.id > 0);
}

void GraphId::Set(const unsigned int tileid, const unsigned int level,
                  const unsigned int id) {
  graphid_.tileid = (tileid < kMaxGraphTileId) ? tileid : 0;
  graphid_.level = (level < kMaxGraphHierarchy) ? level : 0;
  graphid_.id = (id < kMaxGraphId) ? id : 0;
}

/**
 * Post increments the id.
 */
void GraphId::operator ++(int) {
  graphid_.id++;
}

// TODO - could this be simplified using unions in the struct?
bool GraphId::operator <(const GraphId& other) const {
  if (level() == other.level()) {
    return id() < other.id();
  }
  return level() < other.level();
}

bool GraphId::operator ==(const GraphId& rhs) const {
  return (tileid() == rhs.tileid() && level() == rhs.level() && id() == rhs.id());
}

std::size_t GraphId::HashCode() const {
  return (std::hash<uint64_t>()(graphid_.tileid)
      ^ std::hash<uint64_t>()(graphid_.level)
      ^ std::hash<uint64_t>()(graphid_.id));
}
}
}
