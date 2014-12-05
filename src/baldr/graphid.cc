#include "baldr/graphid.h"

namespace {

// Maximum of 8 (0-7) graph hierarchies are supported.
constexpr unsigned int kMaxGraphHierarchy = 7;

// Maximum unique identifier within a graph hierarchy (~536 million)
constexpr unsigned int kMaxGraphId = 536870911;

}

namespace valhalla{
namespace baldr{

  GraphId::GraphId() {
    // TODO - better way to initialize?
    Set(0, 0);
  }

  GraphId::GraphId(const unsigned int level, const unsigned int id) {
    Set(level, id);
  }

  GraphId::GraphId(const GraphId& g) {
    Set(g.Level(), g.Id());
  }

  unsigned int GraphId::Level() const {
    return graphid_.level;
  }

  unsigned int GraphId::Id() const {
    return graphid_.id;
  }

  bool GraphId::IsValid() const {
    return (graphid_.id > 0);
  }

  void GraphId::Set(const unsigned int level, const unsigned int id) {
    graphid_.id = (id < kMaxGraphId ) ? id : 0;
    graphid_.level = (level < kMaxGraphHierarchy) ? level : 0;
  }

  /**
   * Post increments the id.
   */
  void GraphId::operator ++(int) {
    graphid_.id++;
  }

  // TODO - could this be simplified using unions in the struct?
  bool GraphId::operator < (const GraphId& other) const {
    if (Level() == other.Level()) {
      return Id() < other.Id();
    }
    return Level() < other.Level();
  }

}
}
