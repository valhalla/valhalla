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

  /**
   * Constructor.
   * @param  hierarchy   hierarchy ID
   * @param  id         Unique identifier within the hierarchy.
   */
  GraphId::GraphId(const unsigned int hierarchy, const unsigned int id) {
    Set(hierarchy, id);
  }

  /**
   * Copy constructor.
   * @param  g   GraphId to copy
   */
  GraphId::GraphId(const GraphId& g) {
    Set(g.Hierarchy(), g.Id());
  }

  /**
   * Gets the hierarchy number.
   * @return   Returns the hierarchy.
   */
  unsigned int GraphId::Hierarchy() const {
    return graphid.hierarchy;
  }

  /**
   * Gets the identifier within the hierarchy.
   * @return   Returns the unique identifier within the hierarchy.
   */
  unsigned int GraphId::Id() const {
    return graphid.id;
  }

  /**
   * Test if this is a valid graph element. Invalid elements have id == 0.
   * @return   Returns true if valid, false if not.
   */
  bool GraphId::IsValid() const {
    return (graphid.id > 0);
  }

  /**
   * Convenience method to set individual greph Id elements.
   * @param  hierarchy  hierarchy ID
   * @param  id         Unique identifier within the hierarchy
   */
  void GraphId::Set(const unsigned int hierarchy, const unsigned int id) {
    graphid.id = (id < kMaxGraphId ) ? id : 0;
    graphid.hierarchy = (hierarchy < kMaxGraphHierarchy) ? hierarchy : 0;
  }

  /**
   * Post increments the id.
   */
  void GraphId::operator ++(int) {
    graphid.id++;
  }

}
}
