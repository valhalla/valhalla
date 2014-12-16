#ifndef VALHALLA_BALDR_GRAPHID_H_
#define VALHALLA_BALDR_GRAPHID_H_

#include <cstdint>

namespace valhalla {
namespace baldr {

/**
 * Identifier of a node or an edge within the tiled, hierarchical graph.
 * Includes the tile Id, hierarchy level and a unique identifier within the
 * level.
 * @author  David W. Nesbitt
 */
class GraphId {
 public:
  /**public:
   * Default constructor
   */
  GraphId();

  /**
   * Constructor.
   * @param  tileid Tile Id.
   * @param  level  Hierarchy level
   * @param  id     Unique identifier within the level.
   */
  GraphId(const unsigned int tileid, const unsigned int level,
          const unsigned int id);

  /**
   * Copy constructor.
   * @param  g   GraphId to copy
   */
  GraphId(const GraphId& g);

  /**
   * Gets the tile Id.
   * @return   Returns the tile Id.
   */
  unsigned int tileid() const;

  /**
   * Gets the hierarchy level.
   * @return   Returns the level.
   */
  unsigned int level() const;

  /**
   * Gets the identifier within the hierarchy level.
   * @return   Returns the unique identifier within the level.
   */
  unsigned int id() const;

  /**
   * Convenience method to set individual graph Id elements.
   * @param  tileid Tile Id.
   * @param  level  Hierarchy level
   * @param  id     Unique identifier within the level
   */
  void Set(const unsigned int tileid, const unsigned int level,
           const unsigned int id);

  /**
   * Post increments the id.
   */
  void operator ++(int);

  /**
   * Less than operator for sorting.
   * @param  other  Other graph Id for comparison.
   * @return  Returns true if this GraphId is less than the other.
   */
  bool operator <(const GraphId& other) const;

  // Operator EqualTo.
  bool operator ==(const GraphId& rhs) const;

  // Returns the hash code for this object.
  std::size_t HashCode() const;

 protected:
  struct Fields {
    uint64_t tileid :24;
    uint64_t level :3;
    uint64_t id :37;
  };
  Fields graphid_;
};

}
}

#endif // VALHALLA_BALDR_GRAPHID_H_
