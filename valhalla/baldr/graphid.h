#ifndef VALHALLA_BALDR_GRAPHID_H_
#define VALHALLA_BALDR_GRAPHID_H_

#include <cstdint>
#include <functional>
#include <iostream>

namespace valhalla {
namespace baldr {

/**
 * Identifier of a node or an edge within the tiled, hierarchical graph.
 * Includes the tile Id, hierarchy level, and a unique identifier within
 * the tile/level.
 * TODO - currently the Ids are indexes into the node and directed edge
 * "lists" for the tile/level. May need to create persistent Ids at some
 * point.
 * @author  David W. Nesbitt
 */
class GraphId {
 public:
  /**
   * Default constructor
   */
  GraphId();

  /**
   * Constructor.
   * @param  tileid Tile Id.
   * @param  level  Hierarchy level
   * @param  id     Unique identifier within the level.
   */
  GraphId(const uint32_t tileid, const uint32_t level,
          const uint64_t id);

  /**
   * Copy constructor.
   * @param  g   GraphId to copy
   */
  GraphId(const GraphId& g);

  /**
   * Return a single 64 bit value representing the graph id.
   * @return  Returns the graphId 64-bit value.
   */
  uint64_t value() const;

  /**
   * Gets the tile Id.
   * @return   Returns the tile Id.
   */
  uint32_t tileid() const;

  /**
   * Gets the hierarchy level.
   * @return   Returns the level.
   */
  uint32_t level() const;

  /**
   * Gets the identifier within the hierarchy level.
   * @return   Returns the unique identifier within the level.
   */
  uint64_t id() const;

  /**
   * Convenience method to set individual graph Id elements.
   * @param  tileid Tile Id.
   * @param  level  Hierarchy level
   * @param  id     Unique identifier within the level
   */
  void Set(const uint32_t tileid, const uint32_t level,
           const uint64_t id);

  /**
   * Returns true if the id is valid
   *
   * @return boolean true if the id is valid
   */
  bool Is_Valid() const;

  /**
   * Returns a GraphId omitting the id of the of the object within the level
   *
   * @return graphid with only tileid and level included
   */
  GraphId Tile_Base() const;

  /**
   * Get the hash value of GraphId attributes.
   * @return   Returns hash value.
   */
  const uint64_t hash_value();

  /**
   * Post increments the id.
   */
  void operator ++(int);

  /**
   * Less than operator for sorting.
   * @param  rhs  Right hand side graph Id for comparison.
   * @return  Returns true if this GraphId is less than the right hand side.
   */
  bool operator <(const GraphId& rhs) const;

  // Operator EqualTo.
  bool operator ==(const GraphId& rhs) const;

 protected:
  union Id {
    struct Fields {
      uint64_t tileid :24;
      uint64_t level :3;
      uint64_t id :37;
    } fields;
    uint64_t v;
  };
  Id graphid_;

  friend std::ostream& operator<<(std::ostream& os, const GraphId& id);
};

}
}

// Extend the standard namespace to know how to hash graphids
namespace std {
  template <>
  struct hash<valhalla::baldr::GraphId>
  {
    std::size_t operator()(const valhalla::baldr::GraphId& k) const
    {
      return static_cast<size_t>(k.value());
    }
  };
}


#endif // VALHALLA_BALDR_GRAPHID_H_
