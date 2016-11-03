#ifndef VALHALLA_BALDR_GRAPHID_H_
#define VALHALLA_BALDR_GRAPHID_H_

#include <cstdint>
#include <functional>
#include <iostream>

#include <valhalla/baldr/json.h>

namespace valhalla {
namespace baldr {

// Maximum of 8 (0-7) graph hierarchies are supported.
constexpr uint32_t kMaxGraphHierarchy = 7;

/**
 * Identifier of a node or an edge within the tiled, hierarchical graph.
 * Includes the tile Id, hierarchy level, and a unique identifier within
 * the tile/level.
 * TODO - currently the Ids are indexes into the node and directed edge
 * "lists" for the tile/level. May need to create persistent Ids at some
 * point.
 * @author  David W. Nesbitt
 */
union GraphId {
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
  GraphId(const uint32_t tileid, const uint32_t level, const uint32_t id);

  /**
   * Constructor
   * @param value all the various bits rolled into one
   */
  explicit GraphId(const uint64_t value);

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
  uint32_t id() const;


  /**
   * Convenience method to set individual graph Id elements.
   * @param  tileid Tile Id.
   * @param  level  Hierarchy level
   * @param  id     Unique identifier within the level
   */
  void Set(const uint32_t tileid, const uint32_t level,
           const uint32_t id);

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
   * The json representation of the id
   *
   * @return  json
   */
  json::Value json() const;

  /**
   * Post increments the id.
   */
  void operator ++(int);

  /**
   * Advances the id
   */
  GraphId operator+(uint64_t offset) const;

  /**
   * Less than operator for sorting.
   * @param  rhs  Right hand side graph Id for comparison.
   * @return  Returns true if this GraphId is less than the right hand side.
   */
  bool operator <(const GraphId& rhs) const;

  // Operator EqualTo.
  bool operator ==(const GraphId& rhs) const;

  bool operator !=(const GraphId& rhs) const;

  // cast operator
  operator uint64_t() const;

  struct Fields {
    //the hierarchy level
    uint64_t level :3;
    //the tile id
    uint64_t tileid :22;
    //the id of the element within the tile
    uint64_t id :21;
    uint64_t spare :18;
  } fields;
  //a single 64 bit value representing the graph id.
  uint64_t value;

  friend std::ostream& operator<<(std::ostream& os, const GraphId& id);
};

}
}

// Extend the standard namespace to know how to hash graphids
namespace std {
  template <>
  struct hash<valhalla::baldr::GraphId>
  {
    inline std::size_t operator()(const valhalla::baldr::GraphId& k) const
    {
      return static_cast<size_t>(k.value);
    }
  };
}


#endif // VALHALLA_BALDR_GRAPHID_H_
