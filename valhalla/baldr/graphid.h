#ifndef VALHALLA_BALDR_GRAPHID_H_
#define VALHALLA_BALDR_GRAPHID_H_

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/json.h>

namespace valhalla {
namespace baldr {

// Maximum of 8 (0-7) graph hierarchies are supported.
constexpr uint32_t kMaxGraphHierarchy = 7;

// 46 bits are used for the non-spare part of a graph Id. Fill all of them.
// If we ever change the size of GraphId fields this will also need to change.
constexpr uint64_t kInvalidGraphId = 0x3fffffffffff;

// Value used to increment an Id by 1
constexpr uint64_t kIdIncrement = 1 << 25;

/**
 * Identifier of a node or an edge within the tiled, hierarchical graph.
 * Includes the tile Id, hierarchy level, and a unique identifier within
 * the tile/level.
 * TODO - currently the Ids are indexes into the node and directed edge
 * "lists" for the tile/level. May need to create persistent Ids at some
 * point.
 */
struct GraphId {
public:
  // Single 64 bit value representing the graph id.
  // Bit fields within the Id include:
  //      3  bits for hierarchy level
  //      22 bits for tile Id (supports lat,lon tiles down to 1/8 degree)
  //      21 bits for id within the tile.
  uint64_t value;

  /**
   * Default constructor
   */
  GraphId() : value(kInvalidGraphId) {
  }

  /**
   * Constructor.
   * @param  tileid Tile Id.
   * @param  level  Hierarchy level
   * @param  id     Unique identifier within the level. Cast this to 64 bits
   *                since the Id portion of the value crosses the 4-byte bdry.
   */
  GraphId(const uint32_t tileid, const uint32_t level, const uint32_t id) {
    if (tileid > kMaxGraphTileId) {
      throw std::logic_error("Tile id out of valid range");
    }
    if (level > kMaxGraphHierarchy) {
      throw std::logic_error("Level out of valid range");
    }
    if (id > kMaxGraphId) {
      throw std::logic_error("Id out of valid range");
    }
    value = level | (tileid << 3) | (static_cast<uint64_t>(id) << 25);
  }

  /**
   * Constructor
   * @param value all the various bits rolled into one
   */
  explicit GraphId(const uint64_t value) : value(value) {
    if (tileid() > kMaxGraphTileId) {
      throw std::logic_error("Tile id out of valid range");
    }
    if (level() > kMaxGraphHierarchy) {
      throw std::logic_error("Level out of valid range");
    }
    if (id() > kMaxGraphId) {
      throw std::logic_error("Id out of valid range");
    }
  }

  /**
   * Constructor
   * @param value a string of the form level/tile_id/id
   */
  explicit GraphId(const std::string& value) {
    std::vector<uint32_t> values;
    std::string::size_type pos = 0;
    while (pos != std::string::npos) {
      auto next = value.find('/', pos + (pos > 0));
      values.push_back(std::stoul(value.substr(pos + (pos > 0), next)));
      pos = next;
    }
    if (values.size() != 3)
      throw std::logic_error("Tile string format does not match level/tile/id");
    *this = GraphId(values[1], values[0], values[2]);
  }

  /**
   * Gets the tile Id.
   * @return   Returns the tile Id.
   */
  inline uint32_t tileid() const {
    return (value & 0x1fffff8) >> 3;
  }

  /**
   * Gets the hierarchy level.
   * @return   Returns the level.
   */
  inline uint32_t level() const {
    return (value & 0x7);
  }

  /**
   * Gets the identifier within the hierarchy level.
   * @return   Returns the unique identifier within the level.
   */
  inline uint32_t id() const {
    return (value & 0x3ffffe000000) >> 25;
  }

  /**
   * Set the Id portion of the GraphId. Since the Id crosses the 4-byte
   * boundary cast it to 64 bits.
   * @param  id  Id to set.
   */
  void set_id(const uint32_t id) {
    value = (value & 0x1ffffff) | (static_cast<uint64_t>(id & 0x1fffff) << 25);
  }

  /**
   * Conversion to bool for use in conditional statements.
   * Note that this is explicit to avoid unexpected implicit conversions. Some
   * statements, including "if", "&&", "||", "!" are "implicit explicit" and
   * will result in conversion.
   * @return boolean true if the id is valid.
   */
  explicit inline operator bool() const {
    return Is_Valid();
  }

  /**
   * Returns true if the id is valid
   * @return boolean true if the id is valid
   */
  bool Is_Valid() const {
    // TODO: make this strict it should check the tile hierarchy not bit field widths
    return value != kInvalidGraphId;
  }

  /**
   * Returns a GraphId omitting the id of the of the object within the level.
   * Construct a new GraphId with the Id portion omitted.
   * @return graphid with only tileid and level included
   */
  GraphId Tile_Base() const {
    return GraphId((value & 0x1ffffff));
  }

  /**
   * Returns a value indicating the tile (level and tile id) of the graph Id.
   * @return  Returns a 32 bit value.
   */
  inline uint32_t tile_value() const {
    return (value & 0x1ffffff);
  }

  /**
   * The json representation of the id
   * @return  json
   */
  json::Value json() const;

  /**
   * Post increments the id.
   */
  GraphId operator++(int) {
    GraphId t = *this;
    value += kIdIncrement;
    return t;
  }

  /**
   * Pre increments the id.
   */
  GraphId& operator++() {
    value += kIdIncrement;
    return *this;
  }

  /**
   * Advances the id
   */
  GraphId operator+(uint64_t offset) const {
    return GraphId(tileid(), level(), id() + offset);
  }

  /**
   * Less than operator for sorting.
   * @param  rhs  Right hand side graph Id for comparison.
   * @return  Returns true if this GraphId is less than the right hand side.
   */
  bool operator<(const GraphId& rhs) const {
    return value < rhs.value;
  }

  // Operator EqualTo.
  bool operator==(const GraphId& rhs) const {
    return value == rhs.value;
  }

  // Operator not equal
  bool operator!=(const GraphId& rhs) const {
    return value != rhs.value;
  }

  // cast operator
  operator uint64_t() const {
    return value;
  }

  // Stream output
  friend std::ostream& operator<<(std::ostream& os, const GraphId& id);
};

} // namespace baldr
} // namespace valhalla

// Extend the standard namespace to know how to hash graphids
namespace std {
template <> struct hash<valhalla::baldr::GraphId> {
  inline std::size_t operator()(const valhalla::baldr::GraphId& k) const {
    // simplified version of murmur3 hash for 64 bit
    uint64_t v = k.value;
    v ^= v >> 33;
    v *= 0xff51afd7ed558ccdULL;
    v ^= v >> 33;
    v *= 0xc4ceb9fe1a85ec53ULL;
    v ^= v >> 33;
    return static_cast<size_t>(v);
  }
};
inline std::string to_string(const valhalla::baldr::GraphId& id) {
  return std::to_string(id.level()) + "/" + std::to_string(id.tileid()) + "/" +
         std::to_string(id.id());
}
} // namespace std

#endif // VALHALLA_BALDR_GRAPHID_H_
