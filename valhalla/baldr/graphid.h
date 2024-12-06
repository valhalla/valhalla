#ifndef VALHALLA_BALDR_GRAPHID_H_
#define VALHALLA_BALDR_GRAPHID_H_

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/json.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace baldr {

// Maximum of 8 (0-7) graph hierarchies are supported.
constexpr uint32_t kMaxGraphHierarchy = 7;

// 46 bits are used for the non-spare part of a graph Id. Fill all of them.
// If we ever change the size of GraphId fields this will also need to change.
constexpr uint64_t kInvalidGraphId = 0x3fffffffffff;

// Value used to increment an Id by 1
constexpr uint64_t kIdIncrement = 1 << 25;

constexpr std::array<int, 4> kBoundingCircleRadii = {25 * 25, 75 * 75, 150 * 150, 375 * 375};
constexpr uint64_t kImpossibleBoundingCircle = 0xffff;

// bin size in meters at the equator (half that since we offset from center)
const double kMaxOffsetMeters =
    0.05 * midgard::kMetersPerDegreeLat / 2 + std::sqrt(kBoundingCircleRadii.back());
// in meters
const double kOffsetIncrement = kMaxOffsetMeters / 128;

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

/**
 * A derived class of GraphId that uses the 18 spare bits to store a coarse bounding circle that
 * contains the geometry of the edge this GraphId refers to.
 *
 * It stores three additional pieces of information: the center lat/lon offsets and the radius.
 * The 2 most significant bits represent an index into a fixed array of radii that were chosen based
 * on the distribution of minimum bounding circle radii of all edges in a planet build.
 *
 * The remaing 2*8 bits are used to store offsets in meters relative to the center of the tile bin in
 * which this structure is stored. One increment evaluates to roughly 21.6 meters, in order to support
 * the maximum combination of the maximum distance from the bin center (half its size at the equator
 * plus the max supported radius).
 */
struct DiscretizedBoundingCircle : public GraphId {
  using GraphId::GraphId;

  /**
   * this is annoying, but to be backwards compatible with older graph tiles
   * that have the 18 bits of the Graph IDs zero'ed, we need the  zero'ed version
   * to be the sentinel value for an invalid circle, even though it's actually a valid
   * bounding circle: it just means the center is less than kOffsetIncrement meters away
   * from the bin center in both dimensions, and the radius is less than or equal to the
   * minimum radius we support.
   *
   * Luckily, there are some combinations which are not valid: those where the radius is
   * smaller than distance between the circle center and the bin boundaries. So we can use one
   * of those "impossible" values to imply the valid zero'ed out value, and reserve the zero'ed
   * out value to mean "invalid circle, go look at the shape".
   */

  /**
   * Set the bounding circle.
   *
   * @param bin_center_approx distance approximator for the current bin's center point
   * @param bin_center        the current bin's center point
   * @param circle_center     the bounding circle's center point
   * @param radius            the bounding circle's radius in meters
   * @return the index of the radius used, or the max radius index + 1 if it was too large
   */
  size_t set(const midgard::DistanceApproximator<midgard::PointLL>& bin_center_approx,
             const midgard::PointLL& bin_center,
             const midgard::PointLL& circle_center,
             double radius) {
    // reset if it was previously set
    value |= (value & kInvalidGraphId);

    midgard::PointLL offset{circle_center.lng() - bin_center.lng(),
                            circle_center.lat() - bin_center.lat()};

    double x_meters = offset.lng() * bin_center_approx.GetLngScale() * midgard::kMetersPerDegreeLat;
    double y_meters = offset.lat() * midgard::kMetersPerDegreeLat;

    unsigned int index = kBoundingCircleRadii.size();

    // if any of the offsets is larger than the largest value we support bail
    if (std::abs(x_meters) >= kMaxOffsetMeters - 0.5 || std::abs(y_meters) >= kMaxOffsetMeters - 0.5)
      return index;

    // this gives us the offset values we store
    uint64_t x_offset_increments =
        static_cast<uint64_t>((x_meters + kMaxOffsetMeters) / kOffsetIncrement + 0.5);

    uint64_t y_offset_increments =
        static_cast<uint64_t>((y_meters + kMaxOffsetMeters) / kOffsetIncrement + 0.5);

    // but we're not done yet, we need to account for the loss of precision
    // and resize the radius accordingly so that the bounding circle is
    // still guaranteed to cover the edge shape
    double discretized_y_offset =
        ((static_cast<double>(y_offset_increments) / static_cast<double>(1 << 8)) * kMaxOffsetMeters *
         2) -
        kMaxOffsetMeters;
    double discretized_x_offset =
        ((static_cast<double>(x_offset_increments) / static_cast<double>(1 << 8)) * kMaxOffsetMeters *
         2) -
        kMaxOffsetMeters;

    midgard::PointLL discretized_center{(discretized_x_offset / (bin_center_approx.GetLngScale() *
                                                                 midgard::kMetersPerDegreeLat)) +
                                            bin_center.lng(),
                                        discretized_y_offset / midgard::kMetersPerDegreeLat +
                                            bin_center.lat()};

    auto loss_of_precision = circle_center.Distance(discretized_center);
    radius += loss_of_precision;
    auto radius_sq = radius * radius;

    for (unsigned int i = 0; i < kBoundingCircleRadii.size(); ++i) {
      auto r = kBoundingCircleRadii[i];
      if (radius_sq <= r) {
        index = i;
        break;
      }
    }
    // only set the value if radius doesn't exceed the max radius we support
    if (!index == kBoundingCircleRadii.size())
      value |= (static_cast<uint64_t>(index) << 62) | (x_offset_increments << 54) |
               (y_offset_increments << 46);

    return index;
  }

  /**
   * Set the bounding circle from another discretized bounding circle
   * (useful for opposing edge pairs).
   */
  void set_from_other(const DiscretizedBoundingCircle& other) {
    value = (value & kInvalidGraphId) | (other.value & 0xffffc00000000000);
  }

  // TODO(chris): remove after testing
  std::pair<midgard::PointLL, double>
  get(const midgard::DistanceApproximator<midgard::PointLL>& approx,
      const midgard::PointLL& bin_center) {

    // old data doesn't have this set, so this means go check the shape
    auto bounding_circle = value >> 46;
    if (!bounding_circle) {
      return {{0, 0}, 0.};
    }

    // for the case where we created a valid circle, but it happens to be at
    // the bin's center and has the smallest radius
    if (bounding_circle == kImpossibleBoundingCircle) {
      bounding_circle = 0;
    }
    auto y_offset_meters =
        ((static_cast<double>(bounding_circle & 0xff) / static_cast<double>(1 << 8)) *
         kMaxOffsetMeters * 2) -
        kMaxOffsetMeters;
    auto x_offset_meters =
        ((static_cast<double>((bounding_circle >> 8) & 0xff) / static_cast<double>(1 << 8)) *
         kMaxOffsetMeters * 2) -
        kMaxOffsetMeters;
    midgard::PointLL center{(x_offset_meters / (approx.GetMetersPerLngDegree())) + bin_center.lng(),
                            y_offset_meters / midgard::kMetersPerDegreeLat + bin_center.lat()};

    auto discretized_radius_sq = kBoundingCircleRadii[bounding_circle >> 16];
    return {center, std::sqrt(discretized_radius_sq)};
  }

  /**
   * This is the actual test whether the edge is within a given radius to a
   * location.
   *
   * @param approx the bin center distance approximator.
   * @param loc_approx the bin center point.
   * @param radius_sq the squared search radius of the location
   * @param approx the bin center point.
   *
   * @return false if the edge is not within the given radius to the edge, else true
   */
  bool operator()(const midgard::DistanceApproximator<midgard::PointLL>& approx,
                  const midgard::DistanceApproximator<midgard::PointLL>& loc_approx,
                  const double radius_sq,
                  const midgard::PointLL& bin_center) const {
    // old data doesn't have this set, so this means go check the shape
    auto bounding_circle = value >> 46;
    if (!bounding_circle) {
      return true;
    }

    // for the case where we created a valid circle, but it happens to be at
    // the bin's center and has the smallest radius
    if (bounding_circle == kImpossibleBoundingCircle)
      bounding_circle = 0;

    double y_offset_meters =
        ((static_cast<double>(bounding_circle & 0xff) / static_cast<double>(1 << 8)) *
         kMaxOffsetMeters * 2) -
        kMaxOffsetMeters;
    double x_offset_meters =
        ((static_cast<double>((bounding_circle >> 8) & 0xff) / static_cast<double>(1 << 8)) *
         kMaxOffsetMeters * 2) -
        kMaxOffsetMeters;
    midgard::PointLL center{(x_offset_meters /
                             (approx.GetLngScale() * midgard::kMetersPerDegreeLat)) +
                                bin_center.lng(),
                            y_offset_meters / midgard::kMetersPerDegreeLat + bin_center.lat()};

    auto discretized_radius_sq = kBoundingCircleRadii[bounding_circle >> 16];
    return loc_approx.DistanceSquared(center) < discretized_radius_sq + radius_sq;
  }
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
