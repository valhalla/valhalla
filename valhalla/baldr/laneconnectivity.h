#ifndef VALHALLA_BALDR_LANECONNECTIVITY_H_
#define VALHALLA_BALDR_LANECONNECTIVITY_H_

#include <cstdint>
#include <string>

namespace valhalla {
namespace baldr {

constexpr size_t kMaxLanesPerConnectionBits = 4;
constexpr size_t kMaxLanesPerConnection = (1 << kMaxLanesPerConnectionBits) - 1;

/**
 * Structure to compactly store lane mask.
 *
 * Example: 1|2|3|4|4
 */
class LaneConnectivityLanes {
public:
  /**
   * Constructor with arguments.
   * @param lanes String representation of lane mask
   */
  LaneConnectivityLanes(const std::string& lanes);

  /**
   * Get the text representation of lane mask.
   * @return  text representation of lane mask.
   */
  std::string to_string() const;

private:
  void set_lane(uint8_t n, uint8_t lane);
  uint8_t get_lane(uint8_t n) const;

protected:
  uint64_t value_; // Single 64 bit value representing lane mask.
};

/**
 * Structure to store lane connectivity between two edges.
 */
class LaneConnectivity {
public:
  /**
   * Default constructor.
   */
  LaneConnectivity();

  /**
   * Constructor with arguments.
   * @param  idx     Directed edge index.
   * @param  from     From segment Id.
   * @param  to_lanes  List of lanes on `to` edge
   * @param  from_lanes List of lanes on `from` edge.
   */
  LaneConnectivity(const uint32_t idx,
                   const uint64_t from,
                   const std::string& to_lanes,
                   const std::string& from_lanes);

  /**
   * operator < - for sorting. Sort by `to` Id.
   * @param  other  Other lane connectivity structure to compare to.
   * @return  Returns true if id < other id.
   */
  bool operator<(const LaneConnectivity& other) const;

  /**
   * Get the index of the directed edge this lane connection applies to.
   * @return  Returns the directed edge index (within the same tile
   *          as the lane connectivity information).
   */
  uint32_t to() const;

  /**
   * Set the directed edge index to which this lane connection applies to.
   * @param idx Edge index.
   */
  void set_to(const uint32_t idx);

  /**
   * Get the OSM id of the incoming way of this lane connection.
   * @return  OSM id of the incoming way of this lane connection.
   */
  uint64_t from() const;

  /**
   * Get the text representation of lanes in the incoming way.
   * @return  text representation of lanes
   */
  std::string from_lanes() const;

  /**
   * Get the text representation of lanes in the current way.
   * @return  text representation of lanes
   */
  std::string to_lanes() const;

protected:
  uint64_t to_ : 22;   // where this connection is going to (edge index)
  uint64_t from_ : 42; // where this connection is coming from (way id) // (XXX: 42-bits, maybe be a
                       // problem in a few years)
  LaneConnectivityLanes to_lanes_;
  LaneConnectivityLanes from_lanes_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_LANECONNECTIVITY_H_
