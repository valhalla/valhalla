#ifndef VALHALLA_THOR_PATHINFO_H_
#define VALHALLA_THOR_PATHINFO_H_

#include <unordered_map>
#include <valhalla/baldr/graphid.h>
#include <valhalla/sif/costconstants.h>

namespace valhalla {
namespace thor {

/**
 * Simple(ish) structure to pass path information from PathAlgorithm
 * to TripLegBuilder
 */
struct PathInfo {
  sif::TravelMode mode;   // Travel mode along this edge
  sif::Cost elapsed_cost; // Elapsed cost at the end of the edge including any turn cost at the start
                          // of the edge
  uint32_t trip_id;       // Trip Id (0 if not a transit edge).
  baldr::GraphId edgeid;  // Directed edge Id
  float path_distance;    // Distance (in meters) from the start to the edge
  uint8_t restriction_index;    // Record which restriction
  sif::Cost transition_cost;    // Turn cost at the beginning of the edge
  bool start_node_is_recovered; // Indicates if the start node of the edge is an inner node
                                // of a shortcut that was recovered. Pay attention this flag
                                // is 'false' for the first and the last shortcut nodes.

  // TODO: drop this superfluous constructor
  PathInfo(const sif::TravelMode m,
           const sif::Cost c,
           const baldr::GraphId& edge,
           const uint32_t tripid,
           const float path_distance,
           const uint8_t restriction_idx = baldr::kInvalidRestriction,
           const sif::Cost tc = {},
           bool start_node_is_recovered = false)
      : mode(m), elapsed_cost(c), trip_id(tripid), edgeid(edge), path_distance(path_distance),
        restriction_index(restriction_idx), transition_cost(tc),
        start_node_is_recovered(start_node_is_recovered) {
  }

  // Stream output
  friend std::ostream& operator<<(std::ostream& os, const PathInfo& p) {
    os << std::fixed << std::setprecision(3);
    os << "mode: " << static_cast<int>(p.mode) << ", elapsed_time: " << p.elapsed_cost.secs
       << ", elapsed_cost: " << p.elapsed_cost.cost << ", trip_id: " << p.trip_id
       << ", edgeid: " << p.edgeid << ", path_distance" << p.path_distance
       << ", transition_time: " << p.transition_cost.secs
       << ", transition_cost: " << p.transition_cost.cost;
    return os;
  }
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_PATHINFO_H_
