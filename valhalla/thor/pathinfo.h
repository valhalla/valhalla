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
  sif::TravelMode mode; // Travel mode along this edge
  float elapsed_time; // Elapsed time in seconds at the end of the edge including any turn cost at the
                      // start of the edge
  uint32_t trip_id;   // Trip Id (0 if not a transit edge).
  baldr::GraphId edgeid;      // Directed edge Id
  float elapsed_cost;         // Cost to the end of the edge in cost units
  bool has_time_restrictions; // Whether or not this edge has a time restriction
  float turn_cost; // Turn cost in seconds at the beginning of the edge, only in map matching for now

  // TODO: drop this superfluous constructor
  PathInfo(const sif::TravelMode m,
           const float t,
           const baldr::GraphId& edge,
           const uint32_t tripid,
           const float c,
           const bool time_restriction,
           const float tcs = 0)
      : mode(m), elapsed_time(t), trip_id(tripid), edgeid(edge), elapsed_cost(c),
        has_time_restrictions(time_restriction), turn_cost(tcs) {
  }

  // Stream output
  friend std::ostream& operator<<(std::ostream& os, const PathInfo& p) {
    os << std::fixed << std::setprecision(3);
    os << "mode: " << static_cast<int>(p.mode) << ", elapsed_time: " << p.elapsed_time
       << ", trip_id: " << p.trip_id << ", edgeid: " << p.edgeid << ", turn_cost: " << p.turn_cost;
    return os;
  }
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_PATHINFO_H_
