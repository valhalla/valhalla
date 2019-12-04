#ifndef VALHALLA_THOR_PATHINFO_H_
#define VALHALLA_THOR_PATHINFO_H_

#include <unordered_map>
#include <valhalla/baldr/graphid.h>
#include <valhalla/sif/costconstants.h>

namespace valhalla {
namespace thor {

/**
 * Simple structure to pass path information from PathAlgorithm
 * to TripLegBuilder
 */
struct PathInfo {
  sif::TravelMode mode;  // Travel mode along this edge
  float elapsed_time;    // Elapsed time in seconds to the end of the edge
  uint32_t trip_id;      // Trip Id (0 if not a transit edge).
  baldr::GraphId edgeid; // Directed edge Id
  float elapsed_cost;    // Cost to the end of the edge in cost units
  bool has_time_restrictions;

  PathInfo(const sif::TravelMode m,
           const float t,
           const baldr::GraphId& edge,
           const uint32_t tripid,
           const float c,
           const bool time_restriction)
      : mode(m), elapsed_time(t), trip_id(tripid), edgeid(edge), elapsed_cost(c),
        has_time_restrictions(time_restriction) {
  }
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_PATHINFO_H_
