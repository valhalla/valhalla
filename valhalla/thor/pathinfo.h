#ifndef VALHALLA_THOR_PATHINFO_H_
#define VALHALLA_THOR_PATHINFO_H_

#include <unordered_map>
#include <valhalla/baldr/graphid.h>
#include <valhalla/sif/costconstants.h>

namespace valhalla {
namespace thor {

/**
 * Simple structure to pass path information from PathAlgorithm
 * to TripPathBuilder
 */
struct PathInfo {
  sif::TravelMode mode;    // Travel mode along this edge
  uint32_t elapsed_time;   // Elapsed time in seconds to the end of the edge
  baldr::GraphId edgeid;   // Directed edge Id

  PathInfo(const sif::TravelMode m, const uint32_t t,
           const baldr::GraphId& edge)
      : mode(m),
        elapsed_time(t),
        edgeid(edge) {
  }
};

}
}

#endif  // VALHALLA_THOR_PATHINFO_H_
