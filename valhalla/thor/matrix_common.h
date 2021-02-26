#ifndef VALHALLA_THOR_MATRIX_COMMON_H_
#define VALHALLA_THOR_MATRIX_COMMON_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/astarheuristic.h>
#include <valhalla/thor/costmatrix.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathalgorithm.h>

namespace valhalla {
namespace thor {

// These cost thresholds are in addition to the distance
// thresholds for quick rejection
constexpr float kTimeDistCostThresholdAutoDivisor =
    112.0f; // 400 km distance threshold will result in a cost threshold of ~2600 (1 hour)
constexpr float kTimeDistCostThresholdBicycleDivisor =
    19.0f; // 200 km distance threshold will result in a cost threshold of ~10800 (3 hours)
constexpr float kTimeDistCostThresholdPedestrianDivisor =
    7.0f; // 200 km distance threshold will result in a cost threshold of ~28800 (8 hours)

// Structure to hold information about each destination.
struct Destination {
  bool settled;        // Has the best time/distance to this destination
                       // been found?
  sif::Cost best_cost; // Current best cost to this destination
  uint32_t distance;   // Path distance for the best cost path
  float threshold;     // Threshold above current best cost where no longer
                       // need to search for this destination.

  // Potential edges for this destination (and their partial distance)
  std::unordered_map<uint64_t, float> dest_edges;

  // Constructor - set best_cost to an absurdly high value so any new cost
  // will be lower.
  Destination() : settled(false), best_cost{kMaxCost, kMaxCost}, distance(0), threshold(0.0f) {
  }
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_MATRIX_COMMON_H_
