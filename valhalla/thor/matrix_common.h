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

// Default for time distance matrix is to find all locations
constexpr uint32_t kAllLocations = std::numeric_limits<uint32_t>::max();

// Structure to hold information about each destination.
struct Destination {
  // per-origin information
  bool settled;        // Has the best time/distance to this destination
                       // been found?
  sif::Cost best_cost; // Current best cost to this destination
  // Set of still available correlated edges;
  std::unordered_set<uint64_t> dest_edges_available;

  // global information which only needs to be set once or is reset for every origin in the algorithm
  uint32_t distance; // Path distance for the best cost path
  float threshold;   // Threshold above current best cost where no longer
                     // need to search for this destination.
  // partial distance of correlated edges
  std::unordered_map<uint64_t, float> dest_edges_percent_along;

  // Constructor - set best_cost to an absurdly high value so any new cost
  // will be lower.
  Destination() : settled(false), best_cost{kMaxCost, kMaxCost}, distance(0), threshold(0.0f) {
  }

  // clears the per-origin information
  void reset() {
    settled = false;
    best_cost = {kMaxCost, kMaxCost};
    dest_edges_available.clear();
  }
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_MATRIX_COMMON_H_
