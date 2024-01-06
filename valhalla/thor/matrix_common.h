#ifndef VALHALLA_THOR_MATRIX_COMMON_H_
#define VALHALLA_THOR_MATRIX_COMMON_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "proto/matrix.pb.h"
#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/astarheuristic.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathalgorithm.h>
#include <valhalla/worker.h>

namespace valhalla {
namespace thor {

// Default for time distance matrix is to find all locations
constexpr uint32_t kAllLocations = std::numeric_limits<uint32_t>::max();
constexpr float kMaxCost = 99999999.9999f;

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

// return true if any location had a valid time set
// return false if it doesn't make sense computationally and add warnings accordingly
inline bool check_matrix_time(Api& request, const Matrix::Algorithm algo) {
  const auto& options = request.options();
  bool less_sources = options.sources().size() <= options.targets().size();

  for (const auto& source : options.sources()) {
    if (!source.date_time().empty()) {
      if (!less_sources && algo == Matrix::TimeDistanceMatrix) {
        add_warning(request, 201);
        return false;
      }
      return true;
    }
  }
  for (const auto& target : options.targets()) {
    if (!target.date_time().empty()) {
      if (less_sources && algo == Matrix::TimeDistanceMatrix) {
        add_warning(request, 202);
        return false;
      } else if (algo == Matrix::CostMatrix) {
        add_warning(request, 206);
        return false;
      }
      return true;
    }
  }

  return false;
}

// resizes all PBF sequences except for date_times
inline void reserve_pbf_arrays(valhalla::Matrix& matrix, size_t size) {
  matrix.mutable_from_indices()->Resize(size, 0U);
  matrix.mutable_to_indices()->Resize(size, 0U);
  matrix.mutable_distances()->Resize(size, 0U);
  matrix.mutable_times()->Resize(size, 0U);
  matrix.mutable_date_times()->Reserve(size);
  matrix.mutable_shapes()->Reserve(size);
}

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_MATRIX_COMMON_H_
