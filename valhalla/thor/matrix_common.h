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
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathalgorithm.h>
#include <valhalla/worker.h>

namespace valhalla {
namespace thor {

enum class MatrixType : bool { TimeDist, Cost };

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

// Time and Distance structure
struct TimeDistance {
  uint32_t time; // Time in seconds
  uint32_t dist; // Distance in meters
  std::string date_time;

  TimeDistance() : time(0), dist(0), date_time("") {
  }

  TimeDistance(const uint32_t secs, const uint32_t meters) : time(secs), dist(meters), date_time("") {
  }

  TimeDistance(const uint32_t secs, const uint32_t meters, std::string date_time)
      : time(secs), dist(meters), date_time(date_time) {
  }
};

// Will return a destination's date_time string
inline std::string get_date_time(const std::string& origin_dt,
                                 const uint64_t& origin_tz,
                                 const baldr::GraphId& pred_id,
                                 baldr::GraphReader& reader,
                                 const uint64_t& offset) {
  if (origin_dt.empty()) {
    return "";
  } else if (!offset) {
    return origin_dt;
  }
  graph_tile_ptr tile = nullptr;
  uint32_t dest_tz = 0;
  if (pred_id.Is_Valid()) {
    // get the timezone of the output location
    auto out_nodes = reader.GetDirectedEdgeNodes(pred_id, tile);
    dest_tz = reader.GetTimezone(out_nodes.first, tile) || reader.GetTimezone(out_nodes.second, tile);
  }

  auto in_epoch =
      baldr::DateTime::seconds_since_epoch(origin_dt,
                                           baldr::DateTime::get_tz_db().from_index(origin_tz));
  uint64_t out_epoch = in_epoch + offset;
  std::string out_dt =
      baldr::DateTime::seconds_to_date(out_epoch, baldr::DateTime::get_tz_db().from_index(dest_tz),
                                       false);

  return out_dt;
}

// return true if any location had a valid time set
// return false if it doesn't make sense computationally and add warnings accordingly
inline bool check_matrix_time(Api& request, const MatrixType type) {
  const auto& options = request.options();
  bool less_sources = options.sources().size() <= options.targets().size();

  for (const auto& source : options.sources()) {
    if (!source.date_time().empty()) {
      if (!less_sources && type == MatrixType::TimeDist) {
        add_warning(request, 201);
        return false;
      }
      return true;
    }
  }
  for (const auto& target : options.targets()) {
    if (!target.date_time().empty()) {
      if (less_sources && type == MatrixType::TimeDist) {
        add_warning(request, 202);
        return false;
      } else if (type == MatrixType::Cost) {
        add_warning(request, 206);
        return false;
      }
      return true;
    }
  }

  return false;
}

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_MATRIX_COMMON_H_
