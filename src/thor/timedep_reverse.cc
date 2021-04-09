#include "baldr/datetime.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "thor/timedep.h"
#include <algorithm>
#include <string>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

// TODO - compute initial label count based on estimated route length
constexpr uint32_t kInitialEdgeLabelCount = 500000;

// Number of iterations to allow with no convergence to the destination
constexpr uint32_t kMaxIterationsWithoutConvergence = 1800000;

// Default constructor
TimeDepReverse::TimeDepReverse(const boost::property_tree::ptree& config) : TimeDep(config) {
}

// Calculate time-dependent best path using a reverse search. Supports
// "arrive-by" routes.
std::vector<std::vector<PathInfo>>
TimeDepReverse::GetBestPath(valhalla::Location& origin,
                            valhalla::Location& destination,
                            GraphReader& graphreader,
                            const sif::mode_costing_t& mode_costing,
                            const TravelMode mode,
                            const Options& /*options*/) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();
  access_mode_ = costing_->access_mode();

  // date_time must be set on the destination. Log an error but allow routes for now.
  if (!destination.has_date_time()) {
    LOG_ERROR("TimeDepReverse called without time set on the destination location");
    // return {};
  }

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  // Note: because we can correlate to more than one place for a given PathLocation
  // using edges.front here means we are only setting the heuristics to one of them
  // alternate paths using the other correlated points to may be harder to find
  midgard::PointLL origin_new(origin.path_edges(0).ll().lng(), origin.path_edges(0).ll().lat());
  midgard::PointLL destination_new(destination.path_edges(0).ll().lng(),
                                   destination.path_edges(0).ll().lat());
  Init<TimeDep::ExpansionType::reverse>(origin_new, destination_new);
  float mindist = astarheuristic_.GetDistance(origin_new);

  // Get time information for backward search
  auto reverse_time_info = TimeInfo::make(destination, graphreader, &tz_cache_);

  // Initialize the locations. For a reverse path search the destination location
  // is used as the "origin" and the origin location is used as the "destination".
  uint32_t density = SetDestination<TimeDep::ExpansionType::reverse>(graphreader, origin);
  SetOrigin<TimeDep::ExpansionType::reverse>(graphreader, destination, origin,
                                             reverse_time_info.second_of_week);

  // Update hierarchy limits
  ModifyHierarchyLimits(mindist, density);

  // Find shortest path
  uint32_t nc = 0; // Count of iterations with no convergence
                   // towards destination
  std::pair<int32_t, float> best_path = std::make_pair(-1, 0.0f);
  graph_tile_ptr tile;
  size_t total_labels = 0;
  while (true) {
    // Allow this process to be aborted
    size_t current_labels = edgelabels_.size();
    if (interrupt &&
        total_labels / kInterruptIterationsInterval < current_labels / kInterruptIterationsInterval) {
      (*interrupt)();
    }
    total_labels = current_labels;

    // Abort if max label count is exceeded
    if (total_labels > max_label_count_) {
      return {};
    }

    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_.pop();

    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " + std::to_string(edgelabels_.size()));
      return {};
    }

    // Copy the BDEdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    BDEdgeLabel pred = edgelabels_[predindex];
    if (destinations_percent_along_.find(pred.edgeid()) != destinations_percent_along_.end()) {
      // Check if a trivial path using opposing edge. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        // Use opposing edge.
        if (IsTrivial(pred.opp_edgeid(), origin, destination)) {
          return {FormPathFromReverseSearch(predindex)};
        }
      } else {
        return {FormPathFromReverseSearch(predindex)};
      }
    }

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge (this will allow loops/around the block cases)
    if (!pred.origin()) {
      edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Check that distance is converging towards the destination. Return route
    // failure if no convergence for TODO iterations
    float dist2dest = pred.distance();
    if (dist2dest < mindist) {
      mindist = dist2dest;
      nc = 0;
    } else if (nc++ > kMaxIterationsWithoutConvergence) {
      if (best_path.first >= 0) {
        return {FormPathFromReverseSearch(best_path.first)};
      } else {
        LOG_ERROR("No convergence to destination after = " + std::to_string(edgelabels_.size()));
        return {};
      }
    }

    // Do not expand based on hierarchy level based on number of upward
    // transitions and distance to the destination
    if (hierarchy_limits_[pred.endnode().level()].StopExpanding(dist2dest)) {
      continue;
    }

    // Get the opposing predecessor directed edge. Need to make sure we get
    // the correct one if a transition occurred
    const DirectedEdge* opp_pred_edge =
        graphreader.GetGraphTile(pred.opp_edgeid())->directededge(pred.opp_edgeid());

    // Expand forward from the end node of the predecessor edge.
    Expand<ExpansionType::reverse>(graphreader, pred.endnode(), pred, predindex, opp_pred_edge,
                                   reverse_time_info, destination, best_path);
  }
  return {}; // Should never get here
}

} // namespace thor
} // namespace valhalla
