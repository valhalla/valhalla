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
TimeDepReverse::TimeDepReverse(const boost::property_tree::ptree& config) : TimeDepForward(config) {
}

// Initialize prior to finding best path
void TimeDepReverse::Init(const midgard::PointLL& origll, const midgard::PointLL& destll) {
  // Set the origin lat,lon (since this is reverse path) and cost factor
  // in the A* heuristic
  astarheuristic_.Init(origll, costing_->AStarCostFactor());

  // Get the initial cost based on A* heuristic from destination
  float mincost = astarheuristic_.Get(destll);

  // Reserve size for edge labels - do this here rather than in constructor so
  // to limit how much extra memory is used for persistent objects.
  // TODO - reserve based on estimate based on distance and route type.
  edgelabels_.reserve(kInitialEdgeLabelCount);

  // Construct adjacency list, clear edge status.
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing_->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_.reuse(mincost, range, bucketsize, &edgelabels_);
  edgestatus_.clear();

  // Get hierarchy limits from the costing. Get a copy since we increment
  // transition counts (i.e., this is not a const reference).
  hierarchy_limits_ = costing_->GetHierarchyLimits();
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
  Init(origin_new, destination_new);
  float mindist = astarheuristic_.GetDistance(origin_new);

  // Get time information for backward search
  auto reverse_time_info = TimeInfo::make(destination, graphreader, &tz_cache_);

  // Initialize the locations. For a reverse path search the destination location
  // is used as the "origin" and the origin location is used as the "destination".
  uint32_t density = SetDestination(graphreader, origin);
  SetOrigin(graphreader, destination, origin, reverse_time_info.second_of_week);

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
          return {FormPath(graphreader, predindex)};
        }
      } else {
        return {FormPath(graphreader, predindex)};
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
        return {FormPath(graphreader, best_path.first)};
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

// The origin of the reverse path is the destination location.
// TODO - how do we set the
void TimeDepReverse::SetOrigin(GraphReader& graphreader,
                               const valhalla::Location& origin,
                               const valhalla::Location& destination,
                               const uint32_t seconds_of_week) {
  // Only skip outbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.path_edges().begin(), origin.path_edges().end(),
                [&has_other_edges](const valhalla::Location::PathEdge& e) {
                  has_other_edges = has_other_edges || !e.begin_node();
                });

  // Check if the origin edge matches a destination edge at the node.
  auto trivial_at_node = [this, &destination](const valhalla::Location::PathEdge& edge) {
    auto p = destinations_percent_along_.find(edge.graph_id());
    if (p != destinations_percent_along_.end()) {
      for (const auto& destination_edge : destination.path_edges()) {
        if (destination_edge.graph_id() == edge.graph_id()) {
          return true;
        }
      }
    }
    return false;
  };

  // Iterate through edges and add to adjacency list
  Cost c;
  const NodeInfo* nodeinfo = nullptr;
  const NodeInfo* closest_ni = nullptr;
  for (const auto& edge : origin.path_edges()) {
    // If the origin (real destination) is at a node, skip any outbound
    // edges (so any opposing inbound edges are not considered) unless the
    // destination is also at the same end node (trivial path).
    if (has_other_edges && edge.begin_node() && !trivial_at_node(edge)) {
      continue;
    }

    // Get the directed edge
    GraphId edgeid(edge.graph_id());
    graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the opposing directed edge, continue if we cannot get it
    GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid);
    if (!opp_edge_id.Is_Valid()) {
      continue;
    }
    const DirectedEdge* opp_dir_edge = graphreader.GetOpposingEdge(edgeid);

    // Get cost
    uint8_t flow_sources;
    Cost cost =
        costing_->EdgeCost(directededge, tile, seconds_of_week, flow_sources) * edge.percent_along();
    float dist = astarheuristic_.GetDistance(tile->get_node_ll(opp_dir_edge->endnode()));

    // We need to penalize this location based on its score (distance in meters from input)
    // We assume the slowest speed you could travel to cover that distance to start/end the route
    // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
    // Perhaps need to adjust score?
    cost.cost += edge.distance();

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge) if the
    // destination is in a forward direction along the edge.
    auto settled_dest_edge = destinations_percent_along_.find(opp_edge_id);
    if (settled_dest_edge != destinations_percent_along_.end()) {
      // Reverse the origin and destination in the IsTrivial call.
      if (IsTrivial(edgeid, destination, origin)) {
        // Find the destination edge and update cost.
        for (const auto& dest_path_edge : destination.path_edges()) {
          if (dest_path_edge.graph_id() == edgeid) {
            // a trivial route passes along a single edge, meaning that the
            // destination point must be on this edge, and so the distance
            // remaining must be zero.
            GraphId id(dest_path_edge.graph_id());
            const DirectedEdge* dest_edge = tile->directededge(id);
            Cost remainder_cost = costing_->EdgeCost(dest_edge, tile, seconds_of_week, flow_sources) *
                                  (dest_path_edge.percent_along());
            // Remove the cost of the final "unused" part of the destination edge
            cost -= remainder_cost;
            // Add back in the edge score/penalty to account for destination edges
            // farther from the input location lat,lon.
            cost.cost += dest_path_edge.distance();
            cost.cost = std::max(0.0f, cost.cost);
            dist = 0.0;
          }
        }
      }
    }
    // Store the closest node info
    if (closest_ni == nullptr) {
      closest_ni = nodeinfo;
    }

    // Compute sortcost
    float sortcost = cost.cost + astarheuristic_.Get(dist);

    // Add BDEdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path. Make sure the opposing
    // edge (edgeid) is set.
    // DO NOT SET EdgeStatus - it messes up trivial paths with oneways
    uint32_t idx = edgelabels_.size();
    edgelabels_.emplace_back(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, sortcost, dist,
                             mode_, c, false, !(costing_->IsClosed(directededge, tile)),
                             static_cast<bool>(flow_sources & kDefaultFlowMask),
                             sif::InternalTurn::kNoTurn, -1);
    adjacencylist_.add(idx);

    // Set the initial not_thru flag to false. There is an issue with not_thru
    // flags on small loops. Set this to false here to override this for now.
    edgelabels_.back().set_not_thru(false);

    // Set the origin flag
    edgelabels_.back().set_origin();
  }
}

// Add destination edges at the origin location. If the location is at a node
// skip any outbound edges since the path search is reversed.
// TODO - test to make sure that excluding outbound edges is what we want!
uint32_t TimeDepReverse::SetDestination(GraphReader& graphreader, const valhalla::Location& dest) {
  // Only skip outbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(dest.path_edges().begin(), dest.path_edges().end(),
                [&has_other_edges](const valhalla::Location::PathEdge& e) {
                  has_other_edges = has_other_edges || !e.begin_node();
                });

  // For each edge
  uint32_t density = 0;
  for (const auto& edge : dest.path_edges()) {
    // If destination is at a node skip any outbound edges
    if (has_other_edges && edge.begin_node()) {
      continue;
    }

    // Keep the id and the cost to traverse the partial distance for the
    // remainder of the edge. This cost is subtracted from the total cost
    // up to the end of the destination edge.
    GraphId id(edge.graph_id());
    auto tile = graphreader.GetGraphTile(id);
    const DirectedEdge* directededge = tile->directededge(id);

    // The opposing edge Id is added as a destination since the search
    // is done in reverse direction.
    auto t2 = directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
    if (!t2) {
      continue;
    }
    GraphId oppedge = t2->GetOpposingEdgeId(directededge);
    destinations_percent_along_[oppedge] = edge.percent_along();

    // Edge score (penalty) is handled within GetPath. Do not add score here.

    // Get the tile relative density
    density = tile->header()->density();
  }
  return density;
}

// Form the path from the adjacency list.
std::vector<PathInfo> TimeDepReverse::FormPath(GraphReader& /*graphreader*/, const uint32_t dest) {
  // Metrics to track
  LOG_DEBUG("path_cost::" + std::to_string(edgelabels_[dest].cost().cost));
  LOG_DEBUG("path_iterations::" + std::to_string(edgelabels_.size()));

  // Form the reverse path from the destination (true origin) using opposing edges.
  std::vector<PathInfo> path;
  Cost cost, previous_transition_cost;
  uint32_t edgelabel_index = dest;
  while (edgelabel_index != kInvalidLabel) {
    const BDEdgeLabel& edgelabel = edgelabels_[edgelabel_index];

    // Get elapsed time on the edge, then add the transition cost at prior edge.
    uint32_t predidx = edgelabel.predecessor();
    if (predidx == kInvalidLabel) {
      cost += edgelabel.cost();
    } else {
      cost += edgelabel.cost() - edgelabels_[predidx].cost();
    }
    cost += previous_transition_cost;
    path.emplace_back(edgelabel.mode(), cost, edgelabel.opp_edgeid(), 0, edgelabel.restriction_idx(),
                      previous_transition_cost);

    // Check if this is a ferry
    if (edgelabel.use() == Use::kFerry) {
      has_ferry_ = true;
    }

    // Update edgelabel_index and transition cost to apply at next iteration
    edgelabel_index = predidx;
    // We apply the turn cost at the beginning of the edge, as is done in the forward path
    // Semantically this can be thought of is, how much time did it take to turn onto this edge
    // To do this we need to carry the cost forward to the next edge in the path so we cache it here
    previous_transition_cost = edgelabel.transition_cost();
  }

  return path;
}

} // namespace thor
} // namespace valhalla
