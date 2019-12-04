#include "thor/astar.h"
#include "baldr/datetime.h"
#include "midgard/logging.h"
#include <algorithm>
#include <iostream> // TODO remove if not needed
#include <map>

using namespace valhalla::baldr;
using namespace valhalla::sif;

// TODO: make a class that extends std::exception, with messages and
// error codes and return the appropriate error codes

namespace valhalla {
namespace thor {

constexpr uint64_t kInitialEdgeLabelCount = 500000;

// Number of iterations to allow with no convergence to the destination
constexpr uint32_t kMaxIterationsWithoutConvergence = 200000;

// Default constructor
AStarPathAlgorithm::AStarPathAlgorithm()
    : PathAlgorithm(), mode_(TravelMode::kDrive), travel_type_(0), adjacencylist_(nullptr),
      max_label_count_(std::numeric_limits<uint32_t>::max()) {
}

// Destructor
AStarPathAlgorithm::~AStarPathAlgorithm() {
  Clear();
}

// Clear the temporary information generated during path construction.
void AStarPathAlgorithm::Clear() {
  // Clear the edge labels and destination list. Reset the adjacency list
  // and clear edge status.
  edgelabels_.clear();
  destinations_.clear();
  adjacencylist_.reset();
  edgestatus_.clear();

  // Set the ferry flag to false
  has_ferry_ = false;
}

// Initialize prior to finding best path
void AStarPathAlgorithm::Init(const midgard::PointLL& origll, const midgard::PointLL& destll) {
  LOG_TRACE("Orig LL = " + std::to_string(origll.lat()) + "," + std::to_string(origll.lng()));
  LOG_TRACE("Dest LL = " + std::to_string(destll.lat()) + "," + std::to_string(destll.lng()));

  // Set the destination and cost factor in the A* heuristic
  astarheuristic_.Init(destll, costing_->AStarCostFactor());

  // Get the initial cost based on A* heuristic from origin
  float mincost = astarheuristic_.Get(origll);

  // Reserve size for edge labels - do this here rather than in constructor so
  // to limit how much extra memory is used for persistent objects.
  // TODO - reserve based on estimate based on distance and route type.
  edgelabels_.reserve(kInitialEdgeLabelCount);

  // Set up lambda to get sort costs
  const auto edgecost = [this](const uint32_t label) { return edgelabels_[label].sortcost(); };

  // Construct adjacency list, clear edge status.
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing_->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_.reset(new DoubleBucketQueue(mincost, range, bucketsize, edgecost));
  edgestatus_.clear();

  // Get hierarchy limits from the costing. Get a copy since we increment
  // transition counts (i.e., this is not a const reference).
  hierarchy_limits_ = costing_->GetHierarchyLimits();
}

// Modulate the hierarchy expansion within distance based on density at
// the destination (increase distance for lower densities and decrease
// for higher densities) and the distance between origin and destination
// (increase for shorter distances).
void AStarPathAlgorithm::ModifyHierarchyLimits(const float dist, const uint32_t density) {
  // TODO - default distance below which we increase expansion within
  // distance. This is somewhat temporary to address route quality on shorter
  // routes - hopefully we will mark the data somehow to indicate how to
  // use the hierarchy when approaching the destination (or use a
  // bi-directional search without hierarchies for shorter routes).
  float factor = 1.0f;
  if (25000.0f < dist && dist < 100000.0f) {
    factor = std::min(3.0f, 100000.0f / dist);
  }
  /* TODO - need a reliable density factor near the destination (e.g. tile level?)
  // Low density - increase expansion within distance.
  // High density - decrease expansion within distance.
  if (density < 8) {
    float f = 1.0f + (8.0f - density) * 0.125f;
    factor *= f;
  } else if (density > 8) {
    float f = 0.5f + (15.0f - density) * 0.0625;
    factor *= f;
  }*/
  // TODO - just arterial for now...investigate whether to alter local as well
  hierarchy_limits_[1].expansion_within_dist *= factor;
}

// Expand from the node along the forward search path. Immediately expands
// from the end node of any transition edge (so no transition edges are added
// to the adjacency list or EdgeLabel list). Does not expand transition
// edges if from_transition is false.
void AStarPathAlgorithm::ExpandForward(GraphReader& graphreader,
                                       const GraphId& node,
                                       const EdgeLabel& pred,
                                       const uint32_t pred_idx,
                                       const bool from_transition,
                                       const valhalla::Location& destination,
                                       std::pair<int32_t, float>& best_path) {
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }
  const NodeInfo* nodeinfo = tile->node(node);
  if (!costing_->Allowed(nodeinfo)) {
    return;
  }

  // Expand from end node.
  uint32_t shortcuts = 0;
  uint32_t max_shortcut_length = static_cast<uint32_t>(pred.distance() * 0.5f);
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  EdgeStatusInfo* es = edgestatus_.GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++directededge, ++edgeid, ++es) {
    // Skip shortcut edges until we have stopped expanding on the next level.
    // Also skip shortcut edges when near the destination. Always skip within
    // 10km but also reject long shortcut edges outside this distance.
    // TODO - configure this distance based on density?
    // Use regular edges while still expanding on the next level since we can still
    // transition down to that level. If using a shortcut, set the shortcuts mask.
    // Skip if this is a regular edge superseded by a shortcut.
    if (directededge->is_shortcut()) {
      if (!hierarchy_limits_[edgeid.level() + 1].StopExpanding() ||
          (pred.distance() < 10000.0f || directededge->length() > max_shortcut_length)) {
        continue;
      } else {
        shortcuts |= directededge->shortcut();
      }
    } else if (shortcuts & directededge->superseded()) {
      continue;
    }

    // Skip this edge if permanently labeled (best path already found to this
    // directed edge), if no access is allowed to this edge (based on costing method),
    // or if a complex restriction exists.
    bool has_time_restrictions = false;
    if (es->set() == EdgeSet::kPermanent ||
        !costing_->Allowed(directededge, pred, tile, edgeid, 0, 0, has_time_restrictions) ||
        costing_->Restricted(directededge, pred, edgelabels_, tile, edgeid, true)) {
      continue;
    }

    // Compute the cost to the end of this edge
    Cost newcost = pred.cost() + costing_->EdgeCost(directededge, tile) +
                   costing_->TransitionCost(directededge, nodeinfo, pred);

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge).
    auto p = destinations_.find(edgeid);
    if (p != destinations_.end()) {
      // Subtract partial cost and time
      newcost -= p->second;

      // Find the destination edge and update cost to include the edge score.
      // Note - with high edge scores the convergence test fails some routes
      // so reduce the edge score.
      for (const auto& destination_edge : destination.path_edges()) {
        if (destination_edge.graph_id() == edgeid) {
          newcost.cost += destination_edge.distance();
        }
      }
      newcost.cost = std::max(0.0f, newcost.cost);

      // Mark this as the best connection if that applies. This allows
      // a path to be formed even if the convergence test fails (can
      // happen with large edge scores)
      if (best_path.first == -1 || newcost.cost < best_path.second) {
        best_path.first = (es->set() == EdgeSet::kTemporary) ? es->index() : edgelabels_.size();
        best_path.second = newcost.cost;
      }
    }

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (es->set() == EdgeSet::kTemporary) {
      EdgeLabel& lab = edgelabels_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_->decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, has_time_restrictions);
      }
      continue;
    }

    // If this is a destination edge the A* heuristic is 0. Otherwise the
    // sort cost (with A* heuristic) is found using the lat,lng at the
    // end node of the directed edge.
    float dist = 0.0f;
    float sortcost = newcost.cost;
    if (p == destinations_.end()) {
      const GraphTile* t2 =
          directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (t2 == nullptr) {
        continue;
      }
      sortcost += astarheuristic_.Get(t2->get_node_ll(directededge->endnode()), dist);
    }

    // Add to the adjacency list and edge labels.
    uint32_t idx = edgelabels_.size();
    edgelabels_.emplace_back(pred_idx, edgeid, directededge, newcost, sortcost, dist, mode_, 0);
    *es = {EdgeSet::kTemporary, idx};
    adjacencylist_->add(idx);
  }

  // Handle transitions - expand from the end node of each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      if (trans->up()) {
        hierarchy_limits_[node.level()].up_transition_count++;
        ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true, destination, best_path);
      } else if (!hierarchy_limits_[trans->endnode().level()].StopExpanding(pred.distance())) {
        ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true, destination, best_path);
      }
    }
  }
}

// Calculate best path. This method is single mode, not time-dependent.
std::vector<std::vector<PathInfo>>
AStarPathAlgorithm::GetBestPath(valhalla::Location& origin,
                                valhalla::Location& destination,
                                GraphReader& graphreader,
                                const std::shared_ptr<DynamicCost>* mode_costing,
                                const TravelMode mode,
                                const Options& options) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  // Note: because we can correlate to more than one place for a given PathLocation
  // using edges.front here means we are only setting the heuristics to one of them
  // alternate paths using the other correlated points to may be harder to find
  midgard::PointLL origin_new(origin.path_edges(0).ll().lng(), origin.path_edges(0).ll().lat());
  midgard::PointLL destination_new(destination.path_edges(0).ll().lng(),
                                   destination.path_edges(0).ll().lat());
  Init(origin_new, destination_new);
  float mindist = astarheuristic_.GetDistance(origin_new);

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  uint32_t density = SetDestination(graphreader, destination);
  SetOrigin(graphreader, origin, destination);

  // Update hierarchy limits
  ModifyHierarchyLimits(mindist, density);

  // Find shortest path
  uint32_t nc = 0; // Count of iterations with no convergence
                   // towards destination
  std::pair<int32_t, float> best_path = std::make_pair(-1, 0.0f);
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
    uint32_t predindex = adjacencylist_->pop();
    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " + std::to_string(edgelabels_.size()));
      return {};
    }

    // Copy the EdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    EdgeLabel pred = edgelabels_[predindex];
    if (destinations_.find(pred.edgeid()) != destinations_.end()) {
      // Check if a trivial path. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        if (IsTrivial(pred.edgeid(), origin, destination)) {
          return {FormPath(predindex)};
        }
      } else {
        return {FormPath(predindex)};
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
        return {FormPath(best_path.first)};
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

    // Expand forward from the end node of the predecessor edge.
    ExpandForward(graphreader, pred.endnode(), pred, predindex, false, destination, best_path);
  }
  return {}; // Should never get here
}

// Add an edge at the origin to the adjacency list
void AStarPathAlgorithm::SetOrigin(GraphReader& graphreader,
                                   valhalla::Location& origin,
                                   const valhalla::Location& destination) {
  // Only skip inbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.path_edges().begin(), origin.path_edges().end(),
                [&has_other_edges](const valhalla::Location::PathEdge& e) {
                  has_other_edges = has_other_edges || !e.end_node();
                });

  // Check if the origin edge matches a destination edge at the node.
  auto trivial_at_node = [this, &destination](const valhalla::Location::PathEdge& edge) {
    auto p = destinations_.find(edge.graph_id());
    if (p != destinations_.end()) {
      for (const auto& destination_edge : destination.path_edges()) {
        if (destination_edge.graph_id() == edge.graph_id()) {
          return true;
        }
      }
    }
    return false;
  };

  // Iterate through edges and add to adjacency list
  const NodeInfo* nodeinfo = nullptr;
  const NodeInfo* closest_ni = nullptr;
  for (const auto& edge : origin.path_edges()) {
    // If origin is at a node - skip any inbound edge (dist = 1) unless the
    // destination is also at the same end node (trivial path).
    if (has_other_edges && edge.end_node() && !trivial_at_node(edge)) {
      continue;
    }

    // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
    GraphId edgeid(edge.graph_id());
    if (costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
      continue;
    }

    // Get the directed edge
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the tile at the end node. Skip if tile not found as we won't be
    // able to expand from this origin edge.
    const GraphTile* endtile = graphreader.GetGraphTile(directededge->endnode());
    if (endtile == nullptr) {
      continue;
    }

    // Get cost
    nodeinfo = endtile->node(directededge->endnode());
    Cost cost = costing_->EdgeCost(directededge, tile) * (1.0f - edge.percent_along());
    float dist = astarheuristic_.GetDistance(endtile->get_node_ll(directededge->endnode()));

    // We need to penalize this location based on its score (distance in meters from input)
    // We assume the slowest speed you could travel to cover that distance to start/end the route
    // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
    // Perhaps need to adjust score?
    cost.cost += edge.distance();

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge) if the
    // destination is in a forward direction along the edge. Add back in
    // the edge score/penalty to account for destination edges farther from
    // the input location lat,lon.
    auto p = destinations_.find(edgeid);
    if (p != destinations_.end()) {
      if (IsTrivial(edgeid, origin, destination)) {
        // Find the destination edge and update cost.
        for (const auto& destination_edge : destination.path_edges()) {
          if (destination_edge.graph_id() == edgeid) {
            // a trivial route passes along a single edge, meaning that the
            // destination point must be on this edge, and so the distance
            // remaining must be zero.
            GraphId id(destination_edge.graph_id());
            const DirectedEdge* dest_diredge = tile->directededge(id);
            Cost dest_cost =
                costing_->EdgeCost(dest_diredge, tile, 0) * (1.0f - destination_edge.percent_along());
            cost.secs -= p->second.secs;
            cost.cost -= dest_cost.cost;
            cost.cost += destination_edge.distance();
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

    // Add EdgeLabel to the adjacency list (but do not set its status).
    // Set the predecessor edge index to invalid to indicate the origin
    // of the path.
    uint32_t d = static_cast<uint32_t>(directededge->length() * (1.0f - edge.percent_along()));
    EdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost, sortcost, dist, mode_, d);
    // Set the origin flag
    edge_label.set_origin();

    // Add EdgeLabel to the adjacency list
    uint32_t idx = edgelabels_.size();
    edgelabels_.push_back(std::move(edge_label));
    adjacencylist_->add(idx);

    // DO NOT SET EdgeStatus - it messes up trivial paths with oneways
  }

  // Set the origin timezone
  if (closest_ni != nullptr && origin.has_date_time() && origin.date_time() == "current") {
    origin.set_date_time(
        DateTime::iso_date_time(DateTime::get_tz_db().from_index(closest_ni->timezone())));
  }
}

// Add a destination edge
uint32_t AStarPathAlgorithm::SetDestination(GraphReader& graphreader,
                                            const valhalla::Location& dest) {
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

    // Disallow any user avoided edges if the avoid location is behind the destination along the edge
    GraphId edgeid(edge.graph_id());
    if (costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
      continue;
    }

    // Keep the cost to traverse the partial distance for the remainder of the edge. This cost
    // is subtracted from the total cost up to the end of the destination edge.
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);
    destinations_[edge.graph_id()] =
        costing_->EdgeCost(directededge, tile) * (1.0f - edge.percent_along());

    // Edge score (penalty) is handled within GetPath. Do not add score here.

    // Get the tile relative density
    density = tile->header()->density();
  }
  return density;
}

// Form the path from the adjacency list.
std::vector<PathInfo> AStarPathAlgorithm::FormPath(const uint32_t dest) {
  // Metrics to track
  LOG_DEBUG("path_cost::" + std::to_string(edgelabels_[dest].cost().cost));
  LOG_DEBUG("path_iterations::" + std::to_string(edgelabels_.size()));

  // Work backwards from the destination
  std::vector<PathInfo> path;
  for (auto edgelabel_index = dest; edgelabel_index != kInvalidLabel;
       edgelabel_index = edgelabels_[edgelabel_index].predecessor()) {
    const EdgeLabel& edgelabel = edgelabels_[edgelabel_index];
    path.emplace_back(edgelabel.mode(), edgelabel.cost().secs, edgelabel.edgeid(), 0,
                      edgelabel.cost().cost, edgelabel.has_time_restriction());

    // Check if this is a ferry
    if (edgelabel.use() == Use::kFerry) {
      has_ferry_ = true;
    }
  }

  // Reverse the list and return
  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace thor
} // namespace valhalla
