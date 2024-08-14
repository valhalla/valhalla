#include "thor/astar_bss.h"
#include "baldr/datetime.h"
#include "midgard/logging.h"
#include <algorithm>

using namespace valhalla::baldr;
using namespace valhalla::sif;

// TODO: make a class that extends std::exception, with messages and
// error codes and return the appropriate error codes

namespace {

static travel_mode_t get_other_travel_mode(const travel_mode_t current_mode) {
  static const auto bss_modes =
      std::vector<travel_mode_t>{travel_mode_t::kPedestrian, travel_mode_t::kBicycle};
  return bss_modes[static_cast<size_t>(current_mode == travel_mode_t::kPedestrian)];
}
} // namespace

namespace valhalla {
namespace thor {
// Number of iterations to allow with no convergence to the destination
constexpr uint32_t kMaxIterationsWithoutConvergence = 200000;

// Default constructor
AStarBSSAlgorithm::AStarBSSAlgorithm(const boost::property_tree::ptree& config)
    : PathAlgorithm(config.get<uint32_t>("max_reserved_labels_count_astar",
                                         kInitialEdgeLabelCountAstar),
                    config.get<bool>("clear_reserved_memory", false)) {
  mode_ = travel_mode_t::kDrive;
}

// Destructor
AStarBSSAlgorithm::~AStarBSSAlgorithm() {
}

// Clear the temporary information generated during path construction.
void AStarBSSAlgorithm::Clear() {
  // Reduce edge labels capacity if it's more than limit
  auto reservation = clear_reserved_memory_ ? 0 : max_reserved_labels_count_;
  if (edgelabels_.size() > reservation) {
    edgelabels_.resize(reservation);
    edgelabels_.shrink_to_fit();
  }

  // Clear the edge labels and destination list. Reset the adjacency list
  // and clear edge status.
  edgelabels_.clear();
  destinations_.clear();
  adjacencylist_.clear();
  pedestrian_edgestatus_.clear();
  bicycle_edgestatus_.clear();

  // Set the ferry flag to false
  has_ferry_ = false;
}

// Initialize prior to finding best path
void AStarBSSAlgorithm::Init(const midgard::PointLL& origll, const midgard::PointLL& destll) {
  LOG_TRACE("Orig LL = " + std::to_string(origll.lat()) + "," + std::to_string(origll.lng()));
  LOG_TRACE("Dest LL = " + std::to_string(destll.lat()) + "," + std::to_string(destll.lng()));

  // Set the destination and cost factor in the A* heuristic
  // In order to maintain the underestimation, we chose the min value of those two factors
  // At the same time, we also want to get the factors as large as possible to optimize the
  // performance
  // TODO: Any better idea to normalize the A* heuristic cost?
  auto common_astar_cost =
      std::min(pedestrian_costing_->AStarCostFactor(), bicycle_costing_->AStarCostFactor());

  pedestrian_astarheuristic_.Init(destll, common_astar_cost);
  bicycle_astarheuristic_.Init(destll, common_astar_cost);

  // Get the initial cost based on A* heuristic from origin
  float mincost =
      std::min(bicycle_astarheuristic_.Get(origll), pedestrian_astarheuristic_.Get(origll));

  // Reserve size for edge labels - do this here rather than in constructor so
  // to limit how much extra memory is used for persistent objects.
  // TODO - reserve based on estimate based on distance and route type.
  edgelabels_.reserve(max_reserved_labels_count_);

  // Construct adjacency list, clear edge status.
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = std::max(pedestrian_costing_->UnitSize(), bicycle_costing_->UnitSize());
  float range = kBucketCount * bucketsize;
  adjacencylist_.reuse(mincost, range, bucketsize, &edgelabels_);
  pedestrian_edgestatus_.clear();
  bicycle_edgestatus_.clear();
}

// Expand from the node along the forward search path. Immediately expands
// from the end node of any transition edge (so no transition edges are added
// to the adjacency list or EdgeLabel list). Does not expand transition
// edges if from_transition is false.
void AStarBSSAlgorithm::ExpandForward(GraphReader& graphreader,
                                      const GraphId& node,
                                      const BDEdgeLabel& pred,
                                      const uint32_t pred_idx,
                                      const bool from_transition,
                                      const bool from_bss,
                                      const sif::travel_mode_t mode,
                                      const valhalla::Location& destination,
                                      std::pair<int32_t, float>& best_path) {
  const auto& current_costing =
      (mode == travel_mode_t::kPedestrian ? pedestrian_costing_ : bicycle_costing_);
  const auto& current_heuristic =
      (mode == travel_mode_t::kPedestrian ? pedestrian_astarheuristic_ : bicycle_astarheuristic_);

  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  graph_tile_ptr tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }
  const NodeInfo* nodeinfo = tile->node(node);
  if (!current_costing->Allowed(nodeinfo)) {
    return;
  }

  // Expand from end node.
  uint32_t shortcuts = 0;
  uint32_t max_shortcut_length = static_cast<uint32_t>(pred.distance() * 0.5f);
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());

  EdgeStatusInfo* current_es =
      (mode == travel_mode_t::kPedestrian ? pedestrian_edgestatus_ : bicycle_edgestatus_)
          .GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());

  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++directededge, ++edgeid, ++current_es) {
    // Use regular edges while still expanding on the next level since we can still
    // transition down to that level. If using a shortcut, set the shortcuts mask.
    // Skip if this is a regular edge superseded by a shortcut.
    if (directededge->is_shortcut()) {
      if (pred.distance() < 10000.0f || directededge->length() > max_shortcut_length) {
        continue;
      }
      shortcuts |= directededge->shortcut();
    } else if (shortcuts & directededge->superseded()) {
      continue;
    }

    // Skip this edge if permanently labeled (best path already found to this
    // directed edge), if no access is allowed to this edge (based on costing method),
    // or if a complex restriction exists.
    uint8_t has_time_restrictions = -1;
    const bool is_dest = destinations_.find(edgeid) != destinations_.cend();
    if (current_es->set() == EdgeSet::kPermanent ||
        !current_costing->Allowed(directededge, is_dest, pred, tile, edgeid, 0, 0,
                                  has_time_restrictions) ||
        current_costing->Restricted(directededge, pred, edgelabels_, tile, edgeid, true)) {
      continue;
    }

    auto edge_cost = current_costing->EdgeCost(directededge, tile);
    Cost normalized_edge_cost = {edge_cost.cost * current_costing->GetModeFactor(), edge_cost.secs};
    auto transition_cost = current_costing->TransitionCost(directededge, nodeinfo, pred);

    // Compute the cost to the end of this edge
    Cost newcost = pred.cost() + normalized_edge_cost + transition_cost;

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge).
    auto p = mode != travel_mode_t::kPedestrian ? destinations_.end() : destinations_.find(edgeid);
    if (p != destinations_.end()) {
      // Subtract partial cost and time
      newcost -= p->second;

      // Find the destination edge and update cost to include the edge score.
      // Note - with high edge scores the convergence test fails some routes
      // so reduce the edge score.
      for (const auto& destination_edge : destination.correlation().edges()) {
        if (destination_edge.graph_id() == edgeid) {
          newcost.cost += destination_edge.distance();
        }
      }
      newcost.cost = std::max(0.0f, newcost.cost);

      // Mark this as the best connection if that applies. This allows
      // a path to be formed even if the convergence test fails (can
      // happen with large edge scores)
      if (best_path.first == -1 || newcost.cost < best_path.second) {
        best_path.first =
            (current_es->set() == EdgeSet::kTemporary) ? current_es->index() : edgelabels_.size();
        best_path.second = newcost.cost;
      }
    }

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (current_es->set() == EdgeSet::kTemporary) {
      auto& lab = edgelabels_[current_es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_.decrease(current_es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, transition_cost, has_time_restrictions);
      }
      continue;
    }

    // If this is a destination edge the A* heuristic is 0. Otherwise the
    // sort cost (with A* heuristic) is found using the lat,lng at the
    // end node of the directed edge.
    float dist = 0.0f;
    float sortcost = newcost.cost;

    if (p == destinations_.end()) {
      graph_tile_ptr t2 =
          directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (t2 == nullptr) {
        continue;
      }
      sortcost += current_heuristic.Get(t2->get_node_ll(directededge->endnode()), dist);
    }

    // Add to the adjacency list and edge labels.
    uint32_t idx = edgelabels_.size();
    edgelabels_.emplace_back(pred_idx, edgeid, GraphId(), directededge, newcost, sortcost, dist, mode,
                             transition_cost, false, false, false, InternalTurn::kNoTurn,
                             baldr::kInvalidRestriction);
    *current_es = {EdgeSet::kTemporary, idx};
    adjacencylist_.add(idx);
  }

  if (!from_bss && nodeinfo->type() == NodeType::kBikeShare) {
    auto other_mode = get_other_travel_mode(pred.mode());
    auto from_bss = true;
    ExpandForward(graphreader, node, pred, pred_idx, from_transition, from_bss, other_mode,
                  destination, best_path);
  }
  // Handle transitions - expand from the end node of each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true, from_bss, mode, destination,
                    best_path);
    }
  }
}

// Calculate best path. This method is single mode, not time-dependent.
std::vector<std::vector<PathInfo>>
AStarBSSAlgorithm::GetBestPath(valhalla::Location& origin,
                               valhalla::Location& destination,
                               GraphReader& graphreader,
                               const sif::mode_costing_t& mode_costing,
                               const travel_mode_t mode,
                               const Options&) {
  // Set the mode and costing
  mode_ = mode;
  pedestrian_costing_ = mode_costing[static_cast<uint32_t>(travel_mode_t::kPedestrian)];
  bicycle_costing_ = mode_costing[static_cast<uint32_t>(travel_mode_t::kBicycle)];
  travel_type_ = pedestrian_costing_->travel_type();

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  // Note: because we can correlate to more than one place for a given PathLocation
  // using edges.front here means we are only setting the heuristics to one of them
  // alternate paths using the other correlated points to may be harder to find
  midgard::PointLL origin_new(origin.correlation().edges(0).ll().lng(),
                              origin.correlation().edges(0).ll().lat());
  midgard::PointLL destination_new(destination.correlation().edges(0).ll().lng(),
                                   destination.correlation().edges(0).ll().lat());
  Init(origin_new, destination_new);
  float mindist = pedestrian_astarheuristic_.GetDistance(origin_new);

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  SetDestination(graphreader, destination);
  SetOrigin(graphreader, origin, destination);

  // Find shortest path
  uint32_t nc = 0; // Count of iterations with no convergence
                   // towards destination
  std::pair<int32_t, float> best_path = std::make_pair(-1, 0.0f);
  size_t n = 0;
  while (true) {
    // Allow this process to be aborted
    if (interrupt && (++n % kInterruptIterationsInterval) == 0) {
      (*interrupt)();
    }

    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    const uint32_t predindex = adjacencylist_.pop();
    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " + std::to_string(edgelabels_.size()));
      return {};
    }

    // Copy the EdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    auto pred = edgelabels_[predindex];

    if (destinations_.find(pred.edgeid()) != destinations_.end() &&
        pred.mode() == travel_mode_t::kPedestrian) {
      // Check if a trivial path. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        if (IsTrivial(pred.edgeid(), origin, destination)) {
          return {FormPath(graphreader, predindex)};
        }
      } else {
        return {FormPath(graphreader, predindex)};
      }
    }

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge (this will allow loops/around the block cases)
    if (!pred.origin() && pred.mode() == travel_mode_t::kPedestrian) {
      pedestrian_edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    if (!pred.origin() && pred.mode() == travel_mode_t::kBicycle) {
      bicycle_edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent);
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

    // Expand forward from the end node of the predecessor edge.
    ExpandForward(graphreader, pred.endnode(), pred, predindex, false, false, pred.mode(),
                  destination, best_path);
  }
  return {}; // Should never get here
}

// Add an edge at the origin to the adjacency list
void AStarBSSAlgorithm::SetOrigin(GraphReader& graphreader,
                                  valhalla::Location& origin,
                                  const valhalla::Location& destination) {

  // Only skip inbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.correlation().edges().begin(), origin.correlation().edges().end(),
                [&has_other_edges](const auto& e) {
                  has_other_edges = has_other_edges || !e.end_node();
                });

  // Check if the origin edge matches a destination edge at the node.
  auto trivial_at_node = [this, &destination](const auto& edge) {
    auto p = destinations_.find(edge.graph_id());
    if (p != destinations_.end()) {
      for (const auto& destination_edge : destination.correlation().edges()) {
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
  for (const auto& edge : origin.correlation().edges()) {
    // If origin is at a node - skip any inbound edge (dist = 1) unless the
    // destination is also at the same end node (trivial path).
    if (has_other_edges && edge.end_node() && !trivial_at_node(edge)) {
      continue;
    }

    // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
    GraphId edgeid(edge.graph_id());
    if (pedestrian_costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
      continue;
    }

    // Get the directed edge
    graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the tile at the end node. Skip if tile not found as we won't be
    // able to expand from this origin edge.
    graph_tile_ptr endtile = graphreader.GetGraphTile(directededge->endnode());
    if (endtile == nullptr) {
      continue;
    }

    // Get cost
    nodeinfo = endtile->node(directededge->endnode());
    Cost cost = pedestrian_costing_->EdgeCost(directededge, tile) * (1.0f - edge.percent_along());
    float dist =
        pedestrian_astarheuristic_.GetDistance(endtile->get_node_ll(directededge->endnode()));

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
        for (const auto& destination_edge : destination.correlation().edges()) {
          if (destination_edge.graph_id() == edgeid) {
            // a trivial route passes along a single edge, meaning that the
            // destination point must be on this edge, and so the distance
            // remaining must be zero.
            GraphId id(destination_edge.graph_id());
            const DirectedEdge* dest_diredge = tile->directededge(id);
            Cost dest_cost = pedestrian_costing_->EdgeCost(dest_diredge, tile) *
                             (1.0f - destination_edge.percent_along());
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
    float sortcost = cost.cost + pedestrian_astarheuristic_.Get(dist);

    // Add EdgeLabel to the adjacency list (but do not set its status).
    // Set the predecessor edge index to invalid to indicate the origin
    // of the path.
    uint32_t d = static_cast<uint32_t>(directededge->length() * (1.0f - edge.percent_along()));
    BDEdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost, sortcost, dist,
                           travel_mode_t::kPedestrian, baldr::kInvalidRestriction, false, false,
                           sif::InternalTurn::kNoTurn);
    // Set the origin flag and path distance
    edge_label.set_origin();
    edge_label.set_path_distance(d);

    // Add EdgeLabel to the adjacency list
    uint32_t idx = edgelabels_.size();
    edgelabels_.push_back(std::move(edge_label));
    adjacencylist_.add(idx);

    // DO NOT SET EdgeStatus - it messes up trivial paths with oneways
  }

  // Set the origin timezone
  if (closest_ni != nullptr && !origin.date_time().empty() && origin.date_time() == "current") {
    origin.set_date_time(
        DateTime::iso_date_time(DateTime::get_tz_db().from_index(closest_ni->timezone())));
  }
}

// Add a destination edge
void AStarBSSAlgorithm::SetDestination(GraphReader& graphreader, const valhalla::Location& dest) {

  // Only skip outbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(dest.correlation().edges().begin(), dest.correlation().edges().end(),
                [&has_other_edges](const auto& e) {
                  has_other_edges = has_other_edges || !e.begin_node();
                });

  for (const auto& edge : dest.correlation().edges()) {
    // If destination is at a node skip any outbound edges
    if (has_other_edges && edge.begin_node()) {
      continue;
    }

    // Disallow any user avoided edges if the avoid location is behind the destination along the edge
    GraphId edgeid(edge.graph_id());
    if (pedestrian_costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
      continue;
    }

    // Keep the cost to traverse the partial distance for the remainder of the edge. This cost
    // is subtracted from the total cost up to the end of the destination edge.
    auto tile = graphreader.GetGraphTile(edgeid);
    assert(tile);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    destinations_[edge.graph_id()] =
        pedestrian_costing_->EdgeCost(directededge, tile) * (1.0f - edge.percent_along());

    // Edge score (penalty) is handled within GetPath. Do not add score here.
  }
  return;
}

// Form the path from the adjacency list.
std::vector<PathInfo> AStarBSSAlgorithm::FormPath(baldr::GraphReader& graphreader,
                                                  const uint32_t dest) {
  // Metrics to track
  LOG_DEBUG("path_cost::" + std::to_string(edgelabels_[dest].cost().cost));
  LOG_DEBUG("path_iterations::" + std::to_string(edgelabels_.size()));

  // Work backwards from the destination
  std::vector<PathInfo> path;
  travel_mode_t old = travel_mode_t::kPedestrian;
  [[maybe_unused]] int mode_change_count = 0;

  for (auto edgelabel_index = dest; edgelabel_index != kInvalidLabel;
       edgelabel_index = edgelabels_[edgelabel_index].predecessor()) {
    const auto& edgelabel = edgelabels_[edgelabel_index];
    path.emplace_back(edgelabel.mode(), edgelabel.cost(), edgelabel.edgeid(), 0,
                      edgelabel.path_distance(), edgelabel.restriction_idx(),
                      edgelabel.transition_cost());

    graph_tile_ptr tile = graphreader.GetGraphTile(edgelabel.edgeid());

    // Check if this is a ferry
    if (edgelabel.use() == Use::kFerry) {
      has_ferry_ = true;
    }

    if (edgelabel.mode() != old) {
      old = edgelabel.mode();
      mode_change_count++;
    }
  }
  assert(mode_change_count == 0 || mode_change_count == 2);

  // Reverse the list and return
  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace thor
} // namespace valhalla
