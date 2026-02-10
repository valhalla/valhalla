#include "baldr/datetime.h"
#include "midgard/logging.h"
#include "proto_conversions.h"
#include "sif/hierarchylimits.h"
#include "thor/multimodal_astar.h"

#include <algorithm>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {
// Number of iterations to allow with no convergence to the destination
constexpr uint32_t kMaxIterationsWithoutConvergence = 200000;

// Default constructor
MultimodalAStar::MultimodalAStar(const boost::property_tree::ptree& config)
    : PathAlgorithm(config.get<uint32_t>("max_reserved_labels_count_astar",
                                         kInitialEdgeLabelCountAstar),
                    config.get<bool>("clear_reserved_memory", false)) {
  mode_ = travel_mode_t::kDrive;
}

// Destructor
MultimodalAStar::~MultimodalAStar() {
}

// Clear the temporary information generated during path construction.
void MultimodalAStar::Clear() {
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
  std::for_each(edge_status_.begin(), edge_status_.end(), [](auto& status) { status.clear(); });

  // Set the ferry flag to false
  has_ferry_ = false;
}

// Initialize prior to finding best path
void MultimodalAStar::Init(const midgard::PointLL& origll, const midgard::PointLL& destll) {
  LOG_TRACE("Orig LL = " + std::to_string(origll.lat()) + "," + std::to_string(origll.lng()));
  LOG_TRACE("Dest LL = " + std::to_string(destll.lat()) + "," + std::to_string(destll.lng()));

  // Set the destination and cost factor in the A* heuristic
  // In order to maintain the underestimation, we chose the min value of those two factors
  // At the same time, we also want to get the factors as large as possible to optimize the
  // performance
  // TODO: Any better idea to normalize the A* heuristic cost?
  auto common_astar_cost =
      std::min(start_costing_->AStarCostFactor(), other_costing_->AStarCostFactor());

  start_astarheuristic_.Init(destll, common_astar_cost);
  end_astarheuristic_.Init(destll, common_astar_cost);

  // Get the initial cost based on A* heuristic from origin
  float mincost = std::min(start_astarheuristic_.Get(origll), end_astarheuristic_.Get(origll));

  // Reserve size for edge labels - do this here rather than in constructor so
  // to limit how much extra memory is used for persistent objects.
  // TODO - reserve based on estimate based on distance and route type.
  edgelabels_.reserve(max_reserved_labels_count_);

  // Construct adjacency list, clear edge status.
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = std::max(start_costing_->UnitSize(), other_costing_->UnitSize());
  float range = kBucketCount * bucketsize;
  adjacencylist_.reuse(mincost, range, bucketsize, &edgelabels_);
  std::for_each(edge_status_.begin(), edge_status_.end(), [](auto& status) { status.clear(); });
  hierarchy_limits_[0] = start_costing_->GetHierarchyLimits();
  hierarchy_limits_[1] = other_costing_->GetHierarchyLimits();
}

// Expand from the node along the forward search path. Immediately expands
// from the end node of any transition edge (so no transition edges are added
// to the adjacency list or EdgeLabel list). Does not expand transition
// edges if from_transition is false.
void MultimodalAStar::ExpandForward(GraphReader& graphreader,
                                    const GraphId& node,
                                    const BDEdgeLabel& pred,
                                    const uint32_t pred_idx,
                                    const bool from_transition,
                                    const bool from_mode_change,
                                    const sif::travel_mode_t current_mode,
                                    const valhalla::Location& destination,
                                    std::pair<int32_t, float>& best_path,
                                    size_t mode_transition_count) {
  const auto& current_costing = (current_mode == start_mode_ ? start_costing_ : other_costing_);
  const auto& current_heuristic =
      (current_mode == start_mode_ ? start_astarheuristic_ : end_astarheuristic_);

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

  EdgeStatus& current_es = (edge_status_[static_cast<size_t>(current_mode != start_mode_)]);

  auto meta = EdgeMetadata::make(node, nodeinfo, tile, current_es);

  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++meta) {
    // Use regular edges while still expanding on the next level since we can still
    // transition down to that level. If using a shortcut, set the shortcuts mask.
    // Skip if this is a regular edge superseded by a shortcut.
    if (meta.edge->is_shortcut()) {
      auto& hierarchy_limits = hierarchy_limits_[static_cast<size_t>(current_mode != start_mode_)];
      if ((current_costing->UseHierarchyLimits() &&
           StopExpanding(hierarchy_limits[meta.edge_id.level()], pred.distance())) ||
          // this preserves the previous logic in place for using shortcuts in bikeshare mode
          // for backwards compatibility
          (pred.distance() < 10000.0f || meta.edge->length() > max_shortcut_length)) {
        continue;
      }
      shortcuts |= meta.edge->shortcut();
    } else if (shortcuts & meta.edge->superseded()) {
      continue;
    }

    // Skip this edge if permanently labeled (best path already found to this
    // directed edge), if no access is allowed to this edge (based on costing method),
    // or if a complex restriction exists.
    uint8_t has_time_restrictions = -1;
    uint8_t destonly_restriction_mask = 0;
    const bool is_dest = destinations_.find(meta.edge_id) != destinations_.cend();
    if (meta.edge_status->set() == EdgeSet::kPermanent ||
        !current_costing->Allowed(meta.edge, is_dest, pred, tile, meta.edge_id, 0, 0,
                                  has_time_restrictions, destonly_restriction_mask) ||
        current_costing->Restricted(meta.edge, pred, edgelabels_, tile, meta.edge_id, true)) {
      continue;
    }

    auto edge_cost = current_costing->EdgeCost(meta.edge, meta.edge_id, tile);
    Cost normalized_edge_cost = {edge_cost.cost * current_costing->GetModeFactor(), edge_cost.secs};
    auto reader_getter = [&graphreader]() { return baldr::LimitedGraphReader(graphreader); };
    auto transition_cost =
        current_costing->TransitionCost(meta.edge, nodeinfo, pred, tile, reader_getter);

    // Compute the cost to the end of this edge
    Cost newcost = pred.cost() + normalized_edge_cost + transition_cost;

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge).
    auto p = current_mode == end_mode_ ? destinations_.find(meta.edge_id) : destinations_.end();
    if (p != destinations_.end()) {
      // Subtract partial cost and time
      newcost -= p->second;

      // Find the destination edge and update cost to include the edge score.
      // Note - with high edge scores the convergence test fails some routes
      // so reduce the edge score.
      for (const auto& destination_edge : destination.correlation().edges()) {
        if (destination_edge.graph_id() == meta.edge_id) {
          newcost.cost += destination_edge.distance();
        }
      }
      newcost.cost = std::max(0.0f, newcost.cost);

      // Mark this as the best connection if that applies. This allows
      // a path to be formed even if the convergence test fails (can
      // happen with large edge scores)
      if (best_path.first == -1 || newcost.cost < best_path.second) {
        best_path.first = (meta.edge_status->set() == EdgeSet::kTemporary) ? meta.edge_status->index()
                                                                           : edgelabels_.size();
        best_path.second = newcost.cost;
      }
    }

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (meta.edge_status->set() == EdgeSet::kTemporary) {
      auto& lab = edgelabels_[meta.edge_status->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_.decrease(meta.edge_status->index(), newsortcost);
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
          meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
      if (t2 == nullptr) {
        continue;
      }
      sortcost += current_heuristic.Get(t2->get_node_ll(meta.edge->endnode()), dist);
    }

    // Add to the adjacency list and edge labels.
    uint32_t idx = edgelabels_.size();
    edgelabels_.emplace_back(pred_idx, meta.edge_id, GraphId(), meta.edge, newcost, sortcost, dist,
                             current_mode, transition_cost, false, false, false,
                             InternalTurn::kNoTurn, baldr::kInvalidRestriction);
    *meta.edge_status = {EdgeSet::kTemporary, idx};
    adjacencylist_.add(idx);
    if (expansion_callback_) {
      expansion_callback_(graphreader, meta.edge_id, GraphId(pred.edgeid()), name(),
                          Expansion_EdgeStatus_reached, newcost.secs, dist, newcost.cost,
                          Expansion_ExpansionType::Expansion_ExpansionType_forward, kNoFlowMask,
                          static_cast<TravelMode>(static_cast<size_t>(current_mode)));
    }
  }

  if (!from_mode_change && nodeinfo->type() == mode_transition_ &&
      // this greatly reduces the expansion on larger routes by avoiding pedestrian
      // mode changes further away than the user is willing to walk
      (pred.distance() <= max_walking_distance_) && mode_transition_count < max_transitions_) {
    auto new_mode = current_mode == start_mode_ ? other_mode_ : start_mode_;
    ExpandForward(graphreader, node, pred, pred_idx, from_transition, /*from_mode_change=*/true,
                  new_mode, destination, best_path, mode_transition_count + 1);
  }
  // Handle transitions - expand from the end node of each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    auto& hierarchy_limits = hierarchy_limits_[static_cast<size_t>(current_mode != start_mode_)];
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      if (current_costing->UseHierarchyLimits() &&
          (!trans->up() &&
           StopExpanding(hierarchy_limits[trans->endnode().level()], pred.distance()))) {
        continue;
      }
      ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true, from_mode_change,
                    current_mode, destination, best_path, mode_transition_count);
    }
  }
}

// Calculate best path. This method is single mode, not time-dependent.
std::vector<std::vector<PathInfo>>
MultimodalAStar::GetBestPath(valhalla::Location& origin,
                             valhalla::Location& destination,
                             GraphReader& graphreader,
                             const sif::mode_costing_t& mode_costing,
                             const travel_mode_t mode,
                             const Options& options) {

  auto costing = options.costing_type();
  start_mode_ = mode;
  mode_ = mode;

  switch (costing) {
    case Costing::bikeshare:
      end_mode_ = travel_mode_t::kPedestrian;
      other_mode_ = travel_mode_t::kBicycle;
      mode_transition_ = baldr::NodeType::kBikeShare;
      max_walking_distance_ = std::numeric_limits<uint32_t>::max();
      max_transitions_ = 2;
      break;
    case Costing::auto_pedestrian:
      end_mode_ = travel_mode_t::kPedestrian;
      other_mode_ = travel_mode_t::kPedestrian;
      mode_transition_ = baldr::NodeType::kParking;
      // in order to not break bikeshare mode, only enable
      // this for auto_pedestrian for now
      max_walking_distance_ = options.costings()
                                  .find(Costing::pedestrian)
                                  ->second.options()
                                  .transit_start_end_max_distance();
      max_transitions_ = 2;
      break;
    default:
      throw std::runtime_error("Costing not handled by this algorithm: " +
                               Costing_Enum_Name(costing));
  }

  // Set the mode and costing
  start_costing_ = mode_costing[static_cast<uint32_t>(start_mode_)];
  other_costing_ = mode_costing[static_cast<uint32_t>(other_mode_)];

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  // Note: because we can correlate to more than one place for a given PathLocation
  // using edges.front here means we are only setting the heuristics to one of them
  // alternate paths using the other correlated points to may be harder to find
  midgard::PointLL origin_new(origin.correlation().edges(0).ll().lng(),
                              origin.correlation().edges(0).ll().lat());
  midgard::PointLL destination_new(destination.correlation().edges(0).ll().lng(),
                                   destination.correlation().edges(0).ll().lat());
  Init(origin_new, destination_new);
  float mindist = start_astarheuristic_.GetDistance(origin_new);

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

    if (pred.mode() == end_mode_ && destinations_.find(pred.edgeid()) != destinations_.end()) {
      // Check if a trivial path. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        if (IsTrivial(pred.edgeid(), origin, destination)) {
          if (expansion_callback_) {
            expansion_callback_(graphreader, pred.edgeid(),
                                GraphId(pred.predecessor() == kInvalidLabel
                                            ? kInvalidGraphId
                                            : edgelabels_[pred.predecessor()].edgeid()),
                                name(), Expansion_EdgeStatus_connected, pred.cost().secs,
                                pred.distance(), pred.cost().cost,
                                Expansion_ExpansionType::Expansion_ExpansionType_forward, kNoFlowMask,
                                static_cast<TravelMode>(static_cast<size_t>(pred.mode())));
          }
          return {FormPath(graphreader, predindex)};
        }
      } else {
        if (expansion_callback_) {
          expansion_callback_(graphreader, pred.edgeid(),
                              GraphId(pred.predecessor() == kInvalidLabel
                                          ? kInvalidGraphId
                                          : edgelabels_[pred.predecessor()].edgeid()),
                              name(), Expansion_EdgeStatus_connected, pred.cost().secs,
                              pred.distance(), pred.cost().cost,
                              Expansion_ExpansionType::Expansion_ExpansionType_forward, kNoFlowMask,
                              static_cast<TravelMode>(static_cast<size_t>(pred.mode())));
        }
        return {FormPath(graphreader, predindex)};
      }
    }

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge (this will allow loops/around the block cases)
    if (!pred.origin()) {
      edge_status_[static_cast<size_t>(pred.mode() != start_mode_)].Update(pred.edgeid(),
                                                                           EdgeSet::kPermanent);
    }
    if (expansion_callback_) {
      expansion_callback_(graphreader, pred.edgeid(),
                          GraphId(pred.predecessor() == kInvalidLabel
                                      ? kInvalidGraphId
                                      : edgelabels_[pred.predecessor()].edgeid()),
                          name(), Expansion_EdgeStatus_settled, pred.cost().secs, pred.distance(),
                          pred.cost().cost, Expansion_ExpansionType::Expansion_ExpansionType_forward,
                          kNoFlowMask, static_cast<TravelMode>(static_cast<size_t>(pred.mode())));
    }

    // Check that distance is converging towards the destination. Return route
    // failure if no convergence for TODO iterations
    float dist2dest = pred.distance();
    if (dist2dest < mindist) {
      mindist = dist2dest;
      nc = 0;
    } else if (nc++ > kMaxIterationsWithoutConvergence) {
      if (best_path.first >= 0) {
        if (expansion_callback_) {
          expansion_callback_(graphreader, pred.edgeid(),
                              GraphId(pred.predecessor() == kInvalidLabel
                                          ? kInvalidGraphId
                                          : edgelabels_[pred.predecessor()].edgeid()),
                              name(), Expansion_EdgeStatus_connected, pred.cost().secs,
                              pred.distance(), pred.cost().cost,
                              Expansion_ExpansionType::Expansion_ExpansionType_forward, kNoFlowMask,
                              static_cast<TravelMode>(static_cast<size_t>(pred.mode())));
        }
        return {FormPath(graphreader, best_path.first)};
      } else {
        LOG_ERROR("No convergence to destination after = " + std::to_string(edgelabels_.size()));
        return {};
      }
    }

    // Expand forward from the end node of the predecessor edge.
    ExpandForward(graphreader, pred.endnode(), pred, predindex, false, false, pred.mode(),
                  destination, best_path, 0);
  }
  return {}; // Should never get here
}

// Add an edge at the origin to the adjacency list
void MultimodalAStar::SetOrigin(GraphReader& graphreader,
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
    if (start_costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
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
    Cost cost =
        start_costing_->PartialEdgeCost(directededge, edgeid, tile, edge.percent_along(), 1.0f);
    float dist = start_astarheuristic_.GetDistance(endtile->get_node_ll(directededge->endnode()));

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
            Cost dest_cost = start_costing_->PartialEdgeCost(dest_diredge, id, tile,
                                                             destination_edge.percent_along(), 1.0f);
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
    float sortcost = cost.cost + start_astarheuristic_.Get(dist);

    // Add EdgeLabel to the adjacency list (but do not set its status).
    // Set the predecessor edge index to invalid to indicate the origin
    // of the path.
    uint32_t d = static_cast<uint32_t>(directededge->length() * (1.0f - edge.percent_along()));
    BDEdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost, sortcost, dist, start_mode_,
                           baldr::kInvalidRestriction, false, false, sif::InternalTurn::kNoTurn);
    // Set the origin flag and path distance
    edge_label.set_origin();
    edge_label.set_path_distance(d);

    // Add EdgeLabel to the adjacency list
    uint32_t idx = edgelabels_.size();
    edgelabels_.push_back(std::move(edge_label));
    adjacencylist_.add(idx);
    if (expansion_callback_) {
      expansion_callback_(graphreader, edgeid, GraphId(kInvalidGraphId), name(),
                          Expansion_EdgeStatus_reached, cost.secs, d, cost.cost,
                          Expansion_ExpansionType::Expansion_ExpansionType_forward, kNoFlowMask,
                          static_cast<TravelMode>(static_cast<size_t>(start_mode_)));
    }

    // DO NOT SET EdgeStatus - it messes up trivial paths with oneways
  }

  // Set the origin timezone
  if (closest_ni != nullptr && !origin.date_time().empty() && origin.date_time() == "current") {
    origin.set_date_time(
        DateTime::iso_date_time(DateTime::get_tz_db().from_index(closest_ni->timezone())));
  }
}

// Add a destination edge
void MultimodalAStar::SetDestination(GraphReader& graphreader, const valhalla::Location& dest) {

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
    if (start_costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
      continue;
    }

    // Keep the cost to traverse the partial distance for the remainder of the edge. This cost
    // is subtracted from the total cost up to the end of the destination edge.
    auto tile = graphreader.GetGraphTile(edgeid);
    assert(tile);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    destinations_[edge.graph_id()] =
        (start_mode_ == end_mode_ ? start_costing_ : other_costing_)
            ->PartialEdgeCost(directededge, edgeid, tile, edge.percent_along(), 1.0f);

    // Edge score (penalty) is handled within GetPath. Do not add score here.
  }
  return;
}

// Form the path from the adjacency list.
std::vector<PathInfo> MultimodalAStar::FormPath(baldr::GraphReader& graphreader,
                                                const uint32_t dest) {
  // Metrics to track
  LOG_DEBUG("path_cost::" + std::to_string(edgelabels_[dest].cost().cost));
  LOG_DEBUG("path_iterations::" + std::to_string(edgelabels_.size()));

  // Work backwards from the destination
  std::vector<PathInfo> path;
  travel_mode_t old = start_mode_;
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
  assert(mode_change_count <= 2);

  // Reverse the list and return
  std::reverse(path.begin(), path.end());
  return path;
}
} // namespace thor
} // namespace valhalla
