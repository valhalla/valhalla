#include <algorithm>
#include <vector>

#include "baldr/datetime.h"
#include "midgard/logging.h"
#include "thor/timedistancematrix.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

// Constructor with cost threshold.
TimeDistanceMatrix::TimeDistanceMatrix(const boost::property_tree::ptree& config)
    : MatrixAlgorithm(config), settled_count_(0), current_cost_threshold_(0),
      max_reserved_labels_count_(config.get<uint32_t>("max_reserved_labels_count_dijkstras",
                                                      kInitialEdgeLabelCountDijkstras)),
      mode_(travel_mode_t::kDrive) {
}

// Compute a cost threshold in seconds based on average speed for the travel mode.
// Use a conservative speed estimate (in MPH) for each travel mode.
float TimeDistanceMatrix::GetCostThreshold(const float max_matrix_distance) const {
  float average_speed_mph;
  switch (mode_) {
    case travel_mode_t::kBicycle:
      average_speed_mph = 10.0f;
      break;
    case travel_mode_t::kPedestrian:
    case travel_mode_t::kPublicTransit:
      average_speed_mph = 2.0f;
      break;
    case travel_mode_t::kDrive:
    default:
      average_speed_mph = 35.0f;
  }

  // Convert max_matrix_distance to seconds based on the average speed
  return max_matrix_distance / (average_speed_mph * kMPHtoMetersPerSec);
}

// Expand from a node in the forward direction
template <const ExpansionType expansion_direction, const bool FORWARD>
void TimeDistanceMatrix::Expand(GraphReader& graphreader,
                                const GraphId& node,
                                const EdgeLabel& pred,
                                const uint32_t pred_idx,
                                const bool from_transition,
                                const baldr::TimeInfo& time_info,
                                const bool invariant) {
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  graph_tile_ptr tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }
  const NodeInfo* nodeinfo = tile->node(node);
  // TODO(nils): handle deadends in this algo, this should be flagged as one too
  if (!costing_->Allowed(nodeinfo)) {
    return;
  }

  // will be updated along the expansion
  auto offset_time = from_transition
                         ? time_info
                         : (FORWARD ? time_info.forward(invariant ? 0.f : pred.cost().secs,
                                                        static_cast<int>(nodeinfo->timezone()))
                                    : time_info.reverse(invariant ? 0.f : pred.cost().secs,
                                                        static_cast<int>(nodeinfo->timezone())));

  const DirectedEdge* opp_pred_edge = nullptr;
  if (!FORWARD) {
    opp_pred_edge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, opp_pred_edge++) {
      if (opp_pred_edge->localedgeidx() == pred.opp_local_idx()) {
        break;
      }
    }
  }

  // Expand from end node.
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  EdgeStatusInfo* es = edgestatus_.GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, ++edgeid, ++es) {
    // Skip shortcut edges
    if (directededge->is_shortcut() || es->set() == EdgeSet::kPermanent) {
      continue;
    }

    graph_tile_ptr t2 = nullptr;
    GraphId opp_edge_id;
    const DirectedEdge* opp_edge = nullptr;
    if (!FORWARD) {
      // Get opposing edge Id and end node tile
      t2 = directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (t2 == nullptr) {
        continue;
      }
      opp_edge_id = t2->GetOpposingEdgeId(directededge);
      opp_edge = t2->directededge(opp_edge_id);
    }

    // Skip this edge if permanently labeled (best path already found to this
    // directed edge), if no access is allowed to this edge (based on costing
    // method), or if a complex restriction prevents this path.
    uint8_t restriction_idx = kInvalidRestriction;
    const bool is_dest = dest_edges_.find(edgeid) != dest_edges_.cend();
    if (FORWARD) {
      if (!costing_->Allowed(directededge, is_dest, pred, tile, edgeid, offset_time.local_time,
                             nodeinfo->timezone(), restriction_idx) ||
          costing_->Restricted(directededge, pred, edgelabels_, tile, edgeid, true, nullptr,
                               offset_time.local_time, nodeinfo->timezone())) {
        continue;
      }
    } else {
      if (!costing_->AllowedReverse(directededge, pred, opp_edge, t2, opp_edge_id,
                                    offset_time.local_time, nodeinfo->timezone(), restriction_idx) ||
          (costing_->Restricted(directededge, pred, edgelabels_, tile, edgeid, false, nullptr,
                                offset_time.local_time, nodeinfo->timezone()))) {
        continue;
      }
    }

    // Get cost and update distance
    uint8_t flow_sources;
    auto newcost = FORWARD ? costing_->EdgeCost(directededge, tile, offset_time, flow_sources)
                           : costing_->EdgeCost(opp_edge, t2, offset_time, flow_sources);
    auto transition_cost =
        FORWARD ? costing_->TransitionCost(directededge, nodeinfo, pred)
                : costing_->TransitionCostReverse(directededge->localedgeidx(), nodeinfo, opp_edge,
                                                  opp_pred_edge,
                                                  static_cast<bool>(flow_sources & kDefaultFlowMask),
                                                  pred.internal_turn());
    newcost += pred.cost() + transition_cost;
    uint32_t path_distance = pred.path_distance() + directededge->length();

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the cost and predecessor are updated.
    if (es->set() == EdgeSet::kTemporary) {
      auto& lab = edgelabels_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        adjacencylist_.decrease(es->index(), newcost.cost);
        lab.Update(pred_idx, newcost, newcost.cost, path_distance, restriction_idx);
      }
      continue;
    }

    // Add to the adjacency list and edge labels.
    uint32_t idx = edgelabels_.size();
    if (FORWARD) {
      edgelabels_.emplace_back(pred_idx, edgeid, directededge, newcost, newcost.cost, mode_,
                               path_distance, restriction_idx,
                               (pred.closure_pruning() || !(costing_->IsClosed(directededge, tile))),
                               0 != (flow_sources & kDefaultFlowMask),
                               costing_->TurnType(pred.opp_local_idx(), nodeinfo, directededge), 0,
                               directededge->destonly() ||
                                   (costing_->is_hgv() && directededge->destonly_hgv()),
                               directededge->forwardaccess() & kTruckAccess);
    } else {
      edgelabels_.emplace_back(pred_idx, edgeid, directededge, newcost, newcost.cost, mode_,
                               path_distance, restriction_idx,
                               (pred.closure_pruning() || !(costing_->IsClosed(opp_edge, t2))),
                               0 != (flow_sources & kDefaultFlowMask),
                               costing_->TurnType(directededge->localedgeidx(), nodeinfo, opp_edge,
                                                  opp_pred_edge),
                               0,
                               opp_edge->destonly() ||
                                   (costing_->is_hgv() && opp_edge->destonly_hgv()),
                               opp_edge->forwardaccess() & kTruckAccess);
    }

    *es = {EdgeSet::kTemporary, idx};
    adjacencylist_.add(idx);
  }

  // Handle transitions - expand from the end node each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      Expand<expansion_direction>(graphreader, trans->endnode(), pred, pred_idx, true, offset_time);
    }
  }
}

template <const ExpansionType expansion_direction, const bool FORWARD>
bool TimeDistanceMatrix::ComputeMatrix(Api& request,
                                       baldr::GraphReader& graphreader,
                                       const float max_matrix_distance) {
  bool invariant = request.options().date_time_type() == Options::invariant;
  uint32_t matrix_locations = request.options().matrix_locations();

  uint32_t bucketsize = costing_->UnitSize();

  auto& origins = FORWARD ? *request.mutable_options()->mutable_sources()
                          : *request.mutable_options()->mutable_targets();
  auto& destinations = FORWARD ? *request.mutable_options()->mutable_targets()
                               : *request.mutable_options()->mutable_sources();

  size_t num_elements = origins.size() * destinations.size();
  auto time_infos = SetTime(origins, graphreader);

  // Initialize destinations once for all origins
  InitDestinations<expansion_direction>(graphreader, destinations);
  // reserve the PBF vectors
  reserve_pbf_arrays(*request.mutable_matrix(), num_elements, costing_->pass());

  for (int origin_index = 0; origin_index < origins.size(); ++origin_index) {
    // reserve some space for the next dijkstras (will be cleared at the end of the loop)
    edgelabels_.reserve(max_reserved_labels_count_);
    auto& origin = origins.Get(origin_index);
    const auto& time_info = time_infos[origin_index];

    current_cost_threshold_ = GetCostThreshold(max_matrix_distance);

    // Construct adjacency list. Set bucket size and cost range based on DynamicCost.
    adjacencylist_.reuse(0.0f, current_cost_threshold_, bucketsize, &edgelabels_);

    // Initialize the origin and set the available destination edges
    settled_count_ = 0;
    SetOrigin<expansion_direction>(graphreader, origin, time_info);
    SetDestinationEdges();

    uint32_t n = 0;
    // Collect edge_ids used for settling a location to determine its time zone
    std::unordered_map<uint32_t, baldr::GraphId> dest_edge_ids;
    dest_edge_ids.reserve(destinations.size());

    // Find shortest path
    graph_tile_ptr tile;
    while (true) {
      // Get next element from adjacency list. Check that it is valid. An
      // invalid label indicates there are no edges that can be expanded.
      uint32_t predindex = adjacencylist_.pop();
      if (predindex == kInvalidLabel) {
        // Can not expand any further...
        FormTimeDistanceMatrix(request, graphreader, FORWARD, origin_index, origin.date_time(),
                               time_info.timezone_index, dest_edge_ids);
        break;
      }

      // Copy the EdgeLabel for use in costing
      EdgeLabel pred = edgelabels_[predindex];

      // Remove label from adjacency list, mark it as permanently labeled.

      // Mark the edge as permanently labeled. Do not do this for an origin
      // edge. Otherwise loops/around the block cases will not work
      if (!pred.origin()) {
        edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent);
      }

      // Identify any destinations on this edge
      auto destedge = dest_edges_.find(pred.edgeid());
      if (destedge != dest_edges_.end()) {
        // Update any destinations along this edge. Return if all destinations
        // have been settled or the requested amount of destinations has been found
        tile = graphreader.GetGraphTile(pred.edgeid());
        const DirectedEdge* edge = tile->directededge(pred.edgeid());

        for (auto& dest_id : destedge->second) {
          dest_edge_ids[dest_id] = pred.edgeid();
        }
        if (UpdateDestinations(origin, destinations, destedge->second, edge, tile, pred, time_info,
                               matrix_locations)) {
          FormTimeDistanceMatrix(request, graphreader, FORWARD, origin_index, origin.date_time(),
                                 time_info.timezone_index, dest_edge_ids);
          break;
        }
      }

      // Terminate when we are beyond the cost threshold
      if (pred.cost().cost > current_cost_threshold_) {
        FormTimeDistanceMatrix(request, graphreader, FORWARD, origin_index, origin.date_time(),
                               time_info.timezone_index, dest_edge_ids);
        break;
      }

      // Expand forward from the end node of the predecessor edge.
      Expand<expansion_direction>(graphreader, pred.endnode(), pred, predindex, false, time_info,
                                  invariant);

      // Allow this process to be aborted
      if (interrupt_ && (n++ % kInterruptIterationsInterval) == 0) {
        (*interrupt_)();
      }
    }

    reset();
  }

  // TODO(nils): implement second pass here too
  return true;
}

template bool
TimeDistanceMatrix::ComputeMatrix<ExpansionType::forward, true>(Api& request,
                                                                baldr::GraphReader& graphreader,
                                                                const float max_matrix_distance);
template bool
TimeDistanceMatrix::ComputeMatrix<ExpansionType::reverse, false>(Api& request,
                                                                 baldr::GraphReader& graphreader,
                                                                 const float max_matrix_distance);

// Add edges at the origin to the adjacency list
template <const ExpansionType expansion_direction, const bool FORWARD>
void TimeDistanceMatrix::SetOrigin(GraphReader& graphreader,
                                   const valhalla::Location& origin,
                                   const TimeInfo& time_info) {
  // Only skip inbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.correlation().edges().begin(), origin.correlation().edges().end(),
                [&has_other_edges](const valhalla::PathEdge& e) {
                  has_other_edges = has_other_edges || (FORWARD ? !e.end_node() : !e.begin_node());
                });

  // Iterate through edges and add to adjacency list
  for (const auto& edge : origin.correlation().edges()) {
    // If origin is at a node - skip any inbound edge (dist = 1)
    if ((FORWARD ? edge.end_node() : edge.begin_node()) && has_other_edges) {
      continue;
    }

    // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
    GraphId edgeid(edge.graph_id());
    if (FORWARD ? costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())
                : costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
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

    uint8_t flow_sources;
    // Cost is also sortcost, since this is Dijsktra
    Cost cost;
    float dist;
    GraphId opp_edge_id;
    const DirectedEdge* opp_dir_edge;
    if (FORWARD) {
      const auto percent_along = 1.0f - edge.percent_along();
      cost = costing_->EdgeCost(directededge, tile, time_info, flow_sources) * percent_along;
      dist = static_cast<uint32_t>(directededge->length() * percent_along);

    } else {
      opp_edge_id = graphreader.GetOpposingEdgeId(edgeid);
      if (!opp_edge_id.Is_Valid()) {
        continue;
      }
      opp_dir_edge = graphreader.GetOpposingEdge(edgeid);
      cost =
          costing_->EdgeCost(opp_dir_edge, endtile, time_info, flow_sources) * edge.percent_along();
      dist = static_cast<uint32_t>(directededge->length() * edge.percent_along());
    }

    // We need to penalize this location based on its score (distance in meters from input)
    // We assume the slowest speed you could travel to cover that distance to start/end the route
    // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
    cost.cost += edge.distance();

    // Add EdgeLabel to the adjacency list (but do not set its status).
    // Set the predecessor edge index to invalid to indicate the origin
    // of the path. Set the origin flag
    if (FORWARD) {
      edgelabels_.emplace_back(kInvalidLabel, edgeid, directededge, cost, cost.cost, mode_, dist,
                               baldr::kInvalidRestriction, !costing_->IsClosed(directededge, tile),
                               static_cast<bool>(flow_sources & kDefaultFlowMask),
                               InternalTurn::kNoTurn, 0,
                               directededge->destonly() ||
                                   (costing_->is_hgv() && directededge->destonly_hgv()),
                               directededge->forwardaccess() & kTruckAccess);
    } else {
      edgelabels_.emplace_back(kInvalidLabel, opp_edge_id, opp_dir_edge, cost, cost.cost, mode_, dist,
                               baldr::kInvalidRestriction, !costing_->IsClosed(directededge, tile),
                               static_cast<bool>(flow_sources & kDefaultFlowMask),
                               InternalTurn::kNoTurn, 0,
                               directededge->destonly() ||
                                   (costing_->is_hgv() && directededge->destonly_hgv()),
                               directededge->forwardaccess() & kTruckAccess);
    }
    edgelabels_.back().set_origin();
    adjacencylist_.add(edgelabels_.size() - 1);
  }
}

// Set destinations
template <const ExpansionType expansion_direction, const bool FORWARD>
void TimeDistanceMatrix::InitDestinations(
    GraphReader& graphreader,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& locations) {
  // For each destination
  uint32_t idx = 0;
  for (const auto& loc : locations) {
    // Set up the destination - consider each possible location edge.
    bool first_edge = true;
    for (const auto& edge : loc.correlation().edges()) {
      // Disallow any user avoided edges if the avoid location is behind the destination along the
      // edge or before the destination for REVERSE
      GraphId edgeid(edge.graph_id());
      if (FORWARD ? costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())
                  : costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Add a destination if this is the first allowed edge for the location
      if (first_edge) {
        destinations_.emplace_back();
        first_edge = false;
      }

      // Form a threshold cost (the total cost to traverse the edge), also based on forward path for
      // REVERSE
      graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
      const DirectedEdge* directededge = tile->directededge(edgeid);
      float c = costing_->EdgeCost(directededge, tile).cost;

      // Keep the id and the partial distance for the remainder of the edge.
      Destination& d = destinations_.back();
      edgeid = FORWARD ? edgeid : graphreader.GetOpposingEdgeId(edgeid);
      auto percent_along = FORWARD ? (1.0f - edge.percent_along()) : edge.percent_along();

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
      c += edge.distance();
      if (c > d.threshold) {
        d.threshold = c;
      }

      // Mark the edge as having a destination on it and add the
      // destination index
      d.dest_edges_percent_along[edgeid] = percent_along;
      dest_edges_[edgeid].push_back(idx);
    }
    idx++;
  }
}

// Update any destinations along the edge. Returns true if all destinations
// have be settled or if the specified location count has been met or exceeded.
bool TimeDistanceMatrix::UpdateDestinations(
    const valhalla::Location& origin,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
    std::vector<uint32_t>& destinations,
    const DirectedEdge* edge,
    const graph_tile_ptr& tile,
    const EdgeLabel& pred,
    const TimeInfo& time_info,
    const uint32_t matrix_locations) {
  // For each destination along this edge
  for (auto dest_idx : destinations) {
    Destination& dest = destinations_[dest_idx];
    auto& dest_loc = locations.Get(dest_idx);

    // Skip if destination has already been settled. This can happen since we
    // do not remove remaining destination edges for this destination from
    // dest_edges.
    if (dest.settled) {
      continue;
    }

    // See if this edge is part of the destination
    // If the edge isn't there but the path is trivial, then that means the edge
    // was removed towards the beginning which is not an error.
    auto dest_available = dest.dest_edges_available.find(pred.edgeid());
    if (dest_available == dest.dest_edges_available.end()) {
      if (!IsTrivial(pred.edgeid(), origin, locations.Get(dest_idx))) {
        LOG_ERROR("Could not find the destination edge");
      }
      continue;
    }

    // stuff we need to do when settling a destination (edge)
    auto settle_dest = [&]() {
      dest.dest_edges_available.erase(dest_available);
      if (dest.dest_edges_available.empty()) {
        dest.settled = true;
        settled_count_++;
      }
    };

    if (origin.ll().lat() == dest_loc.ll().lat() && origin.ll().lng() == dest_loc.ll().lng()) {
      dest.best_cost = Cost{0.f, 0.f};
      dest.distance = 0;
      settle_dest();
      continue;
    }

    auto dest_edge = dest.dest_edges_percent_along.find(pred.edgeid());

    // Skip case where destination is along the origin edge, there is no
    // predecessor, and the destination cannot be reached via trivial path.
    if (pred.predecessor() == kInvalidLabel && !IsTrivial(pred.edgeid(), origin, dest_loc)) {
      continue;
    }

    // Get the cost. The predecessor cost is cost to the end of the edge.
    // Subtract the partial remaining cost and distance along the edge.
    uint8_t flow_sources;
    float remainder = dest_edge->second;
    Cost newcost =
        pred.cost() - (costing_->EdgeCost(edge, tile, time_info, flow_sources) * remainder);
    if (newcost.cost < dest.best_cost.cost) {
      dest.best_cost = newcost;
      dest.distance = pred.path_distance() - (edge->length() * remainder);
    }

    // Erase this edge from further consideration. Mark this destination as
    // settled if all edges have been found
    settle_dest();
  }

  // Settle any destinations where current cost is above the destination's
  // best cost + threshold. This helps remove destinations where one edge
  // cannot be reached (e.g. on a cul-de-sac or where turn restrictions apply).
  // Update the cost threshold if at least one path to all destinations has
  // been found.
  bool allfound = true;
  float maxcost = 0.0f;

  for (auto& d : destinations_) {
    // Skip any settled destinations
    if (d.settled) {
      continue;
    }

    // Do not update cost threshold if no path to this destination
    // has been found
    if (d.best_cost.cost == kMaxCost) {
      allfound = false;
    } else {
      // Settle any destinations above their threshold and update maxcost
      if ((d.best_cost.cost + d.threshold) < pred.cost().cost) {
        d.settled = true;
        settled_count_++;
      }
      maxcost = std::max(maxcost, d.best_cost.cost + d.threshold);
    }
  }

  // Update cost threshold for early termination if at least one path has
  // been found to each destination
  if (allfound) {
    current_cost_threshold_ = maxcost;
  }

  // Return true if the settled count equals the number of destinations or
  // exceeds the matrix location count provided.
  return settled_count_ == destinations_.size() || settled_count_ >= matrix_locations;
}

// Form the time, distance matrix from the destinations list
void TimeDistanceMatrix::FormTimeDistanceMatrix(Api& request,
                                                GraphReader& reader,
                                                const bool forward,
                                                const uint32_t origin_index,
                                                const std::string& origin_dt,
                                                const uint64_t& origin_tz,
                                                std::unordered_map<uint32_t, GraphId>& edge_ids) {
  // when it's forward, origin_index will be the source_index
  // when it's reverse, origin_index will be the target_index
  valhalla::Matrix& matrix = *request.mutable_matrix();
  graph_tile_ptr tile;
  for (uint32_t i = 0; i < destinations_.size(); i++) {
    auto& dest = destinations_[i];
    auto pbf_idx = forward ? (origin_index * request.options().targets().size()) + i
                           : (i * request.options().targets().size()) + origin_index;
    matrix.mutable_from_indices()->Set(pbf_idx, forward ? origin_index : i);
    matrix.mutable_to_indices()->Set(pbf_idx, forward ? i : origin_index);
    matrix.mutable_distances()->Set(pbf_idx, dest.distance);
    matrix.mutable_times()->Set(pbf_idx, dest.best_cost.secs);

    auto dt_info =
        DateTime::offset_date(origin_dt, origin_tz, reader.GetTimezoneFromEdge(edge_ids[i], tile),
                              static_cast<uint64_t>(dest.best_cost.secs));
    *matrix.mutable_date_times(pbf_idx) = dt_info.date_time;
    *matrix.mutable_time_zone_names(pbf_idx) = dt_info.time_zone_name;
    *matrix.mutable_time_zone_offsets(pbf_idx) = dt_info.time_zone_offset;
  }
}

} // namespace thor
} // namespace valhalla
