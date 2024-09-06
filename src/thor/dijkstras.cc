#include "thor/dijkstras.h"
#include "baldr/datetime.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include <algorithm>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

// Method to get an operator Id from a map of operator strings vs. Id.
uint32_t GetOperatorId(const graph_tile_ptr& tile,
                       uint32_t routeid,
                       std::unordered_map<std::string, uint32_t>& operators) {
  const TransitRoute* transit_route = tile->GetTransitRoute(routeid);

  // Test if the transit operator changed
  if (transit_route && transit_route->op_by_onestop_id_offset()) {
    // Get the operator name and look up in the operators map
    std::string operator_name = tile->GetName(transit_route->op_by_onestop_id_offset());
    auto operator_itr = operators.find(operator_name);
    if (operator_itr == operators.end()) {
      // Operator not found - add to the map
      uint32_t id = operators.size() + 1;
      operators[operator_name] = id;
      return id;
    } else {
      return operator_itr->second;
    }
  }
  return 0;
}

} // namespace

namespace valhalla {
namespace thor {

// Default constructor
Dijkstras::Dijkstras(const boost::property_tree::ptree& config)
    : mode_(travel_mode_t::kDrive), access_mode_(kAutoAccess),
      max_reserved_labels_count_(config.get<uint32_t>("max_reserved_labels_count_dijkstras",
                                                      kInitialEdgeLabelCountDijkstras)),
      clear_reserved_memory_(config.get<bool>("clear_reserved_memory", false)), multipath_(false) {
}

// Clear the temporary information generated during path construction.
void Dijkstras::Clear() {
  // Clear the edge labels, edge status flags, and adjacency list
  // TODO - clear only the edge label set that was used?
  auto reservation = clear_reserved_memory_ ? 0 : max_reserved_labels_count_;
  if (bdedgelabels_.size() > reservation) {
    bdedgelabels_.resize(reservation);
    bdedgelabels_.shrink_to_fit();
  }
  bdedgelabels_.clear();
  if (mmedgelabels_.size() > reservation) {
    mmedgelabels_.resize(reservation);
    mmedgelabels_.shrink_to_fit();
  }
  mmedgelabels_.clear();

  adjacencylist_.clear();
  mmadjacencylist_.clear();
  edgestatus_.clear();
}

// Initialize - create adjacency list, edgestatus support, and reserve
// edgelabels
template <typename label_container_t>
void Dijkstras::Initialize(label_container_t& labels,
                           baldr::DoubleBucketQueue<typename label_container_t::value_type>& queue,
                           const uint32_t bucket_size) {
  // Set aside some space for edge labels
  uint32_t edge_label_reservation;
  uint32_t bucket_count;
  GetExpansionHints(bucket_count, edge_label_reservation);
  labels.reserve(max_reserved_labels_count_);

  // Set up lambda to get sort costs
  float range = bucket_count * bucket_size;
  queue.reuse(0.0f, range, bucket_size, &labels);
}
template void
Dijkstras::Initialize<decltype(Dijkstras::bdedgelabels_)>(decltype(Dijkstras::bdedgelabels_)&,
                                                          baldr::DoubleBucketQueue<sif::BDEdgeLabel>&,
                                                          const uint32_t);
template void
Dijkstras::Initialize<decltype(Dijkstras::mmedgelabels_)>(decltype(Dijkstras::mmedgelabels_)&,
                                                          baldr::DoubleBucketQueue<sif::MMEdgeLabel>&,
                                                          const uint32_t);

// Initializes the time of the expansion if there is one
std::vector<TimeInfo>
Dijkstras::SetTime(google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                   GraphReader& reader) {
  // loop over all locations setting the date time with timezone
  std::vector<TimeInfo> infos;
  for (auto& location : locations) {
    infos.emplace_back(TimeInfo::make(location, reader, &tz_cache_));
  }

  // Hand back the time information
  return infos;
}

template <const ExpansionType expansion_direction>
void Dijkstras::ExpandInner(baldr::GraphReader& graphreader,
                            const baldr::GraphId& node,
                            const typename decltype(Dijkstras::bdedgelabels_)::value_type& pred,
                            const uint32_t pred_idx,
                            const baldr::DirectedEdge* opp_pred_edge,
                            const bool from_transition,
                            const baldr::TimeInfo& time_info) {

  constexpr bool FORWARD = expansion_direction == ExpansionType::forward;
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  graph_tile_ptr tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }

  // Get the nodeinfo
  const NodeInfo* nodeinfo = tile->node(node);

  // We dont need to do transitions again we just need to queue the edges that leave them
  if (!from_transition) {
    // Let implementing class know we are expanding from here
    EdgeLabel* prev_pred =
        pred.predecessor() == kInvalidLabel ? nullptr : &bdedgelabels_[pred.predecessor()];
    ExpandingNode(graphreader, tile, nodeinfo, pred, prev_pred);
  }

  // Bail if we cant expand from here
  if (!costing_->Allowed(nodeinfo)) {
    return;
  }

  // Update the time information
  auto offset_time =
      from_transition
          ? time_info
          : (FORWARD ? time_info.forward(pred.cost().secs, static_cast<int>(nodeinfo->timezone()))
                     : time_info.reverse(pred.cost().secs, static_cast<int>(nodeinfo->timezone())));

  // Expand from end node in forward direction.
  GraphId edgeid = {node.tileid(), node.level(), nodeinfo->edge_index()};
  EdgeStatusInfo* es = edgestatus_.GetPtr(edgeid, tile, pred.path_id());
  const DirectedEdge* directededge = tile->directededge(edgeid);
  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++directededge, ++edgeid, ++es) {
    // Skip this edge if permanently labeled (best path already found to this
    // directed edge). skip shortcuts or if no access is allowed to this edge
    // (based on the costing method) or if a complex restriction exists for
    // this path.
    if (directededge->is_shortcut() || es->set() == EdgeSet::kPermanent ||
        !((FORWARD ? directededge->forwardaccess() : directededge->reverseaccess()) & access_mode_)) {
      continue;
    }

    // Get end node tile, opposing edge Id, and opposing directed edge.
    graph_tile_ptr t2;
    baldr::GraphId oppedgeid;
    const baldr::DirectedEdge* opp_edge;
    // Only populate these at this point for the reverse search - the forward search
    // may exit before needing them, so we can defer some work until later
    if (!FORWARD) { // aka reverse
      t2 = tile;
      oppedgeid = graphreader.GetOpposingEdgeId(edgeid, t2);
      if (t2 == nullptr) {
        continue;
      }
      opp_edge = t2->directededge(oppedgeid);
    }

    // Check if the edge is allowed or if a restriction occurs
    EdgeStatus* todo = nullptr;
    uint8_t restriction_idx = kInvalidRestriction;
    // is_dest is false, because it is a traversal algorithm in this context, not a path search
    // algorithm. In other words, destination edges are not defined for this Dijkstra's algorithm.
    const bool is_dest = false;
    if (offset_time.valid) {
      // With date time we check time dependent restrictions and access
      const bool allowed =
          FORWARD ? costing_->Allowed(directededge, is_dest, pred, tile, edgeid,
                                      offset_time.local_time, nodeinfo->timezone(), restriction_idx)
                  : costing_->AllowedReverse(directededge, pred, opp_edge, t2, oppedgeid,
                                             offset_time.local_time, nodeinfo->timezone(),
                                             restriction_idx);
      if (!allowed || costing_->Restricted(directededge, pred, bdedgelabels_, tile, edgeid, true,
                                           todo, offset_time.local_time, nodeinfo->timezone())) {
        continue;
      }
    } else {
      const bool allowed = FORWARD ? costing_->Allowed(directededge, is_dest, pred, tile, edgeid, 0,
                                                       0, restriction_idx)
                                   : costing_->AllowedReverse(directededge, pred, opp_edge, t2,
                                                              oppedgeid, 0, 0, restriction_idx);

      if (!allowed || costing_->Restricted(directededge, pred, bdedgelabels_, tile, edgeid, true)) {
        continue;
      }
    }

    // Compute the cost and path distance to the end of this edge
    Cost transition_cost, newcost;
    uint8_t flow_sources;

    if (FORWARD) {
      transition_cost = costing_->TransitionCost(directededge, nodeinfo, pred);
      newcost = pred.cost() + costing_->EdgeCost(directededge, tile, offset_time, flow_sources) +
                transition_cost;
    } else {
      transition_cost =
          costing_->TransitionCostReverse(directededge->localedgeidx(), nodeinfo, opp_edge,
                                          opp_pred_edge, pred.has_measured_speed(),
                                          pred.internal_turn());
      newcost =
          pred.cost() + costing_->EdgeCost(opp_edge, t2, offset_time, flow_sources) + transition_cost;
    }
    uint32_t path_dist = pred.path_distance() + directededge->length();

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (es->set() == EdgeSet::kTemporary) {
      BDEdgeLabel& lab = bdedgelabels_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_.decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, transition_cost, path_dist, restriction_idx);
      }
      continue;
    }

    // Only needed if you want to connect with a reverse path - for reverse mode,
    // these were populated earlier
    if (FORWARD) {
      t2 = tile;
      oppedgeid = graphreader.GetOpposingEdgeId(edgeid, t2);
    }

    // Add edge label, add to the adjacency list and set edge status
    uint32_t idx = bdedgelabels_.size();
    *es = {EdgeSet::kTemporary, idx};
    if (FORWARD) {
      bdedgelabels_.emplace_back(pred_idx, edgeid, oppedgeid, directededge, newcost, mode_,
                                 transition_cost, path_dist, false,
                                 (pred.closure_pruning() || !costing_->IsClosed(directededge, tile)),
                                 static_cast<bool>(flow_sources & kDefaultFlowMask),
                                 costing_->TurnType(pred.opp_local_idx(), nodeinfo, directededge),
                                 restriction_idx, pred.path_id(),
                                 directededge->destonly() ||
                                     (costing_->is_hgv() && directededge->destonly_hgv()),
                                 directededge->forwardaccess() & kTruckAccess);

    } else {
      bdedgelabels_.emplace_back(pred_idx, edgeid, oppedgeid, directededge, newcost, mode_,
                                 transition_cost, path_dist, false,
                                 (pred.closure_pruning() || !costing_->IsClosed(opp_edge, t2)),
                                 static_cast<bool>(flow_sources & kDefaultFlowMask),
                                 costing_->TurnType(directededge->localedgeidx(), nodeinfo, opp_edge,
                                                    opp_pred_edge),
                                 restriction_idx, pred.path_id(),
                                 opp_edge->destonly() ||
                                     (costing_->is_hgv() && opp_edge->destonly_hgv()),
                                 opp_edge->forwardaccess() & kTruckAccess);
    }
    adjacencylist_.add(idx);
  }

  // Handle transitions - expand from the end node of each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const baldr::NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      ExpandInner<expansion_direction>(graphreader, trans->endnode(), pred, pred_idx, opp_pred_edge,
                                       true, offset_time);
    }
  }
}

// Note: multimodal not ported yet, so there is no implementation for ExpansionType::multimodal
template void Dijkstras::ExpandInner<ExpansionType::forward>(
    baldr::GraphReader& graphreader,
    const baldr::GraphId& node,
    const typename decltype(Dijkstras::bdedgelabels_)::value_type& pred,
    const uint32_t pred_idx,
    const baldr::DirectedEdge* opp_pred_edge,
    const bool from_transition,
    const baldr::TimeInfo& time_info);
template void Dijkstras::ExpandInner<ExpansionType::reverse>(
    baldr::GraphReader& graphreader,
    const baldr::GraphId& node,
    const typename decltype(Dijkstras::bdedgelabels_)::value_type& pred,
    const uint32_t pred_idx,
    const baldr::DirectedEdge* opp_pred_edge,
    const bool from_transition,
    const baldr::TimeInfo& time_info);

// performs one of the types of expansions
// TODO: reduce code duplication between forward, reverse and multimodal as they are nearly
// identical
void Dijkstras::Expand(const ExpansionType expansion_type,
                       valhalla::Api& api,
                       baldr::GraphReader& reader,
                       const sif::mode_costing_t& costings,
                       const sif::travel_mode_t mode) {
  // compute the expansion
  switch (expansion_type) {
    case ExpansionType::forward:
      Compute<ExpansionType::forward>(*api.mutable_options()->mutable_locations(), reader, costings,
                                      mode);
      break;
    case ExpansionType::reverse:
      Compute<ExpansionType::reverse>(*api.mutable_options()->mutable_locations(), reader, costings,
                                      mode);
      break;
    case ExpansionType::multimodal:
      ComputeMultiModal(*api.mutable_options()->mutable_locations(), reader, costings, mode,
                        api.options());
      break;
    default:
      throw std::runtime_error("Unknown expansion type");
  }
}

template <const ExpansionType expansion_direction>
void Dijkstras::Compute(google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                        baldr::GraphReader& graphreader,
                        const sif::mode_costing_t& mode_costing,
                        const sif::travel_mode_t mode) {

  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  access_mode_ = costing_->access_mode();

  // Prepare for a graph traversal
  Initialize(bdedgelabels_, adjacencylist_, costing_->UnitSize());
  if (expansion_direction == ExpansionType::forward) {
    SetOriginLocations(graphreader, locations, costing_);
  } else {
    SetDestinationLocations(graphreader, locations, costing_);
  }

  // Get the time information for all the origin locations
  auto time_infos = SetTime(locations, graphreader);

  // Compute the isotile
  auto cb_decision = ExpansionRecommendation::continue_expansion;
  while (cb_decision != ExpansionRecommendation::stop_expansion) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_.pop();
    if (predindex == baldr::kInvalidLabel) {
      break;
    }

    // Copy the EdgeLabel for use in costing and settle the edge.
    sif::BDEdgeLabel pred = bdedgelabels_[predindex];
    edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent, pred.path_id());

    // Get the opposing predecessor directed edge. Need to make sure we get
    // the correct one if a transition occurred

    const baldr::DirectedEdge* opp_pred_edge = nullptr;
    if (expansion_direction == ExpansionType::reverse) {
      opp_pred_edge = graphreader.GetOpposingEdge(pred.edgeid());
      if (opp_pred_edge == nullptr) {
        continue;
      }
    }

    // Check if we should stop
    cb_decision = ShouldExpand(graphreader, pred, expansion_direction);
    if (cb_decision != ExpansionRecommendation::prune_expansion) {
      // Expand from the end node in expansion_direction.
      ExpandInner<expansion_direction>(graphreader, pred.endnode(), pred, predindex, opp_pred_edge,
                                       false, time_infos.front());
    }

    if (expansion_callback_) {
      const auto prev_pred = pred.predecessor() == kInvalidLabel
                                 ? GraphId{}
                                 : bdedgelabels_[pred.predecessor()].edgeid();
      expansion_callback_(graphreader, pred.edgeid(), prev_pred, "dijkstras",
                          Expansion_EdgeStatus_settled, pred.cost().secs, pred.path_distance(),
                          pred.cost().cost,
                          static_cast<Expansion_ExpansionType>(expansion_direction));
    }
  }
}

/**
 * NOTE: there's no implementation of ExpansionType::multimodal yet!
 */
template void Dijkstras::Compute<ExpansionType::forward>(
    google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
    baldr::GraphReader& graphreader,
    const sif::mode_costing_t& mode_costing,
    const sif::travel_mode_t mode);

template void Dijkstras::Compute<ExpansionType::reverse>(
    google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
    baldr::GraphReader& graphreader,
    const sif::mode_costing_t& mode_costing,
    const sif::travel_mode_t mode);

// Expand from a node in forward direction using multimodal.
void Dijkstras::ExpandForwardMultiModal(GraphReader& graphreader,
                                        const GraphId& node,
                                        const MMEdgeLabel& pred,
                                        const uint32_t pred_idx,
                                        const bool from_transition,
                                        const std::shared_ptr<DynamicCost>& pc,
                                        const std::shared_ptr<DynamicCost>& tc,
                                        const sif::mode_costing_t& mode_costing,
                                        const TimeInfo& time_info) {
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  auto tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }

  // Get the nodeinfo
  const NodeInfo* nodeinfo = tile->node(node);

  // We dont need to do transitions again we just need to queue the edges that leave them
  if (!from_transition) {
    // Let implementing class we are expanding from here
    EdgeLabel* prev_pred =
        pred.predecessor() == kInvalidLabel ? nullptr : &mmedgelabels_[pred.predecessor()];
    ExpandingNode(graphreader, tile, nodeinfo, pred, prev_pred);
  }

  if (nodeinfo->type() == NodeType::kMultiUseTransitPlatform ||
      nodeinfo->type() == NodeType::kTransitStation) {

    if (processed_tiles_.find(tile->id().tileid()) == processed_tiles_.end()) {
      tc->AddToExcludeList(tile);
      processed_tiles_.emplace(tile->id().tileid());
    }

    // check if excluded.
    if (tc->IsExcluded(tile, nodeinfo)) {
      return;
    }
  }

  if (!mode_costing[static_cast<uint8_t>(mode_)]->Allowed(nodeinfo)) {
    return;
  }

  // Update the time information
  auto offset_time =
      from_transition ? time_info
                      : time_info.forward(pred.cost().secs, static_cast<int>(nodeinfo->timezone()));

  // Set a default transfer penalty at a stop (if not same trip Id and block Id)
  Cost transfer_cost = tc->DefaultTransferCost();

  // Get any transfer times and penalties if this is a transit stop (and
  // transit has been taken at some point on the path) and mode is pedestrian
  mode_ = pred.mode();
  bool has_transit = pred.has_transit();
  GraphId prior_stop = pred.prior_stopid();
  uint32_t operator_id = pred.transit_operator();
  if (nodeinfo->type() == NodeType::kMultiUseTransitPlatform) {
    // Get the transfer penalty when changing stations
    if (mode_ == travel_mode_t::kPedestrian && prior_stop.Is_Valid() && has_transit) {
      transfer_cost = tc->TransferCost();
    }

    // Add transfer time to the local time when entering a stop as a pedestrian. This
    // is a small added cost on top of any costs along paths and roads. We only do this
    // once so if its from a transition we don't need to do it again
    if (mode_ == travel_mode_t::kPedestrian && !from_transition) {
      // TODO(nils): What happens if this wraps the day past midnight?
      // It might have to advance the day_ and dow_?
      offset_time.forward(transfer_cost.secs, static_cast<int>(nodeinfo->timezone()));
    }

    // Update prior stop. TODO - parent/child stop info?
    prior_stop = node;

    // we must get the date from level 3 transit tiles and not level 2.  The level 3 date is
    // set when the fetcher grabbed the transit data and created the schedules.
    if (!date_set_) {
      date_ = DateTime::days_from_pivot_date(DateTime::get_formatted_date(origin_date_time_));
      dow_ = DateTime::day_of_week_mask(origin_date_time_);
      uint32_t date_created = tile->header()->date_created();
      if (date_ < date_created) {
        date_before_tile_ = true;
      } else {
        day_ = date_ - date_created;
      }
      date_set_ = true;
    }
  }

  // TODO: allow mode changes at special nodes
  //      bike share (pedestrian <--> bicycle)
  //      parking (drive <--> pedestrian)
  //      transit stop (pedestrian <--> transit).
  bool mode_change = false;

  // Expand from end node.
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  EdgeStatusInfo* es = edgestatus_.GetPtr(edgeid, tile, pred.path_id());
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, ++edgeid, ++es) {
    // Skip shortcut edges and edges that are permanently labeled (best
    // path already found to this directed edge).
    if (directededge->is_shortcut() || es->set() == EdgeSet::kPermanent) {
      continue;
    }

    // Reset cost and walking distance
    Cost newcost = pred.cost();
    uint32_t walking_distance = pred.walking_distance();
    uint32_t path_distance = pred.path_distance();

    // If this is a transit edge - get the next departure. Do not check if allowed by
    // costing - assume if you get a transit edge you walked to the transit stop
    uint32_t tripid = 0;
    uint32_t blockid = 0;
    uint8_t restriction_idx = kInvalidRestriction;
    const bool is_dest = false;
    if (directededge->IsTransitLine()) {
      // Check if transit costing allows this edge
      if (!tc->Allowed(directededge, is_dest, pred, tile, edgeid, 0, 0, restriction_idx)) {
        continue;
      }

      // check if excluded.
      if (tc->IsExcluded(tile, directededge)) {
        continue;
      }

      // Look up the next departure along this edge
      const TransitDeparture* departure =
          tile->GetNextDeparture(directededge->lineid(), offset_time.day_seconds(), day_, dow_,
                                 date_before_tile_, tc->wheelchair(), tc->bicycle());
      if (departure) {
        // Check if there has been a mode change
        mode_change = (mode_ == travel_mode_t::kPedestrian);

        // Update trip Id and block Id
        tripid = departure->tripid();
        blockid = departure->blockid();
        has_transit = true;

        // There is no cost to remain on the same trip or valid blockId
        if (tripid == pred.tripid() || (blockid != 0 && blockid == pred.blockid())) {
          // This departure is valid without any added cost. Operator Id
          // is the same as the predecessor
          operator_id = pred.transit_operator();
        } else {
          if (pred.tripid() > 0) {
            // tripId > 0 means the prior edge was a transit edge and this
            // is an "in-station" transfer. Add a small transfer time and
            // call GetNextDeparture again if we cannot make the current
            // departure.
            // TODO - is there a better way?
            if (offset_time.day_seconds() + 30 > departure->departure_time()) {
              departure =
                  tile->GetNextDeparture(directededge->lineid(), offset_time.day_seconds() + 30, day_,
                                         dow_, date_before_tile_, tc->wheelchair(), tc->bicycle());
              if (!departure) {
                continue;
              }
            }
          }

          // Get the operator Id
          operator_id = GetOperatorId(tile, departure->routeindex(), operators_);

          // Add transfer penalty and operator change penalty
          if (pred.transit_operator() > 0 && pred.transit_operator() != operator_id) {
            // TODO - create a configurable operator change penalty
            newcost.cost += 300;
          } else {
            newcost.cost += transfer_cost.cost;
          }
        }

        // Change mode and costing to transit. Add edge cost.
        mode_ = travel_mode_t::kPublicTransit;
        newcost += tc->EdgeCost(directededge, departure, offset_time.day_seconds());
      } else {
        // No matching departures found for this edge
        continue;
      }
    } // This is not a transit edge
    else {
      // If current mode is public transit we should only connect to
      // transit connection edges or transit edges
      if (mode_ == travel_mode_t::kPublicTransit) {
        // Disembark from transit and reset walking distance
        mode_ = travel_mode_t::kPedestrian;
        walking_distance = 0;
        mode_change = true;
      } else {
        walking_distance += directededge->length();

        // Prevent going from one transit connection directly to another
        // at a transit stop - this is like entering a station and exiting
        // without getting on transit
        if (nodeinfo->type() == NodeType::kTransitEgress && pred.use() == Use::kTransitConnection &&
            directededge->use() == Use::kTransitConnection) {
          continue;
        }
      }

      // Regular edge - use the appropriate costing and check if access
      // is allowed. If mode is pedestrian this will validate walking
      // distance has not been exceeded.
      if (!pc->Allowed(directededge, false, pred, tile, edgeid, 0, 0, restriction_idx) ||
          walking_distance > max_walking_dist_) {
        continue;
      }

      Cost c = pc->EdgeCost(directededge, tile);
      c.cost *= pc->GetModeFactor();
      newcost += c;
    }

    // Add mode change cost or edge transition cost from the costing model
    Cost transition_cost{};
    if (mode_change) {
      // TODO: make mode change cost configurable. No cost for entering
      // a transit line (assume the wait time is the cost)
      // transition_cost = { 10.0f, 10.0f };
    } else {
      transition_cost =
          mode_costing[static_cast<uint32_t>(mode_)]->TransitionCost(directededge, nodeinfo, pred);
    }
    newcost += transition_cost;

    // Prohibit entering the same station as the prior.
    // TODO: if pred was a station, we wouldn't know about it; we only set
    // prior_stopid for platforms so far; also pull this check much further up
    if (directededge->use() == Use::kPlatformConnection &&
        directededge->endnode() == pred.prior_stopid()) {
      continue;
    }

    // Test if exceeding maximum transfer walking distance
    // TODO: transfer distance != walking distance! (one more label member?)
    if (directededge->use() == Use::kPlatformConnection && pred.prior_stopid().Is_Valid() &&
        walking_distance > max_transfer_distance_) {
      continue;
    }

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change). Update
    // trip Id and block Id.
    if (es->set() == EdgeSet::kTemporary) {
      MMEdgeLabel& lab = mmedgelabels_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        mmadjacencylist_.decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, path_distance, walking_distance, tripid, blockid,
                   transition_cost, restriction_idx);
      }
      continue;
    }

    // Add edge label, add to the adjacency list and set edge status
    uint32_t idx = mmedgelabels_.size();
    *es = {EdgeSet::kTemporary, idx};
    mmedgelabels_.emplace_back(pred_idx, edgeid, directededge, newcost, newcost.cost, 0.0f, mode_,
                               path_distance, walking_distance, tripid, prior_stop, blockid,
                               operator_id, has_transit, transition_cost, restriction_idx,
                               pred.path_id());
    mmadjacencylist_.add(idx);
  }

  // Handle transitions - expand from the end node of each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      ExpandForwardMultiModal(graphreader, trans->endnode(), pred, pred_idx, true, pc, tc,
                              mode_costing, offset_time);
    }
  }
}

// Compute the reverse graph traversal for multimodal
void Dijkstras::ComputeMultiModal(
    google::protobuf::RepeatedPtrField<valhalla::Location>& origin_locations,
    GraphReader& graphreader,
    const sif::mode_costing_t& mode_costing,
    const travel_mode_t mode,
    const Options& options) {

  // For now the date_time must be set on the origin.
  if (origin_locations.Get(0).date_time().empty()) {
    LOG_ERROR("No date time set on the origin location");
    return;
  }

  // For pedestrian costing - set flag allowing use of transit connections
  // Set pedestrian costing to use max distance. TODO - need for other modes
  const auto& pc = mode_costing[static_cast<uint8_t>(travel_mode_t::kPedestrian)];
  pc->SetAllowTransitConnections(true);

  // Set the mode from the origin
  mode_ = mode;
  const auto& tc = mode_costing[static_cast<uint8_t>(travel_mode_t::kPublicTransit)];

  // Get maximum transfer and walking distance
  max_transfer_distance_ = pc->GetMaxTransferDistanceMM();
  max_walking_dist_ =
      options.costings().find(Costing::pedestrian)->second.options().transit_start_end_max_distance();

  // Get the time information for all the origin locations
  auto time_infos = SetTime(origin_locations, graphreader);

  // Prepare for graph traversal
  Initialize(mmedgelabels_, mmadjacencylist_, mode_costing[static_cast<uint8_t>(mode_)]->UnitSize());
  SetOriginLocationsMultiModal(graphreader, origin_locations, pc);

  // Update start time
  date_set_ = false;
  date_before_tile_ = false;
  origin_date_time_ = origin_locations.Get(0).date_time();

  // Clear operators and processed tiles
  operators_.clear();
  processed_tiles_.clear();

  // Expand using adjacency list until we exceed threshold
  auto cb_decision = ExpansionRecommendation::continue_expansion;
  while (cb_decision != ExpansionRecommendation::stop_expansion) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    const uint32_t predindex = mmadjacencylist_.pop();
    if (predindex == kInvalidLabel) {
      break;
    }

    // Copy the EdgeLabel for use in costing and settle the edge.
    MMEdgeLabel pred = mmedgelabels_[predindex];
    edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent, pred.path_id());

    // Check if we should stop
    cb_decision = ShouldExpand(graphreader, pred, ExpansionType::multimodal);
    if (cb_decision != ExpansionRecommendation::prune_expansion) {
      // Expand from the end node of the predecessor edge.
      ExpandForwardMultiModal(graphreader, pred.endnode(), pred, predindex, false, pc, tc,
                              mode_costing, time_infos.front());
    }
  }
}

// Add edge(s) at each origin to the adjacency list
void Dijkstras::SetOriginLocations(GraphReader& graphreader,
                                   google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                                   const std::shared_ptr<DynamicCost>& costing) {
  // Bail if you want to do a multipath expansion with more locations than edge label/status supports
  if (multipath_ && locations.size() > baldr::kMaxMultiPathId)
    throw std::runtime_error("Max number of locations exceeded");

  // Add edges for each location to the adjacency list
  uint8_t path_id = -1;
  for (auto& location : locations) {
    ++path_id;

    // Only skip inbound edges if we have other options
    bool has_other_edges = false;
    std::for_each(location.correlation().edges().begin(), location.correlation().edges().end(),
                  [&has_other_edges](const valhalla::PathEdge& e) {
                    has_other_edges = has_other_edges || !e.end_node();
                  });

    // Iterate through edges and add to adjacency list
    for (const auto& edge : (location.correlation().edges())) {
      // If origin is at a node - skip any inbound edge (dist = 1)
      if (has_other_edges && edge.end_node()) {
        continue;
      }

      // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
      GraphId edgeid(edge.graph_id());
      if (costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Get the directed edge
      graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
      if (tile == nullptr) {
        continue;
      }
      const DirectedEdge* directededge = tile->directededge(edgeid);

      // Get the opposing directed edge, continue if we cannot get it
      graph_tile_ptr opp_tile = nullptr;
      GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid, opp_tile);
      if (!opp_edge_id.Is_Valid()) {
        continue;
      }

      // Get cost
      uint8_t flow_sources;
      Cost cost = costing->EdgeCost(directededge, tile, TimeInfo::invalid(), flow_sources) *
                  (1.0f - edge.percent_along());
      // Get path distance
      auto path_dist = directededge->length() * (1 - edge.percent_along());

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: high edge scores cause issues as there is code to limit cost so
      // that large penalties (e.g., ferries) are excluded.
      cost.cost += edge.distance();

      // Construct the edge label. Set the predecessor edge index to invalid
      // to indicate the origin of the path.
      uint32_t idx = bdedgelabels_.size();
      bdedgelabels_.emplace_back(kInvalidLabel, edgeid, opp_edge_id, directededge, cost, mode_,
                                 Cost{}, path_dist, false, !(costing_->IsClosed(directededge, tile)),
                                 static_cast<bool>(flow_sources & kDefaultFlowMask),
                                 InternalTurn::kNoTurn, kInvalidRestriction, multipath_ ? path_id : 0,
                                 directededge->destonly() ||
                                     (costing_->is_hgv() && directededge->destonly_hgv()),
                                 directededge->forwardaccess() & kTruckAccess);
      // Set the origin flag
      bdedgelabels_.back().set_origin();

      // Add EdgeLabel to the adjacency list
      adjacencylist_.add(idx);
      edgestatus_.Set(edgeid, EdgeSet::kTemporary, idx, tile, multipath_ ? path_id : 0);
    }
  }
}

// Add destination edges to the reverse path adjacency list.
void Dijkstras::SetDestinationLocations(
    GraphReader& graphreader,
    google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
    const std::shared_ptr<DynamicCost>& costing) {
  // Bail if you want to do a multipath expansion with more locations than edge label/status supports
  if (multipath_ && locations.size() > baldr::kMaxMultiPathId)
    throw std::runtime_error("Max number of locations exceeded");

  // Add edges for each location to the adjacency list
  uint8_t path_id = -1;
  for (auto& location : locations) {
    ++path_id;

    // Only skip outbound edges if we have other options
    bool has_other_edges = false;
    std::for_each(location.correlation().edges().begin(), location.correlation().edges().end(),
                  [&has_other_edges](const valhalla::PathEdge& e) {
                    has_other_edges = has_other_edges || !e.begin_node();
                  });

    // Iterate through edges and add to adjacency list
    for (const auto& edge : (location.correlation().edges())) {
      // If the destination is at a node, skip any outbound edges (so any
      // opposing inbound edges are not considered)
      if (has_other_edges && edge.begin_node()) {
        continue;
      }

      // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
      GraphId edgeid(edge.graph_id());
      if (costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Get the directed edge
      graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
      if (tile == nullptr) {
        continue;
      }
      const DirectedEdge* directededge = tile->directededge(edgeid);

      // Get the opposing directed edge, continue if we cannot get it
      graph_tile_ptr opp_tile = tile;
      const DirectedEdge* opp_dir_edge = nullptr;
      auto opp_edge_id = graphreader.GetOpposingEdgeId(edgeid, opp_dir_edge, opp_tile);
      if (opp_dir_edge == nullptr) {
        continue;
      }

      // Get the cost
      uint8_t flow_sources;
      Cost cost = costing->EdgeCost(directededge, tile, TimeInfo::invalid(), flow_sources) *
                  edge.percent_along();
      // Get the path distance
      auto path_dist = directededge->length() * edge.percent_along();

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: high edge scores cause issues as there is code to limit cost so
      // that large penalties (e.g., ferries) are excluded.
      cost.cost += edge.distance();

      // Add EdgeLabel to the adjacency list. Set the predecessor edge index
      // to invalid to indicate the origin of the path. Make sure the opposing
      // edge (edgeid) is set.
      uint32_t idx = bdedgelabels_.size();
      int restriction_idx = -1;
      // TODO: When running expansion in reverse, handle the case where the
      // destination lies on a closure but the expansion started from an open
      // edge. Currently, we begin with closure prunning turned on and hence
      // don't expand into closures. This results in pessimistic reach. What
      // we want is for the expansion to continue when it encounters the first
      // closure and stop when it exits the closure (which can span multiple
      // consecutive edges)
      bdedgelabels_.emplace_back(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, mode_,
                                 Cost{}, path_dist, false, !(costing_->IsClosed(directededge, tile)),
                                 static_cast<bool>(flow_sources & kDefaultFlowMask),
                                 InternalTurn::kNoTurn, restriction_idx, multipath_ ? path_id : 0,
                                 directededge->destonly() ||
                                     (costing_->is_hgv() && directededge->destonly_hgv()),
                                 directededge->forwardaccess() & kTruckAccess);
      adjacencylist_.add(idx);
      edgestatus_.Set(opp_edge_id, EdgeSet::kTemporary, idx, opp_tile, multipath_ ? path_id : 0);
    }
  }
}

// Add edge(s) at each origin to the adjacency list
void Dijkstras::SetOriginLocationsMultiModal(
    GraphReader& graphreader,
    google::protobuf::RepeatedPtrField<valhalla::Location>& origin_locations,
    const std::shared_ptr<DynamicCost>& costing) {
  // Add edges for each location to the adjacency list
  for (auto& origin : origin_locations) {
    // Only skip inbound edges if we have other options
    bool has_other_edges = false;
    std::for_each(origin.correlation().edges().begin(), origin.correlation().edges().end(),
                  [&has_other_edges](const valhalla::PathEdge& e) {
                    has_other_edges = has_other_edges || !e.end_node();
                  });

    // keep the nodeinfo object to set timezone properly at the end
    const NodeInfo* closest_ni = nullptr;
    // Iterate through edges and add to adjacency list
    for (const auto& edge : (origin.correlation().edges())) {
      // If origin is at a node - skip any inbound edge (dist = 1)
      if (has_other_edges && edge.end_node()) {
        continue;
      }

      // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
      GraphId edgeid(edge.graph_id());
      if (costing->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Get the directed edge
      graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
      if (tile == nullptr) {
        continue;
      }
      const DirectedEdge* directededge = tile->directededge(edgeid);

      // Get the tile at the end node. Skip if tile not found as we won't be
      // able to expand from this origin edge.
      graph_tile_ptr endtile = graphreader.GetGraphTile(directededge->endnode());
      if (endtile == nullptr) {
        continue;
      }
      if (closest_ni == nullptr) {
        closest_ni = endtile->node(directededge->endnode());
      }

      // Get cost
      Cost cost = costing->EdgeCost(directededge, endtile) * (1.0f - edge.percent_along());

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      cost.cost += edge.distance();

      uint32_t d = static_cast<uint32_t>(directededge->length() * (1.0f - edge.percent_along()));
      MMEdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost, cost.cost, 0.0f, mode_, d, d,
                             0, GraphId(), 0, 0, false, Cost{}, baldr::kInvalidRestriction);
      // Set the origin flag
      edge_label.set_origin();

      // Add EdgeLabel to the adjacency list
      uint32_t idx = mmedgelabels_.size();
      mmedgelabels_.push_back(edge_label);
      mmadjacencylist_.add(idx);
      edgestatus_.Set(edgeid, EdgeSet::kTemporary, idx, tile, 0);
    }

    // Set the origin timezone
    if (closest_ni != nullptr && !origin.date_time().empty() && origin.date_time() == "current") {
      origin.set_date_time(
          DateTime::iso_date_time(DateTime::get_tz_db().from_index(closest_ni->timezone())));
    }
  }
}

} // namespace thor
} // namespace valhalla
