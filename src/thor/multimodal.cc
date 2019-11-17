#include "thor/multimodal.h"
#include "baldr/datetime.h"
#include "midgard/logging.h"
#include "worker.h"
#include <algorithm>
#include <map>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

// Method to get an operator Id from a map of operator strings vs. Id.
uint32_t GetOperatorId(const GraphTile* tile,
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

constexpr uint64_t kInitialEdgeLabelCount = 200000;

// Default constructor
MultiModalPathAlgorithm::MultiModalPathAlgorithm()
    : PathAlgorithm(), walking_distance_(0), mode_(TravelMode::kPedestrian), travel_type_(0),
      adjacencylist_(nullptr), max_label_count_(std::numeric_limits<uint32_t>::max()) {
}

// Destructor
MultiModalPathAlgorithm::~MultiModalPathAlgorithm() {
  Clear();
}

// Initialize prior to finding best path
void MultiModalPathAlgorithm::Init(const midgard::PointLL& origll,
                                   const midgard::PointLL& destll,
                                   const std::shared_ptr<DynamicCost>& costing) {
  // Disable A* for multimodal
  astarheuristic_.Init(destll, 0.0f);

  // Reserve size for edge labels - do this here rather than in constructor so
  // to limit how much extra memory is used for persistent objects
  edgelabels_.reserve(kInitialEdgeLabelCount);

  // Set up lambda to get sort costs
  const auto edgecost = [this](const uint32_t label) { return edgelabels_[label].sortcost(); };

  // Construct adjacency list and edge status.
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_.reset(new DoubleBucketQueue(0.0f, range, bucketsize, edgecost));
  edgestatus_.clear();

  // Get hierarchy limits from the costing. Get a copy since we increment
  // transition counts (i.e., this is not a const reference).
  hierarchy_limits_ = costing->GetHierarchyLimits();
}

// Clear the temporary information generated during path construction.
void MultiModalPathAlgorithm::Clear() {
  // Clear the edge labels and destination list
  edgelabels_.clear();
  destinations_.clear();

  // Clear elements from the adjacency list
  adjacencylist_.reset();

  // Clear the edge status flags
  edgestatus_.clear();

  // Set the ferry flag to false
  has_ferry_ = false;
}

// Calculate best path using multiple modes (e.g. transit).
std::vector<std::vector<PathInfo>>
MultiModalPathAlgorithm::GetBestPath(valhalla::Location& origin,
                                     valhalla::Location& destination,
                                     GraphReader& graphreader,
                                     const std::shared_ptr<DynamicCost>* mode_costing,
                                     const TravelMode mode,
                                     const Options& options) {
  // For pedestrian costing - set flag allowing use of transit connections
  // Set pedestrian costing to use max distance. TODO - need for other modes
  const auto& pc = mode_costing[static_cast<uint32_t>(TravelMode::kPedestrian)];
  pc->SetAllowTransitConnections(true);
  pc->UseMaxMultiModalDistance();

  // Set the mode from the origin
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode)];
  const auto& tc = mode_costing[static_cast<uint32_t>(TravelMode::kPublicTransit)];

  // Get maximum transfer distance
  max_transfer_distance_ = costing->GetMaxTransferDistanceMM();

  // For now the date_time must be set on the origin.
  if (!origin.has_date_time()) {
    return {};
  };

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  // Note: because we can correlate to more than one place for a given PathLocation
  // using edges.front here means we are only setting the heuristics to one of them
  // alternate paths using the other correlated points to may be harder to find
  midgard::PointLL origin_new(origin.path_edges(0).ll().lng(), origin.path_edges(0).ll().lat());
  midgard::PointLL destination_new(destination.path_edges(0).ll().lng(),
                                   destination.path_edges(0).ll().lat());
  Init(origin_new, destination_new, costing);
  float mindist = astarheuristic_.GetDistance(origin_new);

  // Check if there no possible path to destination based on mode to the
  // destination - for now assume pedestrian
  // TODO - some means of setting destination mode
  disable_transit_ = false;
  if (!CanReachDestination(destination, graphreader, TravelMode::kPedestrian, pc)) {
    // Return if distance exceeds maximum distance set for the starting distance
    // of a multimodal route (TODO - add methods to costing to support this).
    if (mindist > 2000) {
      // Throw an exception so the message is returned in the service
      throw valhalla_exception_t{440};
    } else {
      // Allow routing but disable use of transit
      disable_transit_ = true;
    }
  }

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  SetDestination(graphreader, destination, costing);
  SetOrigin(graphreader, origin, destination, costing);

  // Set route start time (seconds from midnight) and timezone.
  // NOTe: already made sure origin has date_time set.
  date_before_tile_ = false;
  date_set_ = false;
  origin_date_time_ = origin.date_time();

  start_time_ = DateTime::seconds_from_midnight(origin_date_time_);
  start_tz_index_ = edgelabels_.size() == 0 ? 0 : GetTimezone(graphreader, edgelabels_[0].endnode());
  if (start_tz_index_ == 0) {
    // TODO - should we throw an exception and return an error
    LOG_ERROR("Could not get the timezone at the origin location");
    return {};
  }

  // Clear operators and processed tiles
  operators_.clear();
  processed_tiles_.clear();

  // Find shortest path
  uint32_t nc = 0; // Count of iterations with no convergence
                   // towards destination
  const GraphTile* tile;
  size_t total_labels = 0;
  while (true) {
    // Allow this process to be aborted
    size_t current_labels = edgelabels_.size();
    if (interrupt &&
        total_labels / kInterruptIterationsInterval < current_labels / kInterruptIterationsInterval) {
      (*interrupt)();
    }
    total_labels = current_labels;

    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->pop();
    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " + std::to_string(edgelabels_.size()));
      return {};
    }

    // Copy the EdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    MMEdgeLabel pred = edgelabels_[predindex];
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
    } else if (nc++ > 500000) {
      return {};
    }

    // Expand from the end node of the predecessor edge.
    ExpandForward(graphreader, pred.endnode(), pred, predindex, false, pc, tc, mode_costing);
  }
  return {}; // Should never get here
}

// Expand from a node using multi-modal algorithm.
bool MultiModalPathAlgorithm::ExpandForward(GraphReader& graphreader,
                                            const GraphId& node,
                                            const MMEdgeLabel& pred,
                                            const uint32_t pred_idx,
                                            const bool from_transition,
                                            const std::shared_ptr<DynamicCost>& pc,
                                            const std::shared_ptr<DynamicCost>& tc,
                                            const std::shared_ptr<DynamicCost>* mode_costing) {

  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return false;
  }
  const NodeInfo* nodeinfo = tile->node(node);

  if (nodeinfo->type() == NodeType::kMultiUseTransitPlatform ||
      nodeinfo->type() == NodeType::kTransitStation) {

    if (processed_tiles_.find(tile->id().tileid()) == processed_tiles_.end()) {
      tc->AddToExcludeList(tile);
      processed_tiles_.emplace(tile->id().tileid());
    }

    // check if excluded.
    if (tc->IsExcluded(tile, nodeinfo)) {
      return false;
    }
  }

  // Set local time and adjust for time zone (if different from timezone at the start).
  uint32_t localtime = start_time_ + pred.cost().secs;
  if (nodeinfo->timezone() != start_tz_index_) {
    // Get the difference in seconds between the origin tz and current tz
    int tz_diff =
        DateTime::timezone_diff(localtime, DateTime::get_tz_db().from_index(start_tz_index_),
                                DateTime::get_tz_db().from_index(nodeinfo->timezone()));
    localtime += tz_diff;
  }

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
    if (mode_ == TravelMode::kPedestrian && prior_stop.Is_Valid() && has_transit) {
      transfer_cost = tc->TransferCost();
    }

    // Add transfer time to the local time when entering a stop
    // as a pedestrian. This is a small added cost on top of
    // any costs along paths and roads
    if (mode_ == TravelMode::kPedestrian) {
      localtime += transfer_cost.secs;
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

  // Allow mode changes at special nodes
  //      bike share (pedestrian <--> bicycle)
  //      parking (drive <--> pedestrian)
  //      transit stop (pedestrian <--> transit).
  // TODO - evaluate how this will work when an edge may have already
  // been visited using a different mode.
  bool mode_change = false;
  /*if (nodeinfo->type() == NodeType::kBikeShare) {
     if (mode_ == TravelMode::kBicycle) {
       mode_ = TravelMode::kPedestrian;
       mode_change = true;
     } else if (mode_ == TravelMode::kPedestrian) {
       mode_ = TravelMode::kBicycle;
       mode_change = true;
     }
   } else if (nodeinfo->type() == NodeType::kParking) {
     if (mode_ == TravelMode::kDrive) {
       mode_ = TravelMode::kPedestrian;
       mode_change = true;
     } else if (mode_ == TravelMode::kPedestrian) {
       mode_ = TravelMode::kDrive;
       mode_change = true;
     }
   }*/

  // Expand from end node.
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  EdgeStatusInfo* es = edgestatus_.GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, ++edgeid, ++es) {
    // Skip shortcuts and edges that are permanently labeled (best path already found to
    // this directed edge).
    if (directededge->is_shortcut() || es->set() == EdgeSet::kPermanent) {
      continue;
    }

    // Reset cost and walking distance
    Cost newcost = pred.cost();
    walking_distance_ = pred.path_distance();

    // If this is a transit edge - get the next departure. Do not check
    // if allowed by costing - assume if you get a transit edge you
    // walked to the transit stop
    uint32_t tripid = 0;
    uint32_t blockid = 0;
    bool has_time_restrictions;
    if (directededge->IsTransitLine()) {
      // Check if transit costing allows this edge
      if (!tc->Allowed(directededge, pred, tile, edgeid, 0, 0, has_time_restrictions)) {
        continue;
      }
      // check if excluded.
      if (tc->IsExcluded(tile, directededge)) {
        continue;
      }

      // Look up the next departure along this edge
      const TransitDeparture* departure =
          tile->GetNextDeparture(directededge->lineid(), localtime, day_, dow_, date_before_tile_,
                                 tc->wheelchair(), tc->bicycle());

      if (departure) {
        // Check if there has been a mode change
        mode_change = (mode_ == TravelMode::kPedestrian);

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
            if (localtime + 30 > departure->departure_time()) {
              departure = tile->GetNextDeparture(directededge->lineid(), localtime + 30, day_, dow_,
                                                 date_before_tile_, tc->wheelchair(), tc->bicycle());
              if (!departure) {
                continue;
              }
            }
          }

          // Get the operator Id
          operator_id = GetOperatorId(tile, departure->routeid(), operators_);

          // Add transfer penalty and operator change penalty
          if (pred.transit_operator() > 0 && pred.transit_operator() != operator_id) {
            // TODO - create a configurable operator change penalty
            newcost.cost += 300;
          } else {
            newcost.cost += transfer_cost.cost;
          }
        }

        // Change mode and costing to transit. Add edge cost.
        mode_ = TravelMode::kPublicTransit;
        newcost += tc->EdgeCost(directededge, departure, localtime);
      } else {
        // No matching departures found for this edge
        continue;
      }
    } else {
      // If current mode is public transit we should only connect to
      // transit connection edges or transit edges
      if (mode_ == TravelMode::kPublicTransit) {
        // Disembark from transit and reset walking distance
        mode_ = TravelMode::kPedestrian;
        walking_distance_ = 0;
        mode_change = true;
      }

      // Regular edge - use the appropriate costing and check if access
      // is allowed. If mode is pedestrian this will validate walking
      // distance has not been exceeded.
      if (!mode_costing[static_cast<uint32_t>(mode_)]->Allowed(directededge, pred, tile, edgeid, 0, 0,
                                                               has_time_restrictions)) {
        continue;
      }

      Cost c = mode_costing[static_cast<uint32_t>(mode_)]->EdgeCost(directededge, tile);
      c.cost *= mode_costing[static_cast<uint32_t>(mode_)]->GetModeFactor();
      newcost += c;

      // Add to walking distance
      if (mode_ == TravelMode::kPedestrian) {
        walking_distance_ += directededge->length();

        // Prevent going from one transit connection directly to another
        // at a transit stop - this is like entering a station and exiting
        // without getting on transit
        if (nodeinfo->type() == NodeType::kTransitEgress && pred.use() == Use::kTransitConnection &&
            directededge->use() == Use::kTransitConnection) {
          continue;
        }
      }
    }

    // Add mode change cost or edge transition cost from the costing model
    if (mode_change) {
      // TODO: make mode change cost configurable. No cost for entering
      // a transit line (assume the wait time is the cost)
      ; // newcost += {10.0f, 10.0f };
    } else {
      newcost +=
          mode_costing[static_cast<uint32_t>(mode_)]->TransitionCost(directededge, nodeinfo, pred);
    }

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge)
    auto p = destinations_.find(edgeid);
    if (p != destinations_.end()) {
      newcost -= p->second;
    }

    // Do not allow transit connection edges if transit is disabled. Also,
    // prohibit entering the same station as the prior.
    if (directededge->use() == Use::kPlatformConnection &&
        (disable_transit_ || directededge->endnode() == pred.prior_stopid())) {
      continue;
    }

    // Test if exceeding maximum transfer walking distance
    if (directededge->use() == Use::kPlatformConnection && pred.prior_stopid().Is_Valid() &&
        walking_distance_ > max_transfer_distance_) {
      continue;
    }

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change). Update
    // trip Id and block Id.
    if (es->set() == EdgeSet::kTemporary) {
      MMEdgeLabel& lab = edgelabels_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_->decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, walking_distance_, tripid, blockid,
                   has_time_restrictions);
      }
      continue;
    }

    // If this is a destination edge the A* heuristic is 0. Otherwise the
    // sort cost (with A* heuristic) is found using the lat,lng at the
    // end node of the directed edge.
    float dist = 0.0f;
    float sortcost = newcost.cost;
    if (p == destinations_.end()) {
      // Get the end node, skip if the end node tile is not found
      const GraphTile* endtile =
          (directededge->leaves_tile()) ? graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (endtile == nullptr) {
        continue;
      }
      const NodeInfo* endnode = endtile->node(directededge->endnode());
      dist = astarheuristic_.GetDistance(endnode->latlng(endtile->header()->base_ll()));
      sortcost += astarheuristic_.Get(dist);
    }

    // Add edge label, add to the adjacency list and set edge status
    uint32_t idx = edgelabels_.size();
    *es = {EdgeSet::kTemporary, idx};
    edgelabels_.emplace_back(pred_idx, edgeid, directededge, newcost, sortcost, dist, mode_,
                             walking_distance_, tripid, prior_stop, blockid, operator_id,
                             has_transit);
    adjacencylist_->add(idx);
  }

  // Handle transitions - expand from the end node each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true, pc, tc, mode_costing);
    }
  }
  return false;
}

// Add an edge at the origin to the adjacency list
void MultiModalPathAlgorithm::SetOrigin(GraphReader& graphreader,
                                        valhalla::Location& origin,
                                        const valhalla::Location& destination,
                                        const std::shared_ptr<DynamicCost>& costing) {
  // Only skip inbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.path_edges().begin(), origin.path_edges().end(),
                [&has_other_edges](const valhalla::Location::PathEdge& e) {
                  has_other_edges = has_other_edges || !e.end_node();
                });

  // Iterate through edges and add to adjacency list
  const NodeInfo* nodeinfo = nullptr;
  const NodeInfo* closest_ni = nullptr;
  for (const auto& edge : origin.path_edges()) {
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
    Cost cost = costing->EdgeCost(directededge, tile) * (1.0f - edge.percent_along());
    float dist = astarheuristic_.GetDistance(nodeinfo->latlng(endtile->header()->base_ll()));

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
            const DirectedEdge* dest_diredge =
                tile->directededge(GraphId(destination_edge.graph_id()));
            Cost dest_cost =
                costing->EdgeCost(dest_diredge, tile) * (1.0f - destination_edge.percent_along());
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
    MMEdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost, sortcost, dist, mode_, d, 0,
                           GraphId(), 0, 0, false);
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
uint32_t MultiModalPathAlgorithm::SetDestination(GraphReader& graphreader,
                                                 const valhalla::Location& dest,
                                                 const std::shared_ptr<DynamicCost>& costing) {
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
    if (costing->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
      continue;
    }

    // Keep the cost to traverse the partial distance for the remainder of the edge. This cost
    // is subtracted from the total cost up to the end of the destination edge.
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* dest_diredge = tile->directededge(edgeid);
    destinations_[edge.graph_id()] =
        costing->EdgeCost(dest_diredge, tile) * (1.0f - edge.percent_along());

    // We need to penalize this location based on its score (distance in meters from input)
    // We assume the slowest speed you could travel to cover that distance to start/end the route
    // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
    destinations_[edge.graph_id()].cost += edge.distance();

    // Get the tile relative density
    density = tile->header()->density();
  }
  return density;
}

// Expand from the node along the forward search path. Immediately expands from the end node
// of any transition edge (so no transition edges are added to the adjacency list or EdgeLabel
// list). Does not expand transition edges if from_transition is false. This method is only
// used in CanReachDestination.
bool MultiModalPathAlgorithm::ExpandFromNode(baldr::GraphReader& graphreader,
                                             const baldr::GraphId& node,
                                             const sif::EdgeLabel& pred,
                                             const uint32_t pred_idx,
                                             const std::shared_ptr<DynamicCost>& costing,
                                             EdgeStatus& edgestatus,
                                             std::vector<EdgeLabel>& edgelabels,
                                             DoubleBucketQueue& adjlist,
                                             const bool from_transition) {
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return false;
  }
  const NodeInfo* nodeinfo = tile->node(node);
  if (!costing->Allowed(nodeinfo)) {
    return false;
  }

  // Return true if we reach a transit stop
  if (nodeinfo->type() == NodeType::kMultiUseTransitPlatform) {
    return true;
  }

  // Expand edges from the node
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  EdgeStatusInfo* es = edgestatus.GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, ++edgeid, ++es) {
    // Skip this edge if permanently labeled (best path already found to this directed edge) or
    // access is not allowed for this mode.
    bool has_time_restrictions;
    if (es->set() == EdgeSet::kPermanent ||
        !costing->Allowed(directededge, pred, tile, edgeid, 0, 0, has_time_restrictions)) {
      continue;
    }

    // Get cost
    Cost newcost = pred.cost() + costing->EdgeCost(directededge, tile) +
                   costing->TransitionCost(directededge, nodeinfo, pred);
    uint32_t walking_distance = pred.path_distance() + directededge->length();

    // Check if lower cost path
    if (es->set() == EdgeSet::kTemporary) {
      EdgeLabel& lab = edgelabels[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjlist.decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, walking_distance, has_time_restrictions);
      }
      continue;
    }

    // Add edge label, add to the adjacency list and set edge status
    uint32_t idx = edgelabels.size();
    edgelabels.emplace_back(pred_idx, edgeid, directededge, newcost, newcost.cost, 0.0f, mode_,
                            walking_distance);
    *es = {EdgeSet::kTemporary, idx};
    adjlist.add(idx);
  }

  // Handle transitions - expand from the end node each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      ExpandFromNode(graphreader, trans->endnode(), pred, pred_idx, costing, edgestatus, edgelabels,
                     adjlist, true);
    }
  }
  return false;
}

// Check if destination can be reached if walking is the last mode. Checks
// if there are any transit stops within maximum walking distance.
// TODO - once auto/bicycle are allowed modes we need to check if parking
// or bikeshare locations are within walking distance.
bool MultiModalPathAlgorithm::CanReachDestination(const valhalla::Location& destination,
                                                  GraphReader& graphreader,
                                                  const TravelMode dest_mode,
                                                  const std::shared_ptr<DynamicCost>& costing) {
  // Assume pedestrian mode for now
  mode_ = dest_mode;

  // Local edge labels and edge status info
  EdgeStatus edgestatus;
  std::vector<EdgeLabel> edgelabels;

  // Set up lambda to get sort costs (use the local edgelabels, not the class member!)
  const auto edgecost = [&edgelabels](const uint32_t label) { return edgelabels[label].sortcost(); };

  // Use a simple Dijkstra method - no need to recover the path just need to make sure we can
  // get to a transit stop within the specified max. walking distance
  uint32_t bucketsize = costing->UnitSize();
  DoubleBucketQueue adjlist(0.0f, kBucketCount * bucketsize, bucketsize, edgecost);

  // Add the opposing destination edges to the priority queue
  uint32_t label_idx = 0;
  for (const auto& edge : destination.path_edges()) {
    // Keep the id and the cost to traverse the partial distance
    float ratio = (1.0f - edge.percent_along());
    GraphId id(edge.graph_id());
    GraphId oppedge = graphreader.GetOpposingEdgeId(id);

    // Disallow any user avoided edges if the avoid location is behind the destination along the edge
    GraphId edgeid(edge.graph_id());
    if (costing->AvoidAsDestinationEdge(edgeid, ratio)) {
      continue;
    }

    const GraphTile* tile = graphreader.GetGraphTile(oppedge);
    const DirectedEdge* diredge = tile->directededge(oppedge);
    uint32_t length = static_cast<uint32_t>(diredge->length()) * ratio;
    Cost cost = costing->EdgeCost(diredge, tile) * ratio;
    edgelabels.emplace_back(kInvalidLabel, oppedge, diredge, cost, cost.cost, 0.0f, mode_, length);
    adjlist.add(label_idx);
    edgestatus.Set(oppedge, EdgeSet::kTemporary, label_idx, tile);
    label_idx++;
  }

  // Get the next edge from they priority queue until either a transit stop has been found, we exceed
  // the maximum walking distance, or there are no edges in the priority queue.
  // TODO - really should traverse in reverse direction - but since pedestrian access should be the
  // same in either direction we will traverse in a forward direction for now
  uint32_t predindex;
  while ((predindex = adjlist.pop()) != kInvalidLabel) {
    // Mark the edge as as permanently labeled - copy the EdgeLabel for use in costing
    EdgeLabel pred = edgelabels[predindex];
    edgestatus.Update(pred.edgeid(), EdgeSet::kPermanent);

    // Expand from the end node of the predecessor
    if (ExpandFromNode(graphreader, pred.endnode(), pred, predindex, costing, edgestatus, edgelabels,
                       adjlist, false)) {
      return true;
    }
  }
  return false;
}

// Form the path from the adjacency list.
std::vector<PathInfo> MultiModalPathAlgorithm::FormPath(const uint32_t dest) {
  // Metrics to track
  LOG_DEBUG("path_cost::" + std::to_string(edgelabels_[dest].cost().cost));
  LOG_DEBUG("path_iterations::" + std::to_string(edgelabels_.size()));

  // Work backwards from the destination
  std::vector<PathInfo> path;
  for (auto edgelabel_index = dest; edgelabel_index != kInvalidLabel;
       edgelabel_index = edgelabels_[edgelabel_index].predecessor()) {
    const MMEdgeLabel& edgelabel = edgelabels_[edgelabel_index];
    path.emplace_back(edgelabel.mode(), edgelabel.cost().secs, edgelabel.edgeid(), edgelabel.tripid(),
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
