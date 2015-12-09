#include <map>
#include <algorithm>
#include "thor/pathalgorithm.h"
#include <valhalla/baldr/datetime.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

// Default constructor
MultiModalPathAlgorithm::MultiModalPathAlgorithm()
    : PathAlgorithm() {
}

// Destructor
MultiModalPathAlgorithm::~MultiModalPathAlgorithm() {
  Clear();
}

// Initialize prior to finding best path
void MultiModalPathAlgorithm::Init(const PointLL& origll,
                       const PointLL& destll,
                       const std::shared_ptr<DynamicCost>& costing) {
  // Disable A* for multimodal
  astarheuristic_.Init(destll, 0.0f);

  // Construct adjacency list, edge status, and done set
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_.reset(new AdjacencyList(0.0f, range, bucketsize));
  edgestatus_.reset(new EdgeStatus());

  // Get hierarchy limits from the costing. Get a copy since we increment
  // transition counts (i.e., this is not a const reference).
  allow_transitions_ = costing->AllowTransitions();
  hierarchy_limits_  = costing->GetHierarchyLimits();
}

// Calculate best path using multiple modes (e.g. transit).
std::vector<PathInfo> MultiModalPathAlgorithm::GetBestPath(
            PathLocation& origin, PathLocation& destination,
            GraphReader& graphreader,
            const std::shared_ptr<DynamicCost>* mode_costing,
            const TravelMode mode) {
  // For pedestrian costing - set flag allowing use of transit connections
  // Set pedestrian costing to use max distance. TODO - need for other modes
  const auto& pc = mode_costing[static_cast<uint32_t>(TravelMode::kPedestrian)];
  pc->SetAllowTransitConnections(true);
  pc->UseMaxModeDistance();

  // Check if there no possible path to destination based on mode to the
  // destination - for now assume pedestrian
  // TODO - some means of setting destination mode
  if (!CanReachDestination(destination, graphreader, TravelMode::kPedestrian, pc)) {
    LOG_INFO("Cannot reach destination - too far from a transit stop");
    return { };
  }

  // Set the mode from the origin
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode)];
  const auto& tc = mode_costing[static_cast<uint32_t>(TravelMode::kPublicTransit)];

  // For now the date_time must be set on the origin.
  if (!origin.date_time_)
    return { };

  uint32_t start_time, localtime, date, dow, day = 0;
  bool date_before_tile = false;
  if (origin.date_time_ && *origin.date_time_ != "current") {
    // Set route start time (seconds from midnight), date, and day of week
    start_time = DateTime::seconds_from_midnight(*origin.date_time_);
    localtime = start_time;
    date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(*origin.date_time_));
    dow  = DateTime::day_of_week_mask(*origin.date_time_);
    if (date < tile_creation_date_)
      date_before_tile = true;
    else
      day = date - tile_creation_date_;
  }

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  Init(origin.vertex(), destination.vertex(), costing);
  float mindist = astarheuristic_.GetDistance(origin.vertex());

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  SetDestination(graphreader, destination, costing);
  SetOrigin(graphreader, origin, destination, costing);

  // Find shortest path
  uint32_t blockid, tripid, prior_stop;
  uint32_t nc = 0;       // Count of iterations with no convergence
                         // towards destination
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->Remove(edgelabels_);
    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " +
                     std::to_string(edgelabels_.size()));
      return { };
    }

    // Copy the EdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    EdgeLabel pred = edgelabels_[predindex];
    if (destinations_.find(pred.edgeid()) != destinations_.end()) {
      // Check if a trivial path. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        if (IsTrivial(pred.edgeid(), origin, destination)) {
          return FormPath(predindex);
        }
      } else {
        return FormPath(predindex);
      }
    }

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge (this will allow loops/around the block cases)
    if (!pred.origin()) {
      edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);
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

    // Get the end node. Skip if tile not found (can happen with
    // regional data sets).
    GraphId node = pred.endnode();
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Check access at the node
    const NodeInfo* nodeinfo = tile->node(node);
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    if (pred.origin() && origin.date_time_ && *origin.date_time_ == "current") {
      origin.date_time_= DateTime::iso_date_time(DateTime::get_tz_db().from_index(nodeinfo->timezone()));
      // Set route start time (seconds from midnight), date, and day of week
      start_time = DateTime::seconds_from_midnight(*origin.date_time_);
      localtime = start_time;
      date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(*origin.date_time_));
      dow  = DateTime::day_of_week_mask(*origin.date_time_);
      if (date < tile_creation_date_)
        date_before_tile = true;
      else
        day = date - tile_creation_date_;
    }

    // Set a default transfer at a stop (if not same trip Id and block Id)
    // TODO - support in transit costing method
    Cost transfer_cost = { 300.0f, 60.0f };

    // Get any transfer times and penalties if this is a transit stop (and
    // transit has been taken at some point on the path) and mode is pedestrian
    mode_ = pred.mode();
    bool has_transit = pred.has_transit();
    uint32_t prior_stop = pred.prior_stopid();
    if (nodeinfo->type() == NodeType::kMultiUseTransitStop) {
      if (mode_ == TravelMode::kPedestrian && prior_stop != 0 && has_transit) {
        transfer_cost = tc->TransferCost(tile->GetTransfer(prior_stop,
                                      nodeinfo->stop_index()));
      }

      // Update prior stop.
      // TODO - parent/child stop info?
      prior_stop = nodeinfo->stop_index();
    }

    // Set local time. TODO: adjust for time zone. Add true transfer time.
    // Update transfer cost so all that remains is the cost penalty
    uint32_t localtime = start_time + pred.cost().secs + transfer_cost.secs;
    transfer_cost.cost -= transfer_cost.secs;
    transfer_cost.secs = 0.0f;

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
    uint32_t shortcuts = 0;
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                i++, directededge++, edgeid++) {
      // Skip transition edges for now. Should not see any shortcuts since we
      // never transition upwards.
      if (directededge->trans_up() || directededge->trans_down()) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.set() == EdgeSet::kPermanent) {
        continue;
      }

      // Reset cost and walking distance
      Cost newcost = pred.cost();
      walking_distance_ = pred.walking_distance();

      // If this is a transit edge - get the next departure. Do not check
      // if allowed by costing - assume if you get a transit edge you
      // walked to the transit stop
      tripid = 0;
      if (directededge->IsTransitLine()) {
        const TransitDeparture* departure = tile->GetNextDeparture(
                    directededge->lineid(), localtime, day, dow, date_before_tile);
        if (departure) {
          // Check if there has been a mode change
          mode_change = (mode_ == TravelMode::kPedestrian);

          // Update trip Id and block Id
          tripid  = departure->tripid();
          blockid = departure->blockid();
          has_transit = true;

          // Add transfer cost. There is no cost if continuing along the
          // same trip Id or (valid) block Id.
          if (mode_change || tripid != pred.tripid() ||
             (blockid != 0 && blockid != pred.blockid())) {
            newcost += transfer_cost;
          }

          // Change mode and costing to transit. Add edge cost.
          mode_ = TravelMode::kPublicTransit;
          newcost += tc->EdgeCost(directededge, departure, localtime);
        } else {
          continue;  // No matching departures found for this edge
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
        if (!mode_costing[static_cast<uint32_t>(mode_)]->Allowed(
                directededge, pred)) {
          continue;
        }

        Cost c = mode_costing[static_cast<uint32_t>(mode_)]->EdgeCost(
            directededge, nodeinfo->density());
        c.cost *= 2.0f;  // TODO - mode weight...so transit mode is favored
        newcost += c;

        // Add to walking distance
        if (mode_ == TravelMode::kPedestrian) {
          walking_distance_ += directededge->length();
        }
      }

      // Add mode change cost or edge transition cost from the costing model
      if (mode_change) {
        // TODO: make mode change cost configurable. No cost for entering
        // a transit line (assume the wait time is the cost)
        ;  //newcost += {10.0f, 10.0f };
      } else {
        newcost += mode_costing[static_cast<uint32_t>(mode_)]->TransitionCost(
               directededge, nodeinfo, pred);
      }

      // If this edge is a destination, subtract the partial/remainder cost
      // (cost from the dest. location to the end of the edge)
      auto p = destinations_.find(edgeid);
      if (p != destinations_.end()) {
        newcost -= p->second;
      }

      // Get the end node, skip if the end node tile is not found
      const GraphTile* endtile = (directededge->leaves_tile()) ?
          graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (endtile == nullptr) {
        continue;
      }

      // Prohibit entering the same station as the prior. Could this be done in
      // costing?
      const NodeInfo* endnode = endtile->node(directededge->endnode());
      if (directededge->use() == Use::kTransitConnection &&
          endnode->is_transit() &&
          endnode->stop_index() == pred.prior_stopid()) {
        continue;
      }

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change). Update
      // trip Id and block Id.
      if (edgestatus.set() == EdgeSet::kTemporary) {
        uint32_t idx = edgestatus.status.index;
        float dc = edgelabels_[idx].cost().cost - newcost.cost;
        if (dc > 0) {
          float oldsortcost = edgelabels_[idx].sortcost();
          float newsortcost = oldsortcost - dc;
          edgelabels_[idx].Update(predindex, newcost, newsortcost,
                                  walking_distance_, tripid, blockid);
          adjacencylist_->DecreaseCost(idx, newsortcost, oldsortcost);
        }
        continue;
      }

      // If this is a destination edge the A* heuristic is 0. Otherwise the
      // sort cost (with A* heuristic) is found using the lat,lng at the
      // end node of the directed edge.
      float dist = 0.0f;
      float sortcost = newcost.cost;
      if (p == destinations_.end()) {
        dist = astarheuristic_.GetDistance(endnode->latlng());
        sortcost += astarheuristic_.Get(dist);
      }

      // Add edge label, add to the adjacency list and set edge status
      AddToAdjacencyList(edgeid, pred.sortcost());
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                    newcost, sortcost, dist, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_,  walking_distance_,
                    tripid, prior_stop,  blockid, has_transit);
    }
  }
  return {};      // Should never get here
}

// Check if destination can be reached if walking is the last mode. Checks
// if there are any transit stops within maximum walking distance.
// TODO - once auto/bicycle are allowed modes we need to check if parking
// or bikeshare locations are within walking distance.
bool MultiModalPathAlgorithm::CanReachDestination(const PathLocation& destination,
                          GraphReader& graphreader,
                          const TravelMode dest_mode,
                          const std::shared_ptr<DynamicCost>& costing) {
  // Assume pedestrian mode for now
  mode_ = dest_mode;

  // Use a simple Dijkstra method - no need to recover the path just need to
  // make sure we can get to a transit stop within the specified max. walking
  // distance
  uint32_t label_idx = 0;
  uint32_t bucketsize = costing->UnitSize();
  AdjacencyList adjlist(0.0f, kBucketCount * bucketsize, bucketsize);
  std::vector<EdgeLabel> edgelabels;
  EdgeStatus edgestatus;

  // Add the opposing destination edges to the priority queue
  for (const auto& edge : destination.edges()) {
    // Keep the id and the cost to traverse the partial distance
    float ratio = (1.0f - edge.dist);
    GraphId oppedge = graphreader.GetOpposingEdgeId(edge.id);
    const GraphTile* tile = graphreader.GetGraphTile(oppedge);
    const DirectedEdge* diredge = tile->directededge(oppedge);
    uint32_t length = static_cast<uint32_t>(diredge->length()) * ratio;
    Cost cost = costing->EdgeCost(diredge, 0.0f) * ratio;
    edgelabels.emplace_back(kInvalidLabel, oppedge,
            diredge, cost, cost.cost, 0.0f, 0,
            diredge->opp_local_idx(), mode_, length,
            0, 0,  0, false);
    adjlist.Add(label_idx, cost.cost);
    edgestatus.Set(oppedge, EdgeSet::kTemporary, label_idx);
    label_idx++;
  }

  // TODO - we really want to traverse in reverse direction - but since
  // pedestrian access should be the same in either direction we will
  // traverse in a forward direction for now
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjlist.Remove(edgelabels);
    if (predindex == kInvalidLabel) {
      // Throw an exception so the message is returned in the service
      throw std::runtime_error("Cannot reach destination - too far from a transit stop");
      return false;
    }

    // Remove label from adjacency list, mark it as done - copy the EdgeLabel
    // for use in costing
    EdgeLabel pred = edgelabels[predindex];
    edgestatus.Set(pred.edgeid(), EdgeSet::kPermanent, pred.edgeid());

    // Get the end node of the prior directed edge and check access
    GraphId node = pred.endnode();
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }
    const NodeInfo* nodeinfo = tile->node(node);
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    // Return true if we reach a transit stop
    if (nodeinfo->type() == NodeType::kMultiUseTransitStop) {
      return true;
    }

    // Expand edges from the node
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                i++, directededge++, edgeid++) {
      // Skip transition edges or if not allowed for htis mode
      if (directededge->trans_up() || directededge->trans_down() ||
          !costing->Allowed(directededge, pred)) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo es = edgestatus.Get(edgeid);
      if (es.set() == EdgeSet::kPermanent) {
        continue;
      }

      // Get cost
      Cost newcost = pred.cost() +
                     costing->EdgeCost(directededge, nodeinfo->density()) +
                     costing->TransitionCost(directededge, nodeinfo, pred);
      uint32_t walking_distance = pred.walking_distance() + directededge->length();

      // Check if lower cost path
      if (es.set() == EdgeSet::kTemporary) {
        uint32_t idx = es.status.index;
        float dc = edgelabels[idx].cost().cost - newcost.cost;
        if (dc > 0) {
          float oldsortcost = edgelabels[idx].sortcost();
          float newsortcost = oldsortcost - dc;
          edgelabels[idx].Update(predindex, newcost, newsortcost,
                                  walking_distance, 0, 0);
          adjlist.DecreaseCost(idx, newsortcost, oldsortcost);
        }
        continue;
      }

      // Add edge label, add to the adjacency list and set edge status
      edgelabels.emplace_back(predindex, edgeid, directededge,
                    newcost, newcost.cost, 0.0f, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_, walking_distance,
                    0, 0, 0, false);
      adjlist.Add(label_idx, newcost.cost);
      edgestatus.Set(edgeid, EdgeSet::kTemporary, label_idx);
      label_idx++;
    }
  }
  return false;
}

}
}
