#include <iostream> // TODO remove if not needed
#include <map>
#include <algorithm>
#include "thor/pathalgorithm.h"
#include <valhalla/baldr/datetime.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;

// TODO: make a class that extends std::exception, with messages and
// error codes and return the appropriate error codes

namespace {

constexpr uint32_t kBucketCount = 20000;
constexpr uint64_t kInitialEdgeLabelCount = 500000;

// If the destination is at a node we want the incoming edge Ids
// with distance = 1.0 (the full edge). This returns and updated
// destination PathLocation.
// TODO - move this logic into Loki
// TODO - fail the route if no dest edges
PathLocation update_destinations(GraphReader& graphreader,
                                 const PathLocation& destination,
                                 const EdgeFilter& filter) {
  if (destination.IsNode()) {
    // Copy the current destination info and clear the edges
    PathLocation dest = destination;
    dest.ClearEdges();

    // Get the node. Iterate through the edges and get opposing edges. Add
    // to the destination edges if it is allowed by the costing model
    GraphId destedge = destination.edges()[0].id;
    GraphId opposing_edge = graphreader.GetOpposingEdgeId(destedge);
    GraphId endnode = graphreader.GetGraphTile(opposing_edge)->directededge(opposing_edge)->endnode();
    const GraphTile* tile = graphreader.GetGraphTile(endnode);
    const NodeInfo* nodeinfo = tile->node(endnode);
    GraphId edgeid(endnode.tileid(), endnode.level(), nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, edgeid++) {
      GraphId opposing_edge = graphreader.GetOpposingEdgeId(edgeid);
      tile = graphreader.GetGraphTile(opposing_edge);
      const DirectedEdge* edge = tile->directededge(opposing_edge);
      if (!filter(edge)) {
        dest.CorrelateEdge(PathLocation::PathEdge{opposing_edge, 1.0f});
      }
    }
    return dest;
  } else {
    // No need to alter destination edges
    return destination;
  }
}

GraphId trivial(const PathLocation& origin, const PathLocation& destination) {
  //check if any of the pairs of origin and destination edges could be a trivial path
  //NOTE: it is true that there could be a shorter path by leaving this edge and coming
  //back in the other direction however this should be uncommon
  for(const auto& origin_edge : origin.edges()) {
    for(const auto& destination_edge : destination.edges()) {
      //same id and the origin shows up at the beginning of the edge
      //while the destination shows up at the end of the edge
      if (origin_edge.id == destination_edge.id &&
          origin_edge.dist <= destination_edge.dist) {
        return origin_edge.id;
      }
    }
  }
  return {};
}

GraphId loop(const PathLocation& origin, const PathLocation& destination) {
  //if we end up with locations where there is a trivial path but you would have to
  //traverse the edge in reverse to do it we need to mark this as a loop so that
  //the initial edge can be considered on the path at some later point in time
  for(const auto& origin_edge : origin.edges()) {
    for(const auto& destination_edge : destination.edges()) {
      //same id and the origin shows up at the end of the edge
      //while the destination shows up at the beginning of the edge
      if(origin_edge.id == destination_edge.id && origin_edge.dist > destination_edge.dist) {
        //something is wrong if we have more than one option here
        if(origin.edges().size() != 1 || destination.edges().size() != 1)
          throw std::runtime_error("Found oneway loop but multiple ins and outs!");
        return origin_edge.id;
      }
    }
  }
  return {};
}

}

namespace valhalla {
namespace thor {

// Default constructor
PathAlgorithm::PathAlgorithm()
    : allow_transitions_(false),
      edgelabel_index_(0),
      adjacencylist_(nullptr),
      edgestatus_(nullptr),
      best_destination_{kInvalidLabel, Cost(std::numeric_limits<float>::max(), 0.0f)} {
  edgelabels_.reserve(kInitialEdgeLabelCount);
}

// Destructor
PathAlgorithm::~PathAlgorithm() {
  Clear();
}

// Clear the temporary information generated during path construction.
void PathAlgorithm::Clear() {
  // Set the edge label index back to 0
  edgelabel_index_ = 0;
  edgelabels_.clear();
  best_destination_ = std::make_pair(kInvalidLabel,
                         Cost(std::numeric_limits<float>::max(), 0.0f));
  destinations_.clear();

  // Clear elements from the adjacency list
  if(adjacencylist_ != nullptr) {
    adjacencylist_->Clear();
    adjacencylist_ = nullptr;
  }

  // Clear the edge status flags
  if (edgestatus_ != nullptr) {
    delete edgestatus_;
    edgestatus_ = nullptr;
  }
}

// Initialize prior to finding best path
void PathAlgorithm::Init(const PointLL& origll, const PointLL& destll,
    const std::shared_ptr<DynamicCost>& costing, const bool multimodal) {
  LOG_TRACE("Orig LL = " + std::to_string(origll.lat()) + "," + std::to_string(origll.lng()));
  LOG_TRACE("Dest LL = " + std::to_string(destll.lat()) + "," + std::to_string(destll.lng()));

  float mincost = 0.0f;
  if (multimodal) {
    // Disable A* for multimodal
    astarheuristic_.Init(destll, 0.0f);
  } else {
    // Set the destination and cost factor in the A* heuristic
    astarheuristic_.Init(destll, costing->AStarCostFactor());

    // Get the initial cost based on A* heuristic from origin
    mincost = astarheuristic_.Get(origll);
  }

  // Construct adjacency list, edge status, and done set
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_ = new AdjacencyList(mincost, range, bucketsize);
  edgestatus_ = new EdgeStatus();

  // Get hierarchy limits from the costing. Get a copy since we increment
  // transition counts (i.e., this is not a const reference).
  allow_transitions_ = costing->AllowTransitions();
  hierarchy_limits_  = costing->GetHierarchyLimits();
}

// Calculate best path.
std::vector<PathInfo> PathAlgorithm::GetBestPath(const PathLocation& origin,
             const PathLocation& destination, GraphReader& graphreader,
             const std::shared_ptr<DynamicCost>& costing) {
  // Alter the destination edges if at a node - loki always gives edges
  // leaving a node, but when a destination we want edges entering the node
  PathLocation dest = update_destinations(graphreader, destination,
                                          costing->GetFilter());

  // Check for trivial path
  // TODO -currently mode is the same along entire path.
  mode_ = costing->travelmode();
  auto trivial_id = trivial(origin, dest);
  if (trivial_id.Is_Valid()) {
    std::vector<PathInfo> trivialpath;
    trivialpath.emplace_back(mode_, 0, trivial_id, 0);
    return trivialpath;
  }

  // Check for loop path
  PathInfo loop_edge_info(mode_, 0.0f, loop(origin, dest), 0);

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  Init(origin.vertex(), dest.vertex(), costing, false);
  float mindist = astarheuristic_.GetDistance(origin.vertex());

  // Initialize the origin and destination locations
  SetOrigin(graphreader, origin, costing, loop_edge_info);
  SetDestination(graphreader, dest, costing);

  // Find shortest path
  uint32_t nc = 0;       // Count of iterations with no convergence
                         // towards destination
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->Remove(edgelabels_);
    if (predindex == kInvalidLabel) {
      // If we had a destination but we were waiting on other possible ones
      if(best_destination_.first != kInvalidLabel)
        return FormPath(best_destination_.first, graphreader, loop_edge_info);

      // We didn't find any destination edge - return empty list of edges
      LOG_ERROR("Route failed after iterations = " +
                   std::to_string(edgelabel_index_));
 //     throw std::runtime_error("No path could be found for input");
      return { };
    }

    // Remove label from adjacency list, mark it as done - copy the EdgeLabel
    // for use in costing
    EdgeLabel pred = edgelabels_[predindex];
    edgestatus_->Set(pred.edgeid(), kPermanent, pred.edgeid());

    // Check for completion. Form path and return if complete.
    if (IsComplete(predindex)) {
      return FormPath(best_destination_.first, graphreader, loop_edge_info);
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

    // Get the end node of the prior directed edge
    GraphId node   = pred.endnode();
    uint32_t level = node.level();

    // Check hierarchy. Count upward transitions (counted on the level
    // transitioned from). Do not expand based on hierarchy level based on
    // number of upward transitions and distance to the destination
    if (pred.trans_up()) {
      hierarchy_limits_[level+1].up_transition_count++;
    }
    if (hierarchy_limits_[level].StopExpanding(dist2dest)) {
      continue;
    }

    // Skip if tile not found (can happen with regional data sets).
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Check access at the node
    const NodeInfo* nodeinfo = tile->node(node);
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    // Expand from end node.
    uint32_t shortcuts = 0;
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                i++, directededge++, edgeid++) {
      // Handle transition edges they either get skipped or added to the
      // adjacency list using the predecessor info
      if (directededge->trans_up() || directededge->trans_down()) {
          HandleTransitionEdge(level, edgeid, directededge, pred,  predindex);
        continue;
      }

      // Skip shortcut edges when near the destination.
      // TODO - do not think this is needed - moved this out of autocost.
      // If needed should base it on a hierarchy limit...
      if (directededge->is_shortcut() && dist2dest < 10000.0f)
        continue;

      // Skip any superseded edges that match the shortcut mask. Also skip
      // if no access is allowed to this edge (based on costing method)
      if ((shortcuts & directededge->superseded()) ||
          !costing->Allowed(directededge, pred)) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.status.set == kPermanent) {
        continue;
      }

      // Update the_shortcuts mask
      shortcuts |= directededge->shortcut();

      // Get cost
      Cost newcost = pred.cost() +
                     costing->EdgeCost(directededge, nodeinfo->density()) +
                     costing->TransitionCost(directededge, nodeinfo, pred);

      // Update walking distance
      walking_distance_ = (mode_ == TravelMode::kPedestrian) ?
                    pred.walking_distance() + directededge->length() : 0;

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change)
      if (edgestatus.status.set == kTemporary) {
        CheckIfLowerCostPath(edgestatus.status.index, predindex, newcost);
        continue;
      }

      // Find the sort cost (with A* heuristic) using the lat,lng at the
      // end node of the directed edge. Skip if tile not found.
      if ((tile = graphreader.GetGraphTile(directededge->endnode())) == nullptr) {
        continue;
      }
      float dist = astarheuristic_.GetDistance(tile->node(
                directededge->endnode())->latlng());
      float sortcost = newcost.cost + astarheuristic_.Get(dist);

      // Add edge label, add to the adjacency list and set edge status
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                    newcost, sortcost, dist, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_, walking_distance_);
      adjacencylist_->Add(edgelabel_index_, sortcost);
      edgestatus_->Set(edgeid, kTemporary, edgelabel_index_);
      edgelabel_index_++;
    }
  }
  return {};      // Should never get here
}

// Calculate best path.
std::vector<PathInfo> PathAlgorithm::GetBestPathMM(const PathLocation& origin,
             const PathLocation& destination, GraphReader& graphreader,
             const std::shared_ptr<DynamicCost>* mode_costing) {
  // TODO - some means of setting an initial mode and probably a dest/end mode
   mode_ = TravelMode::kPedestrian;
   const auto& costing = mode_costing[static_cast<uint32_t>(mode_)];
   const auto& tc = mode_costing[static_cast<uint32_t>(TravelMode::kPublicTransit)];

   // For pedestrian - set flag allowing use of transit connections
   costing->SetAllowTransitConnections(true);

  // Alter the destination edges if at a node - loki always gives edges
  // leaving a node, but when a destination we want edges entering the node
  PathLocation dest = update_destinations(graphreader, destination,
                                          costing->GetFilter());

  // Check for trivial path
  auto trivial_id = trivial(origin, dest);
  if (trivial_id.Is_Valid()) {
    std::vector<PathInfo> trivialpath;
    trivialpath.emplace_back(mode_, 0, trivial_id, 0);
    return trivialpath;
  }

  // For now the date_time must be set on the origin.
  if (!origin.date_time_)
    return { };

  // Set route start time (seconds from midnight), date, and day of week
  uint32_t start_time = DateTime::seconds_from_midnight(*origin.date_time_);
  uint32_t localtime = start_time;
  uint32_t date = DateTime::days_from_pivot_date(*origin.date_time_);
  uint32_t dow  = DateTime::day_of_week_mask(*origin.date_time_);

  // Check for loop path
  PathInfo loop_edge_info(mode_, 0.0f, loop(origin, dest), 0);

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  Init(origin.vertex(), dest.vertex(), costing, true);
  float mindist = astarheuristic_.GetDistance(origin.vertex());

  // Initialize the origin and destination locations
  SetOrigin(graphreader, origin, costing, loop_edge_info);
  SetDestination(graphreader, dest, costing);

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
      // If we had a destination but we were waiting on other possible ones
      if(best_destination_.first != kInvalidLabel)
        return FormPath(best_destination_.first, graphreader, loop_edge_info);

      // We didn't find any destination edge - return empty list of edges
      LOG_ERROR("Route failed after iterations = " +
                   std::to_string(edgelabel_index_));
 //     throw std::runtime_error("No path could be found for input");
      return { };
    }

    // Remove label from adjacency list, mark it as done - copy the EdgeLabel
    // for use in costing
    EdgeLabel pred = edgelabels_[predindex];
    edgestatus_->Set(pred.edgeid(), kPermanent, pred.edgeid());

    // Check for completion. Form path and return if complete.
    if (IsComplete(predindex)) {
      return FormPath(best_destination_.first, graphreader, loop_edge_info);
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

    // Get the end node and travel mode of the prior directed edge
    GraphId node = pred.endnode();
    mode_ = pred.mode();

    // Skip if tile not found (can happen with regional data sets).
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Set local time. TODO: adjust for time zone
    uint32_t localtime = start_time + pred.cost().secs;

    // Check access at the node
    const NodeInfo* nodeinfo = tile->node(node);
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    // Get any transfer times and penalties if this is a transit stop
    // and mode is pedestrian
    uint32_t prior_stop = pred.prior_stopid();
    Cost transfer_cost = { 0.0f, 0.0f };
    if (nodeinfo->type() == NodeType::kMultiUseTransitStop) {
      if (mode_ == TravelMode::kPedestrian) {
        transfer_cost = tc->TransferCost(tile->GetTransfer(prior_stop,
                                      nodeinfo->stop_id()));
      }

      // Update prior stop
      prior_stop = nodeinfo->stop_id();
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

    // Set the accumulated walking distance
    walking_distance_ = pred.walking_distance();

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
      if (edgestatus.status.set == kPermanent) {
        continue;
      }

      // Reset cost
      Cost newcost = pred.cost();

      // If this is a transit edge - get the next departure. Do not check
      // if allowed by costing - assume if you get a transit edge you
      // walked to the transit stop
      tripid = 0;
      if (directededge->IsTransitLine()) {
        const TransitDeparture* departure = tile->GetNextDeparture(
                    directededge->lineid(), localtime, date, dow);
        if (departure) {
          // Check if there has been a mode change
          mode_change = (mode_ == TravelMode::kPedestrian);

          // Update trip Id and block Id
          tripid  = departure->tripid();
          blockid = departure->blockid();

          // There is no cost if continuing along the same trip Id
          // or (valid) block Id. Make sure we did not walk from stop to stop.
          if (prior_stop == 0) {
            ;
          } else if (!mode_change && (tripid == pred.tripid() ||
              (blockid != 0 && (blockid == pred.blockid())))) {
            ; // LOG_INFO("Stay on trip Id = " + std::to_string(tripid));
          } else {
            // Use the transfer cost computed for the stop
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
        newcost += mode_costing[static_cast<uint32_t>(mode_)]->EdgeCost(
            directededge, nodeinfo->density());
      }

      if (mode_change) {
        // TODO: make mode change cost configurable. No cost for entering
        // a transit line (assume the wait time is the cost)
        ;  //newcost += {10.0f, 10.0f };
      } else {
        // Use the transition costs from the costing model
        newcost += mode_costing[static_cast<uint32_t>(mode_)]->TransitionCost(
               directededge, nodeinfo, pred);
      }

      // Add to walking distance
      if (mode_ == TravelMode::kPedestrian) {
        walking_distance_ += directededge->length();
      }

      // Skip if the end node tile is not found
      const GraphTile* endtile;
      if ((endtile = graphreader.GetGraphTile(directededge->endnode())) == nullptr) {
        continue;
      }

      // Prohibit entering the same station as the prior. Could this be done in
      // costing?
      const NodeInfo* endnode = endtile->node(directededge->endnode());
      if (directededge->use() == Use::kTransitConnection &&
          endnode->is_transit() &&
          endnode->stop_id() == pred.prior_stopid()) {
        continue;
      }

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change)
      if (edgestatus.status.set == kTemporary) {
        CheckIfLowerCostPath(edgestatus.status.index, predindex, newcost);
        continue;
      }

      // Distance and sort cost
      float dist = astarheuristic_.GetDistance(endnode->latlng());
      float sortcost = newcost.cost + astarheuristic_.Get(dist);

      // Add edge label, add to the adjacency list and set edge status
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                    newcost, sortcost, dist, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_,  walking_distance_,
                    tripid, prior_stop,  blockid);
      adjacencylist_->Add(edgelabel_index_, sortcost);
      edgestatus_->Set(edgeid, kTemporary, edgelabel_index_);
      edgelabel_index_++;
    }
  }
  return {};      // Should never get here
}

// Check if edge is temporarily labeled and this path has less cost. If
// less cost the predecessor is updated and the sort cost is decremented
// by the difference in real cost (A* heuristic doesn't change)
void PathAlgorithm::CheckIfLowerCostPath(const uint32_t idx,
                                         const uint32_t predindex,
                                         const Cost& newcost) {
  float dc = edgelabels_[idx].cost().cost - newcost.cost;
  if (dc > 0) {
    float oldsortcost = edgelabels_[idx].sortcost();
    float newsortcost = oldsortcost - dc;
    edgelabels_[idx].Update(predindex, newcost, newsortcost,
                            walking_distance_);
    adjacencylist_->DecreaseCost(idx, newsortcost, oldsortcost);
  }
}

// Handle a transition edge between hierarchies.
void PathAlgorithm::HandleTransitionEdge(const uint32_t level,
                    const GraphId& edgeid, const DirectedEdge* edge,
                    const EdgeLabel& pred, const uint32_t predindex) {
  // Skip any transition edges that are not allowed.
  if (!allow_transitions_ ||
      (edge->trans_up() &&
       !hierarchy_limits_[level].AllowUpwardTransition(pred.distance())) ||
      (edge->trans_down() &&
       !hierarchy_limits_[level].AllowDownwardTransition(pred.distance()))) {
    return;
  }

  // Allow the transition edge. Add it to the adjacency list using the
  // predecessor information. Transition edges have no length.
  edgelabels_.emplace_back(predindex, edgeid,
                edge, pred.cost(), pred.sortcost(), pred.distance(),
                pred.restrictions(), pred.opp_local_idx(), mode_, 0);

  // Add to the adjacency list and set edge status
  adjacencylist_->Add(edgelabel_index_, pred.sortcost());
  edgestatus_->Set(edgeid, kTemporary, edgelabel_index_);
  edgelabel_index_++;
}

// Add an edge at the origin to the adjacency list
void PathAlgorithm::SetOrigin(GraphReader& graphreader,
                 const PathLocation& origin,
                 const std::shared_ptr<DynamicCost>& costing,
                 const PathInfo& loop_edge_info) {
  // Get sort heuristic based on distance from origin to destination
  float dist = astarheuristic_.GetDistance(origin.vertex());
  float heuristic = astarheuristic_.Get(dist);

  //we need to do some additional bookkeeping if this path needs to be a loop
  GraphId loop_edge_id = loop_edge_info.edgeid;
  std::vector<baldr::PathLocation::PathEdge> loop_edges;
  Cost loop_edge_cost {0.0f, 0.0f};
  if (loop_edge_id.Is_Valid()) {
    //grab some info about the edge and whats connected to the end of it
    const auto node_id = graphreader.GetGraphTile(loop_edge_id)->directededge(loop_edge_id)->endnode();
    const auto tile = graphreader.GetGraphTile(node_id);
    const auto node_info = tile->node(node_id);
    loop_edge_cost = costing->EdgeCost(tile->directededge(loop_edge_id), node_info->density()) *
                        (1.f - origin.edges().front().dist);
    //keep information about all the edges leaving the end of this edge
    for(uint32_t edge_index = node_info->edge_index(); edge_index < node_info->edge_index() + node_info->edge_count(); ++edge_index) {
      if(!costing->GetFilter()(tile->directededge(edge_index))) {
        loop_edges.emplace_back(node_id, 0.f);
        loop_edges.back().id.fields.id = edge_index;
      }
    }
  }

  // Iterate through edges and add to adjacency list
  for (const auto& edge : (loop_edges.size() ? loop_edges : origin.edges())) {
    // Get the directed edge
    GraphId edgeid = edge.id;
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get cost and sort cost
    Cost cost = (costing->EdgeCost(directededge, 0) * (1.0f - edge.dist)) + loop_edge_cost;
    float sortcost = cost.cost + heuristic;

    // Add EdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path.
    edgelabels_.emplace_back(kInvalidLabel, edgeid,
            directededge, cost, sortcost, dist, 0,
            directededge->opp_local_idx(), mode_, 0);
    adjacencylist_->Add(edgelabel_index_, sortcost);
    edgestatus_->Set(edgeid, kTemporary, edgelabel_index_);
    edgelabel_index_++;
  }
}

// Add a destination edge
void PathAlgorithm::SetDestination(GraphReader& graphreader,
                     const PathLocation& dest,
                     const std::shared_ptr<DynamicCost>& costing) {
  // For each edge
  float seconds = 0.0f;
  for (const auto& edge : dest.edges()) {
    // Keep the id and the cost to traverse the partial distance
    const GraphTile* tile = graphreader.GetGraphTile(edge.id);
    destinations_[edge.id] = (costing->EdgeCost(tile->directededge(edge.id), 0.0f) * edge.dist);
  }
}

// Test is the shortest path has been found.
bool PathAlgorithm::IsComplete(const uint32_t edge_label_index) {
  //grab the label
  const EdgeLabel& edge_label = edgelabels_[edge_label_index];

  //if we've already found a destination and the search's current edge is more costly to get to, we are done
  if(best_destination_.first != kInvalidLabel && edge_label.cost() > best_destination_.second)
    return true;

  //check if its a destination
  auto p = destinations_.find(edge_label.edgeid());
  //it is indeed one of the possible destination edges
  if(p != destinations_.end()) {
    //if we didnt have another destination yet or this one is better
    auto cost = edge_label.cost() + p->second;
    if(best_destination_.first == kInvalidLabel || cost < best_destination_.second){
      best_destination_.first = edge_label_index;
      best_destination_.second = cost;
    }
    destinations_.erase(p);
    //if we've found all of the destinations we are done looking
    return destinations_.size() == 0;
  }
  return false;
}

// Form the path from the adjacency list.
std::vector<PathInfo> PathAlgorithm::FormPath(const uint32_t dest,
             GraphReader& graphreader, const PathInfo& loop_edge_info) {
  // TODO - leave in for now!
  LOG_INFO("PathCost = " + std::to_string(edgelabels_[dest].cost().cost) +
           "  Iterations = " + std::to_string(edgelabel_index_));

  // Work backwards from the destination
  std::vector<PathInfo> path;
  path.reserve(edgelabels_.size());
  for(auto edgelabel_index = dest; edgelabel_index != kInvalidLabel;
      edgelabel_index = edgelabels_[edgelabel_index].predecessor()) {
    const EdgeLabel& edgelabel = edgelabels_[edgelabel_index];
    path.emplace_back(edgelabel.mode(), edgelabel.cost().secs,
                      edgelabel.edgeid(), edgelabel.tripid());
  }

  // We had a loop which means we end on the same edge we began
  // this special case can only be handled by adding back the start
  // edge at the end of the path finding because we need to encounter
  // the same edge twice (loop) and the algorithm doesn't allow for this
  if (loop_edge_info.edgeid.Is_Valid()) {
    // Loop edge uses the mode of the last edge found above.
    // TODO - what is the elapsed time on the loop edge?
    path.emplace_back(loop_edge_info);
  }

  // Reverse the list and return
  std:reverse(path.begin(), path.end());
  return path;
}

}
}
