#include <map>
#include <algorithm>
#include "thor/bidirectional_astar.h"
#include <valhalla/baldr/datetime.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

// Convenience method to get opposing edge Id given a directed edge and a tile
GraphId GetOpposingEdgeId(const DirectedEdge* edge, const GraphTile* tile) {
  GraphId endnode = edge->endnode();
  return { endnode.tileid(), endnode.level(),
           tile->node(endnode.id())->edge_index() + edge->opp_index() };
}

}

namespace valhalla {
namespace thor {

// Default constructor
BidirectionalAStar::BidirectionalAStar()
    : PathAlgorithm() {
}

// Destructor
BidirectionalAStar::~BidirectionalAStar() {
  Clear();
}

// Clear the temporary information generated during path construction.
void BidirectionalAStar::Clear() {
  edgelabels_.clear();
  edgelabels_reverse_.clear();
  adjacencylist_.reset();
  adjacencylist_reverse_.reset();
  edgestatus_.reset();
  edgestatus_reverse_.reset();
}

// Initialize the A* heuristic and adjacency lists for both the forward
// and reverse search.
void BidirectionalAStar::Init(const PointLL& origll, const PointLL& destll,
                              const std::shared_ptr<DynamicCost>& costing) {
  // Initialize the A* heuristics
  astarheuristic_.Init(destll, costing->AStarCostFactor());
  astarheuristic_reverse_.Init(origll, costing->AStarCostFactor());

  // Construct adjacency list, edge status, and done set
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing->UnitSize();
  float range = kBucketCount * bucketsize;
  float mincost = astarheuristic_.Get(origll);
  adjacencylist_.reset(new AdjacencyList(mincost, range, bucketsize));
  edgestatus_.reset(new EdgeStatus());

  mincost = astarheuristic_reverse_.Get(destll);
  adjacencylist_reverse_.reset(new AdjacencyList(mincost, range, bucketsize));
  edgestatus_reverse_.reset(new EdgeStatus());

  // Initialize best connection with max cost
  best_connection_ = { GraphId(), GraphId(),
		                   std::numeric_limits<float>::max() };

  // Since the search is bidirectional never enter not-thru edges (if
  // destination is within such a region the other search direction will
  // exit the not-thru region)
  costing->set_not_thru_distance(0.0f);

  // Support for hierarchy transitions
  allow_transitions_ = costing->AllowTransitions();
  hierarchy_limits_  = costing->GetHierarchyLimits();
  hierarchy_limits_reverse_ = costing->GetHierarchyLimits();
}

// Calculate best path using bi-directional A*. No hierarchies or time
// dependencies are used. Suitable for pedestrian routes (and bicycle?).
std::vector<PathInfo> BidirectionalAStar::GetBestPath(PathLocation& origin,
             PathLocation& destination, GraphReader& graphreader,
             const std::shared_ptr<sif::DynamicCost>* mode_costing,
             const sif::TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode_)];

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  Init(origin.vertex(), destination.vertex(), costing);

  // Initialize the origin and destination locations
  SetOrigin(graphreader, origin, costing);
  SetDestination(graphreader, destination, costing);

  // Update hierarchy limits (TODO - is density needed?)
  if (allow_transitions_) {
    ModifyHierarchyLimits(astarheuristic_.GetDistance(origin.vertex()), 0);
    ModifyHierarchyLimitsReverse(astarheuristic_.GetDistance(origin.vertex()), 0);
  }

  // Find shortest path. Switch between a forward direction and a reverse
  // direction search based on the current costs. Alternating like this
  // prevents one tree from expanding much more quickly (if in a sparser
  // portion of the graph) rather than strictly alternating.
  uint32_t n = 0;
  uint32_t predindex, predindex2;
  float dist = 0.0f;
  EdgeLabel pred, pred2;
  const GraphTile* tile;
  const GraphTile* tile2;
  bool expand_forward  = true;
  bool expand_reverse  = true;
  while (true) {
    // Get the next predecessor and cost (based on which direction was
    // expanded in prior step)
    if (expand_forward) {
      predindex = adjacencylist_->Remove(edgelabels_);
      if (predindex != kInvalidLabel) {
        pred = edgelabels_[predindex];
      } else {
        LOG_ERROR("Bi-directional route failure - forward search exhausted");
        return { };
      }
    }
    if (expand_reverse) {
      predindex2 = adjacencylist_reverse_->Remove(edgelabels_reverse_);
      if (predindex2 != kInvalidLabel) {
        pred2 = edgelabels_reverse_[predindex2];
      } else {
        LOG_ERROR("Bi-directional route failure - reverse search exhausted");
        return { };
      }
    }

    // Expand from the search direction with lower cost. Using sort_cost seems
    // to give inconsistent results, so use the true cost.
    if (pred.cost().cost < pred2.cost().cost) {
      // Expand forward - set to get next edge from forward adj. list
      // on the next pass
      expand_forward = true;
      expand_reverse = false;

      // Mark edge as done - copy the EdgeLabel for use in costing
      edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);

      // Get the opposing edge - if permanently labeled in reverse search
      // and this is the best connection candidate then we have the
      // shortest path. An invalid opposing edge occurs for
      // transition edges - skip them.
      GraphId oppedge = pred.opp_edgeid();
      if (oppedge.Is_Valid()) {
        EdgeStatusInfo oppedgestatus = edgestatus_reverse_->Get(oppedge);
        if (oppedgestatus.set() == EdgeSet::kPermanent &&
            ((best_connection_.edgeid == pred.edgeid() &&
              best_connection_.opp_edgeid == oppedge) ||
             (best_connection_.edgeid == oppedge &&
              best_connection_.opp_edgeid == pred.edgeid())))	{
          return FormPath(predindex, oppedgestatus.status.index, graphreader);
        }
      }

      // Get the end node of the prior directed edge. Skip if tile not found
      // (can happen with regional data sets).
      GraphId node = pred.endnode();
      if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
        continue;
      }

      // Check access at the node
      const NodeInfo* nodeinfo = tile->node(node);
      if (!costing->Allowed(nodeinfo)) {
        continue;
      }

      // Check hierarchy. Count upward transitions (counted on the level
      // transitioned from). Do not expand based on hierarchy level based on
      // number of upward transitions.
      uint32_t level = node.level();
      if (allow_transitions_) {
        if (pred.trans_up()) {
          hierarchy_limits_[level+1].up_transition_count++;
        }
        if (hierarchy_limits_[level].StopExpanding()) {
          continue;
        }
      }

      // Expand from end node in forward direction.
      uint32_t shortcuts = 0;
      GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
      const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                  i++, directededge++, edgeid++) {
        // Handle upward transition edges they either get skipped or
        // added to the adjacency list using the predecessor info
        if (directededge->trans_up()) {
          HandleTransitionEdge(level, edgeid, directededge, pred,
                 predindex, pred.distance());
          continue;
        }

        // Skip downward transition edges and any superseded edges that match
        // the shortcut mask. Also skip if no access is allowed to this edge
        // (based on costing method)
        if ( directededge->trans_down() ||
            (shortcuts & directededge->superseded()) ||
            !costing->Allowed(directededge, pred, tile, edgeid)) {
          continue;
        }

        // Get the current set. Skip this edge if permanently labeled (best
        // path already found to this directed edge).
        EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
        if (edgestatus.set() == EdgeSet::kPermanent) {
          continue;
        }

        // Update the_shortcuts mask
        shortcuts |= directededge->shortcut();

        // Get cost
        Cost newcost = pred.cost() +
                       costing->EdgeCost(directededge, nodeinfo->density()) +
                       costing->TransitionCost(directededge, nodeinfo, pred);

        // Check if edge is temporarily labeled and this path has less cost. If
        // less cost the predecessor is updated and the sort cost is decremented
        // by the difference in real cost (A* heuristic doesn't change)
        if (edgestatus.set() == EdgeSet::kTemporary) {
          CheckIfLowerCostPath(edgestatus.status.index, predindex, newcost);
          continue;
        }

        // Get end node tile (skip if tile is not found) and opposing edge Id
        const GraphTile* t2 = directededge->leaves_tile() ?
            graphreader.GetGraphTile(directededge->endnode()) : tile;
        if (t2 == nullptr) {
          continue;
        }
        GraphId oppedge = GetOpposingEdgeId(directededge, t2);

        // Find the sort cost (with A* heuristic) using the lat,lng at the
        // end node of the directed edge.
        float sortcost = newcost.cost + astarheuristic_.Get(
              t2->node(directededge->endnode())->latlng(), dist);

        // Add edge label, add to the adjacency list and set edge status
        AddToAdjacencyList(edgeid, sortcost);
        edgelabels_.emplace_back(predindex, edgeid, oppedge, directededge,
                      newcost, sortcost, dist, directededge->restrictions(),
                      directededge->opp_local_idx(), mode_, 0);

        // Check if the opposing edge is in the reverse adjacency list. If so
        // it is a candidate connection - is it least cost?
        EdgeStatusInfo oppedgestatus = edgestatus_reverse_->Get(oppedge);
        if (oppedgestatus.set() != EdgeSet::kUnreached) {
          float c = pred.cost().cost +
              edgelabels_reverse_[oppedgestatus.status.index].cost().cost;
          if (c < best_connection_.cost) {
            best_connection_ = { edgeid, oppedge, c };
          }
        }
      }
    } else {
      // Expand reverse - set to get next edge from reverse adj. list
      // on the next pass
      expand_forward = false;
      expand_reverse = true;

      // Mark edge as done - copy the EdgeLabel for use in costing
      edgestatus_reverse_->Update(pred2.edgeid(), EdgeSet::kPermanent);

      // Get the opposing edge - if permanently labeled in forward search set
      // we have the shortest path. An invalid opposing edge occurs for
      // transition edges - skip them.
      GraphId oppedge = pred2.opp_edgeid();
      if (oppedge.Is_Valid()) {
        EdgeStatusInfo oppedgestatus = edgestatus_->Get(oppedge);
        if (oppedgestatus.set() != EdgeSet::kUnreached &&
            ((best_connection_.edgeid == pred2.edgeid() &&
              best_connection_.opp_edgeid == oppedge) ||
             (best_connection_.edgeid == oppedge &&
              best_connection_.opp_edgeid == pred2.edgeid()))) {
          return FormPath(oppedgestatus.status.index, predindex2, graphreader);
        }
      }

      // Get the end node of the prior directed edge. Skip if tile not found
      // (can happen with regional data sets).
      GraphId node = pred2.endnode();
      if ((tile2 = graphreader.GetGraphTile(node)) == nullptr) {
        continue;
      }

      // Check access at the node
      const NodeInfo* nodeinfo = tile2->node(node);
      if (!costing->Allowed(nodeinfo)) {
        continue;
      }

      // Check hierarchy. Count upward transitions (counted on the level
      // transitioned from). Do not expand based on hierarchy level based on
      // number of upward transitions and distance to the destination
      uint32_t level = node.level();
      if (allow_transitions_) {
        if (pred2.trans_up()) {
          hierarchy_limits_reverse_[level+1].up_transition_count++;
        }
        if (hierarchy_limits_reverse_[level].StopExpanding()) {
          continue;
        }
      }

      // Get the opposing predecessor directed edge
      const DirectedEdge* opp_pred_edge = tile2->directededge(
              nodeinfo->edge_index() + pred2.opp_index());

      // Expand from end node in forward direction.
      uint32_t shortcuts = 0;
      GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
      const DirectedEdge* directededge = tile2->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0; i < nodeinfo->edge_count();
              i++, directededge++, edgeid++) {
        // Handle upward transition edges they either get skipped or added
        // to the adjacency list using the predecessor info
        if (directededge->trans_up()) {
          HandleTransitionEdgeReverse(level, edgeid, directededge, pred2,
                                      predindex2, pred2.distance());
          continue;
        }

        // Skip downward transitions and edges superseded by a shortcut.
        if (directededge->trans_down() ||
           (shortcuts & directededge->superseded())) {
          continue;
        }

        // Get the current set. Skip this edge if permanently labeled (best
        // path already found to this directed edge).
        EdgeStatusInfo edgestatus = edgestatus_reverse_->Get(edgeid);
        if (edgestatus.set() == EdgeSet::kPermanent) {
          continue;
        }

        // Get opposing edge Id and end node tile
        const GraphTile* t2 = directededge->leaves_tile() ?
             graphreader.GetGraphTile(directededge->endnode()) : tile2;
        if (t2 == nullptr) {
          continue;
        }
        GraphId oppedge = GetOpposingEdgeId(directededge, t2);

        // Get opposing directed edge and check if allowed. Do not enter
        // not_thru edges
        const DirectedEdge* opp_edge = t2->directededge(oppedge);
        if (directededge->not_thru() ||
            !costing->AllowedReverse(directededge, pred2, opp_edge,
                                     opp_pred_edge, tile, edgeid)) {
          continue;
        }

        // Update the_shortcuts mask
        shortcuts |= directededge->shortcut();

        // Get cost. Use opposing edge for EdgeCost.
        Cost newcost = pred2.cost() +
              costing->EdgeCost(opp_edge, nodeinfo->density());
        Cost tc = costing->TransitionCostReverse(directededge->localedgeidx(),
                            nodeinfo, opp_edge, opp_pred_edge);
        newcost.cost += tc.cost;

        // Check if edge is temporarily labeled and this path has less cost. If
        // less cost the predecessor is updated and the sort cost is decremented
        // by the difference in real cost (A* heuristic doesn't change)
        if (edgestatus.set() != EdgeSet::kUnreached) {
          CheckIfLowerCostPathReverse(edgestatus.status.index, predindex2,
                                      newcost, tc);
          continue;
        }

        // Find the sort cost (with A* heuristic) using the lat,lng at the
        // end node of the directed edge.
        float sortcost = newcost.cost + astarheuristic_reverse_.Get(
            t2->node(directededge->endnode())->latlng(), dist);

        // Add edge label, add to the adjacency list and set edge status
        AddToAdjacencyListReverse(edgeid, sortcost);
        edgelabels_reverse_.emplace_back(predindex2, edgeid, oppedge,
                      directededge, newcost, sortcost, dist,
                      directededge->restrictions(),
                      directededge->opp_local_idx(), mode_, tc.secs);

        // Check if the opposing edge is in the reverse adjacency list. If so
        //  check if it is least cost candidate.
        EdgeStatusInfo oppedgestatus = edgestatus_->Get(oppedge);
        if (oppedgestatus.set() != EdgeSet::kUnreached) {
          float c = pred.cost().cost +
              edgelabels_[oppedgestatus.status.index].cost().cost;
          if (c < best_connection_.cost) {
            best_connection_ = { edgeid, oppedge, c };
          }
        }
      }
    }
  }
  return {};    // If we are here the route failed
}

// Handle a transition edge between hierarchies.
void BidirectionalAStar::HandleTransitionEdgeReverse(const uint32_t level,
                    const GraphId& edgeid, const DirectedEdge* edge,
                    const EdgeLabel& pred, const uint32_t predindex,
                    const float dist) {
  // Skip any transition edges that are not allowed.
  if (!allow_transitions_ ||
      (edge->trans_up() &&
       !hierarchy_limits_reverse_[level].AllowUpwardTransition(dist)) ||
      (edge->trans_down() &&
       !hierarchy_limits_reverse_[level].AllowDownwardTransition(dist))) {
    return;
  }

  // Allow the transition edge. Add it to the adjacency list and edge labels
  // using the predecessor information. Transition edges have no length.
  AddToAdjacencyListReverse(edgeid, pred.sortcost());
  edgelabels_reverse_.emplace_back(predindex, edgeid, pred.opp_edgeid(),
                edge, pred.cost(), pred.sortcost(), dist,
                pred.restrictions(), pred.opp_local_idx(), mode_, 0);
}

// Modulate the hierarchy expansion within distance based on density at
// the destination (increase distance for lower densities and decrease
// for higher densities) and the distance between origin and destination
// (increase for shorter distances).
void BidirectionalAStar::ModifyHierarchyLimitsReverse(const float dist,
                                          const uint32_t density) {
  // TODO - default distance below which we increase expansion within
  // distance. This is somewhat temporary to address route quality on shorter
  // routes - hopefully we will mark the data somehow to indicate how to
  // use the hierarchy when approaching the destination (or use a
  // bi-directional search without hierarchies for shorter routes).
  float factor = 1.0f;
  if (25000.0f < dist && dist < 100000.0f) {
    factor = std::min(3.0f, 100000.0f / dist);
  }
  hierarchy_limits_reverse_[1].expansion_within_dist *= factor;
}

// Convenience method to add an edge to the reverse adjacency list and
// temporarily label it.
void BidirectionalAStar::AddToAdjacencyListReverse(const GraphId& edgeid,
                                        const float sortcost) {
  uint32_t idx = edgelabels_reverse_.size();
  adjacencylist_reverse_->Add(idx, sortcost);
  edgestatus_reverse_->Set(edgeid, EdgeSet::kTemporary, idx);
}

// Check if edge is temporarily labeled and this path has less cost. If
// less cost the predecessor is updated and the sort cost is decremented
// by the difference in real cost (A* heuristic doesn't change). This is
// done for the reverse search.
void BidirectionalAStar::CheckIfLowerCostPathReverse(const uint32_t idx,
                                         const uint32_t predindex,
                                         const Cost& newcost,
                                         const Cost& tc) {
  float dc = edgelabels_reverse_[idx].cost().cost - newcost.cost;
  if (dc > 0) {
    float oldsortcost = edgelabels_reverse_[idx].sortcost();
    float newsortcost = oldsortcost - dc;
    edgelabels_reverse_[idx].Update(predindex, newcost, newsortcost);
    edgelabels_reverse_[idx].set_transition_cost(tc.secs);
    adjacencylist_reverse_->DecreaseCost(idx, newsortcost, oldsortcost);
  }
}

// Add edges at the origin to the forward adjacency list.
void BidirectionalAStar::SetOrigin(GraphReader& graphreader,
                 PathLocation& origin,
                 const std::shared_ptr<DynamicCost>& costing) {
  // Iterate through edges and add to adjacency list
  const NodeInfo* nodeinfo = nullptr;
  for (const auto& edge : origin.edges()) {
    // If origin is at a node - skip any inbound edge (dist = 1)
    if (origin.IsNode() && edge.dist == 1) {
      continue;
    }

    // Get the directed edge
    GraphId edgeid = edge.id;
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the tile at the end node. Skip if tile not found as we won't be
    // able to expand from this origin edge.
    const GraphTile* endtile = graphreader.GetGraphTile(directededge->endnode());
    if (endtile == nullptr) {
      continue;
    }

    // Get cost and sort cost (based on distance from endnode of this edge
    // to the destination
    nodeinfo = endtile->node(directededge->endnode());
    Cost cost = costing->EdgeCost(directededge,
                   graphreader.GetEdgeDensity(edgeid)) * (1.0f - edge.dist);
    float dist = astarheuristic_.GetDistance(nodeinfo->latlng());
    float sortcost = cost.cost + astarheuristic_.Get(dist);

    // Add EdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path.
    AddToAdjacencyList(edgeid, sortcost);
    edgelabels_.emplace_back(kInvalidLabel, edgeid, directededge, cost,
            sortcost, dist, directededge->restrictions(),
            directededge->opp_local_idx(), mode_);
  }

  // Set the origin timezone
  if (nodeinfo != nullptr && origin.date_time_ &&
      *origin.date_time_ == "current") {
    origin.date_time_= DateTime::iso_date_time(
    		DateTime::get_tz_db().from_index(nodeinfo->timezone()));
  }
}

// Add destination edges to the reverse path adjacency list.
void BidirectionalAStar::SetDestination(GraphReader& graphreader,
                     const PathLocation& dest,
                     const std::shared_ptr<DynamicCost>& costing) {
  // Iterate through edges and add to adjacency list
  for (const auto& edge : dest.edges()) {
    // If the destination is at a node, skip any outbound edges (so any
    // opposing inbound edges are not considered)
    if (dest.IsNode() && edge.dist == 0.0f) {
      continue;
    }

    // Get the directed edge
    GraphId edgeid = edge.id;
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the opposing directed edge, continue if we cannot get it
    GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid);
    if (!opp_edge_id.Is_Valid()) {
      continue;
    }
    const DirectedEdge* opp_dir_edge = graphreader.GetOpposingEdge(edgeid);

    // Get the tile at the end node. Skip if tile not found as we won't be
    // able to expand from this origin edge.
    const GraphTile* endtile = graphreader.GetGraphTile(directededge->endnode());
    if (endtile == nullptr) {
      continue;
    }

    // Get cost and sort cost (based on distance from endnode of this edge
    // to the origin
    Cost cost = costing->EdgeCost(opp_dir_edge,
                    graphreader.GetEdgeDensity(opp_edge_id)) * edge.dist;
    float dist = astarheuristic_.GetDistance(endtile->node(
                      directededge->endnode())->latlng());
    float sortcost = cost.cost + astarheuristic_reverse_.Get(dist);

    // Add EdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path.
    AddToAdjacencyListReverse(opp_edge_id, sortcost);
    edgelabels_reverse_.emplace_back(kInvalidLabel, opp_edge_id,
             opp_dir_edge, cost, sortcost, dist, opp_dir_edge->restrictions(),
             opp_dir_edge->opp_local_idx(), mode_);
  }
}

// Form the path from the adjacency list.
std::vector<PathInfo> BidirectionalAStar::FormPath(const uint32_t idx1,
              const uint32_t idx2, GraphReader& graphreader) {
  // Metrics (TODO)
  LOG_INFO("FormPath path_iterations::" + std::to_string(edgelabels_.size()) +
           "," + std::to_string(edgelabels_reverse_.size()));

  // Work backwards on the forward path
  std::vector<PathInfo> path;
  for (auto edgelabel_index = idx1; edgelabel_index != kInvalidLabel;
      edgelabel_index = edgelabels_[edgelabel_index].predecessor()) {
    const EdgeLabel& edgelabel = edgelabels_[edgelabel_index];
    path.emplace_back(edgelabel.mode(), edgelabel.cost().secs,
                      edgelabel.edgeid(), edgelabel.tripid());
  }

  // Reverse the list
  std:reverse(path.begin(), path.end());

  // Special case code if the last edge of the forward path is
  // the destination edge - update the elapsed time
  if (edgelabels_reverse_[idx2].predecessor() == kInvalidLabel) {
    if (path.size() > 1) {
      path.back().elapsed_time = path[path.size()-2].elapsed_time +
          edgelabels_reverse_[idx2].cost().secs;
    } else {
      path.back().elapsed_time = edgelabels_reverse_[idx2].cost().secs;
    }
    return path;
  }

  // Get the elapsed time at the end of the forward path. NOTE: PathInfo
  // stores elapsed time as uint32_t but EdgeLabels uses float. Need to
  // accumulate in float and cast to int so we do not accumulate roundoff
  // error.
  float secs = path.back().elapsed_time;

  // Get the transition cost at the last edge of the reverse path
  float tc = edgelabels_reverse_[idx2].transition_cost();

  // Append the reverse path from the destination - use opposing edges
  // The first edge on the reverse path is the same as the last on the forward
  // path, so get the predecessor.
  uint32_t edgelabel_index = edgelabels_reverse_[idx2].predecessor();
  while (edgelabel_index != kInvalidLabel) {
    const EdgeLabel& edgelabel = edgelabels_reverse_[edgelabel_index];
    GraphId oppedge = graphreader.GetOpposingEdgeId(edgelabel.edgeid());

    // Get elapsed time on the edge
    uint32_t pred = edgelabels_reverse_[edgelabel_index].predecessor();
    if (pred == kInvalidLabel) {
      secs += edgelabel.cost().secs;
    } else {
      secs += edgelabel.cost().secs - edgelabels_reverse_[pred].cost().secs;
    }
    secs += tc;
    path.emplace_back(edgelabel.mode(), static_cast<uint32_t>(secs),
                            oppedge,  edgelabel.tripid());

    // Update edgelabel_index
    edgelabel_index = pred;
  }
  return path;
}

}
}
