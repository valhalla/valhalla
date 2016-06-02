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

// Find a threshold to continue the search - should be based on
// the max edge cost in the adjacency set?
int GetThreshold(const TravelMode mode, const int n) {
  return (mode == TravelMode::kDrive) ?
          std::min(2700, std::max(100, n / 3)) : 500;
}

}

namespace valhalla {
namespace thor {

constexpr uint64_t kInitialEdgeLabelCountBD = 100000;

// Default constructor
BidirectionalAStar::BidirectionalAStar() {
  threshold_ = 0;
  mode_ = TravelMode::kDrive;
  allow_transitions_ = false;
  adjacencylist_ = nullptr;
  edgestatus_ = nullptr;
  tile_creation_date_ = 0;
  edgelabels_.reserve(kInitialEdgeLabelCountBD);
  edgelabels_reverse_.reserve(kInitialEdgeLabelCountBD);
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
  float factor = costing->AStarCostFactor();
  astarheuristic_.Init(destll, factor);
  astarheuristic_reverse_.Init(origll, factor);

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

  // Set the threshold to 0 (used to extend search once an initial
  // connection has been found).
  threshold_ = 0;

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
  // Set origin and destination locations - seeds the adj. lists
  Init(origin.vertex(), destination.vertex(), costing);
  SetOrigin(graphreader, origin, costing);
  SetDestination(graphreader, destination, costing);

  // Find shortest path. Switch between a forward direction and a reverse
  // direction search based on the current costs. Alternating like this
  // prevents one tree from expanding much more quickly (if in a sparser
  // portion of the graph) rather than strictly alternating.
  int n = 0;
  uint32_t forward_pred_idx, reverse_pred_idx;
  float dist = 0.0f;
  EdgeLabel pred, pred2;
  Cost empty_cost;
  const GraphTile* tile;
  const GraphTile* tile2;
  bool expand_forward  = true;
  bool expand_reverse  = true;
  while (true) {
    // Get the next predecessor (based on which direction was
    // expanded in prior step)
    if (expand_forward) {
      forward_pred_idx = adjacencylist_->Remove(edgelabels_);
      if (forward_pred_idx != kInvalidLabel) {
        // Check if the edge on the forward search connects to a
        // reached edge on the reverse search tree.
        pred = edgelabels_[forward_pred_idx];
        if (!pred.trans_up()) {
          CheckForwardConnection(pred);
        }
      } else {
        if (best_connection_.cost < std::numeric_limits<float>::max()) {
          return FormPath(graphreader);
        } else {
          LOG_ERROR("Bi-directional route failure - forward search exhausted: n = " +
                  std::to_string(edgelabels_.size()) + "," +
                  std::to_string(edgelabels_reverse_.size()));
          return { };
        }
      }
    }
    if (expand_reverse) {
      reverse_pred_idx = adjacencylist_reverse_->Remove(edgelabels_reverse_);
      if (reverse_pred_idx != kInvalidLabel) {
        // Check if the edge on the reverse search connects to a
        // reached edge on the forward search tree.
        pred2 = edgelabels_reverse_[reverse_pred_idx];
        if (!pred2.trans_up()) {
          CheckReverseConnection(pred2);
        }
      } else {
        if (best_connection_.cost < std::numeric_limits<float>::max()) {
          return FormPath(graphreader);
        } else {
          LOG_ERROR("Bi-directional route failure - reverse search exhausted: n = " +
                  std::to_string(edgelabels_reverse_.size()) + "," +
                  std::to_string(edgelabels_.size()));
          return { };
        }
      }
    }

    // Terminate some number of iterations after an initial connection
    // has been found. This is not ideal, probably needs to be based on
    // the max edge cost but that has performance limitations,
    // so for now we use this bit of a hack...stay tuned.
    if (best_connection_.cost < std::numeric_limits<float>::max()) {
      if (n++ > threshold_) {
        return FormPath(graphreader);
      }
    }

    // Expand from the search direction with lower sort cost.
    if (pred.sortcost() < pred2.sortcost()) {
      // Expand forward - set to get next edge from forward adj. list
      // on the next pass
      expand_forward = true;
      expand_reverse = false;

      // Settle this edge.
      edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);

      // Prune path if predecessor is not a through edge
      if (pred.not_thru()) {
        continue;
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
        if (hierarchy_limits_[level].StopExpanding(pred.distance())) {
          continue;
        }
      }

      // Expand from end node in forward direction.
      uint32_t shortcuts = 0;
      GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
      const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0; i < nodeinfo->edge_count();
                  i++, directededge++, edgeid++) {
        // Handle upward transition edges
        if (directededge->trans_up()) {
           if (allow_transitions_ &&
               hierarchy_limits_[level].AllowUpwardTransition(dist)) {
             // Allow the transition edge. Add it to the adjacency list and
             // edge labels using the predecessor information. Transition
             // edges have no length.
             AddToAdjacencyList(edgeid, pred.sortcost());
             edgelabels_.emplace_back(forward_pred_idx, edgeid,
                       pred.opp_edgeid(), directededge, pred.cost(),
                       pred.sortcost(), pred.distance(), pred.restrictions(),
                       pred.opp_local_idx(), mode_,
                       Cost(pred.transition_cost(), pred.transition_secs()));
          }
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

        // Get the current set. Skip this edge if permanently labeled.
        EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
        if (edgestatus.set() == EdgeSet::kPermanent) {
          continue;
        }

        // Get cost. Separate out transition cost. Update the_shortcuts mask.
        shortcuts |= directededge->shortcut();
        Cost tc = costing->TransitionCost(directededge, nodeinfo, pred);
        Cost newcost = pred.cost() + tc +
                      costing->EdgeCost(directededge, nodeinfo->density());

        // Check if edge is temporarily labeled and this path has less cost. If
        // less cost the predecessor is updated and the sort cost is decremented
        // by the difference in real cost (A* heuristic doesn't change)
        if (edgestatus.set() == EdgeSet::kTemporary) {
          CheckIfLowerCostPath(edgestatus.status.index, forward_pred_idx, newcost, tc);
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
        edgelabels_.emplace_back(forward_pred_idx, edgeid, oppedge, directededge,
                      newcost, sortcost, dist, directededge->restrictions(),
                      directededge->opp_local_idx(), mode_, tc);
      }
    } else {
      // Expand reverse - set to get next edge from reverse adj. list
      // on the next pass
      expand_forward = false;
      expand_reverse = true;

      // Settle this edge
      edgestatus_reverse_->Update(pred2.edgeid(), EdgeSet::kPermanent);

      // Prune path if predecessor is not a through edge
      if (pred2.not_thru()) {
        continue;
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
        if (hierarchy_limits_reverse_[level].StopExpanding(pred2.distance())) {
          continue;
        }
      }

      // Get the opposing predecessor directed edge. Need to make sure we get
      // the correct one if a transition occurred
      const DirectedEdge* opp_pred_edge;
      if (pred2.opp_edgeid().Tile_Base() == tile2->id().Tile_Base()) {
        opp_pred_edge = tile2->directededge(pred2.opp_edgeid().id());
      } else {
        opp_pred_edge = graphreader.GetGraphTile(pred2.opp_edgeid().
                         Tile_Base())->directededge(pred2.opp_edgeid());
      }

      // Expand from end node in forward direction.
      uint32_t shortcuts = 0;
      GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
      const DirectedEdge* directededge = tile2->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0; i < nodeinfo->edge_count();
              i++, directededge++, edgeid++) {
        // Handle upward transition edges.
        if (directededge->trans_up()) {
          if (allow_transitions_ &&
              hierarchy_limits_reverse_[level].AllowUpwardTransition(dist)) {
            // Allow the transition edge. Add it to the adjacency list and
            // edge labels using the predecessor information. Transition
            // edges have no length.
            AddToAdjacencyListReverse(edgeid, pred2.sortcost());
            edgelabels_reverse_.emplace_back(reverse_pred_idx, edgeid,
                      pred2.opp_edgeid(), directededge, pred2.cost(),
                      pred2.sortcost(), pred2.distance(),
                      pred2.restrictions(), pred2.opp_local_idx(), mode_,
                      Cost(pred2.transition_cost(), pred2.transition_secs()));
            }
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

        // Get opposing directed edge and check if allowed.
        const DirectedEdge* opp_edge = t2->directededge(oppedge);
        if (!costing->AllowedReverse(directededge, pred2, opp_edge,
                                     tile2, edgeid)) {
          continue;
        }

        // Get cost. Use opposing edge for EdgeCost. Update the_shortcuts mask.
        // Separate the transition seconds so we can properly recover elapsed
        // time on the reverse path.
        shortcuts |= directededge->shortcut();
        Cost tc = costing->TransitionCostReverse(directededge->localedgeidx(),
                                    nodeinfo, opp_edge, opp_pred_edge);
        Cost newcost = pred2.cost() +
                       costing->EdgeCost(opp_edge, nodeinfo->density());
        newcost.cost += tc.cost;

        // Check if edge is temporarily labeled and this path has less cost. If
        // less cost the predecessor is updated and the sort cost is decremented
        // by the difference in real cost (A* heuristic doesn't change)
        if (edgestatus.set() != EdgeSet::kUnreached) {
          CheckIfLowerCostPathReverse(edgestatus.status.index, reverse_pred_idx,
                                      newcost, tc);
          continue;
        }

        // Find the sort cost (with A* heuristic) using the lat,lng at the
        // end node of the directed edge.
        float sortcost = newcost.cost + astarheuristic_reverse_.Get(
            t2->node(directededge->endnode())->latlng(), dist);

        // Add edge label, add to the adjacency list and set edge status
        AddToAdjacencyListReverse(edgeid, sortcost);
        edgelabels_reverse_.emplace_back(reverse_pred_idx, edgeid, oppedge,
                      directededge, newcost, sortcost, dist,
                      directededge->restrictions(),
                      directededge->opp_local_idx(), mode_, tc);
      }
    }
  }
  return {};    // If we are here the route failed
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
// by the difference in real cost (A* heuristic doesn't change)
void BidirectionalAStar::CheckIfLowerCostPath(const uint32_t idx,
                                         const uint32_t predindex,
                                         const Cost& newcost,
                                         const Cost& tc) {
  float dc = edgelabels_[idx].cost().cost - newcost.cost;
  if (dc > 0) {
    float oldsortcost = edgelabels_[idx].sortcost();
    float newsortcost = oldsortcost - dc;
    edgelabels_[idx].Update(predindex, newcost, newsortcost);
    edgelabels_[idx].set_transition_cost(tc);
    adjacencylist_->DecreaseCost(idx, newsortcost, oldsortcost);
  }
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
    edgelabels_reverse_[idx].set_transition_cost(tc);
    adjacencylist_reverse_->DecreaseCost(idx, newsortcost, oldsortcost);
  }
}

// Check if the edge on the forward search connects to a reached edge
// on the reverse search tree.
void BidirectionalAStar::CheckForwardConnection(const sif::EdgeLabel& pred) {
  // Get the opposing edge - if this edge has been reached then a shortest
  // path has been found to the end node of this directed edge.
  // An invalid opposing edge occurs for transition edges - skip them.
  GraphId oppedge = pred.opp_edgeid();
  if (oppedge.Is_Valid()) {
    EdgeStatusInfo oppedgestatus = edgestatus_reverse_->Get(oppedge);
    if (oppedgestatus.set() != EdgeSet::kUnreached) {
      if (threshold_ == 0) {
        threshold_ = GetThreshold(mode_, edgelabels_.size() + edgelabels_reverse_.size());
      }
      uint32_t predidx = edgelabels_reverse_[oppedgestatus.status.index].predecessor();
      float oppcost = (predidx == kInvalidLabel) ?
            0 : edgelabels_reverse_[predidx].cost().cost;
      float c = pred.cost().cost + oppcost +
          edgelabels_reverse_[oppedgestatus.status.index].transition_cost();
      if (c < best_connection_.cost) {
        best_connection_ = { pred.edgeid(), oppedge, c };
      }
    }
  }
}

// Check if the edge on the reverse search connects to a reached edge
// on the forward search tree.
void BidirectionalAStar::CheckReverseConnection(const sif::EdgeLabel& pred) {
  // Get the opposing edge - if this edge has been reached then a shortest
  // path has been found to the end node of this directed edge.
  // An invalid opposing edge occurs for transition edges - skip them.
  GraphId oppedge = pred.opp_edgeid();
  if (oppedge.Is_Valid()) {
    EdgeStatusInfo oppedgestatus = edgestatus_->Get(oppedge);
    if (oppedgestatus.set() != EdgeSet::kUnreached) {
      // Set a threshold to extend search
      if (threshold_ == 0) {
        threshold_ = GetThreshold(mode_, edgelabels_.size() + edgelabels_reverse_.size());
      }
      uint32_t predidx = edgelabels_[oppedgestatus.status.index].predecessor();
      float oppcost = (predidx == kInvalidLabel) ?
            0 : edgelabels_[predidx].cost().cost;
      float c = pred.cost().cost + oppcost +
            edgelabels_[oppedgestatus.status.index].transition_cost();
      if (c < best_connection_.cost) {
        best_connection_ = { oppedge, pred.edgeid(), c };
      }
    }
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
            directededge->opp_local_idx(), mode_, 0);

    // Set the initial not_thru flag to false. There is an issue with not_thru
    // flags on small loops. Set this to false here to override this for now.
    edgelabels_.back().set_not_thru(false);
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
  Cost c;
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

    // Get cost and sort cost (based on distance from endnode of this edge
    // to the origin. Make sure we use the reverse A* heuristic. Note that
    // the end node of the opposing edge is in the same tile as the directed
    // edge.
    Cost cost = costing->EdgeCost(opp_dir_edge,
                    graphreader.GetEdgeDensity(opp_edge_id)) * edge.dist;
    float dist = astarheuristic_reverse_.GetDistance(tile->node(
                    opp_dir_edge->endnode())->latlng());
    float sortcost = cost.cost + astarheuristic_reverse_.Get(dist);

    // Add EdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path. Make sure the opposing
    // edge (edgeid) is set.
    AddToAdjacencyListReverse(opp_edge_id, sortcost);
    edgelabels_reverse_.emplace_back(kInvalidLabel, opp_edge_id, edgeid,
             opp_dir_edge, cost, sortcost, dist, opp_dir_edge->restrictions(),
             opp_dir_edge->opp_local_idx(), mode_, c);

    // Set the initial not_thru flag to false. There is an issue with not_thru
    // flags on small loops. Set this to false here to override this for now.
    edgelabels_reverse_.back().set_not_thru(false);
  }
}

// Form the path from the adjacency list.
std::vector<PathInfo> BidirectionalAStar::FormPath(GraphReader& graphreader) {
  // Get the indexes where the connection occurs.
  uint32_t idx1 = edgestatus_->Get(best_connection_.edgeid).status.index;
  uint32_t idx2 = edgestatus_reverse_->Get(best_connection_.opp_edgeid).status.index;

  // Metrics (TODO - more accurate cost)
  uint32_t pathcost = edgelabels_[idx1].cost().cost +
                      edgelabels_reverse_[idx2].cost().cost;
  LOG_INFO("path_cost::" + std::to_string(pathcost));
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
  float tc = edgelabels_reverse_[idx2].transition_secs();

  // Append the reverse path from the destination - use opposing edges
  // The first edge on the reverse path is the same as the last on the forward
  // path, so get the predecessor.
  uint32_t edgelabel_index = edgelabels_reverse_[idx2].predecessor();
  while (edgelabel_index != kInvalidLabel) {
    const EdgeLabel& edgelabel = edgelabels_reverse_[edgelabel_index];
    GraphId oppedge = graphreader.GetOpposingEdgeId(edgelabel.edgeid());

    // Get elapsed time on the edge, then add the transition cost at
    // prior edge.
    uint32_t predidx = edgelabel.predecessor();
    if (predidx == kInvalidLabel) {
      secs += edgelabel.cost().secs;
    } else {
      secs += edgelabel.cost().secs - edgelabels_reverse_[predidx].cost().secs;
    }
    secs += tc;
    path.emplace_back(edgelabel.mode(), static_cast<uint32_t>(secs),
                            oppedge, edgelabel.tripid());

    // Update edgelabel_index and transition cost to apply at next iteration
    edgelabel_index = predidx;
    tc = edgelabel.transition_secs();
  }
  return path;
}

}
}
