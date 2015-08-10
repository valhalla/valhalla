#include <map>
#include <algorithm>
#include "thor/bidirectional_astar.h"
#include <valhalla/baldr/datetime.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;

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

  // Make sure hierarchy transitions are not allowed (though code below also
  // disables it).
  allow_transitions_ = false;
}

// Calculate best path using bi-directional A*. No hierarchies or time
// dependencies are used. Suitable for pedestrian routes (and bicycle?).
std::vector<PathInfo> BidirectionalAStar::GetBestPath(const PathLocation& origin,
             const PathLocation& destination, GraphReader& graphreader,
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

  // Find shortest path. Switch between a forward direction and a reverse
  // direction search based on the current costs. Alternating like this
  // prevents one tree from expanding much more quickly (if in a sparser
  // portion of the graph) rather than strictly alternating.
  uint32_t n = 0;
  uint32_t predindex, predindex2;
  EdgeLabel pred, pred2;
  const GraphTile* tile;
  bool expand_forward  = true;
  bool expand_reverse  = true;
  bool forward_exhausted = false;
  bool reverse_exhausted = false;
  while (true) {
    // Get the next predecessor and cost (based on which direction was
    // expanded in prior step)
    if (expand_forward) {
      predindex = adjacencylist_->Remove(edgelabels_);
      if (predindex != kInvalidLabel) {
        pred = edgelabels_[predindex];
      } else {
        forward_exhausted = true;
      }
    }
    if (expand_reverse) {
      predindex2 = adjacencylist_reverse_->Remove(edgelabels_reverse_);
      if (predindex2 != kInvalidLabel) {
        pred2 = edgelabels_reverse_[predindex2];
      } else {
        reverse_exhausted = true;
      }
    }

    // Expand from the search direction with lower cost
    if (!forward_exhausted &&
        (reverse_exhausted || pred.cost().cost < pred2.cost().cost)) {
      // Expand forward - set to get next edge from forward adj. list
      // on the next pass
      expand_forward = true;
      expand_reverse = false;

      // Mark edge as done - copy the EdgeLabel for use in costing
      edgestatus_->Update(pred.edgeid(), kPermanent);

      // Get the opposing edge - if permanently labeled in reverse search set
      // we have the shortest path
      GraphId oppedge = graphreader.GetOpposingEdgeId(pred.edgeid());
      EdgeStatusInfo oppedgestatus = edgestatus_reverse_->Get(oppedge);
      if (oppedgestatus.status.set == kPermanent) {
        return FormPath(predindex,oppedgestatus.status.index, graphreader);
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

      // Expand from end node in forward direction.
      GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
      const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                  i++, directededge++, edgeid++) {
        // Skip transition edges and edges where no access is allowed
        if (directededge->trans_up() ||
            !costing->Allowed(directededge, pred)) {
          continue;
        }

        // Get the current set. Skip this edge if permanently labeled (best
        // path already found to this directed edge).
        EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
        if (edgestatus.status.set == kPermanent) {
          continue;
        }

        // Get cost
        Cost newcost = pred.cost() +
                       costing->EdgeCost(directededge, nodeinfo->density()) +
                       costing->TransitionCost(directededge, nodeinfo, pred);

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
        AddToAdjacencyList(edgeid, sortcost);
        edgelabels_.emplace_back(predindex, edgeid, directededge,
                      newcost, sortcost, dist, directededge->restrictions(),
                      directededge->opp_local_idx(), mode_);
      }
    } else if (!reverse_exhausted) {
      // Expand reverse - set to get next edge from reverse adj. list
      // on the next pass
      expand_forward = false;
      expand_reverse = true;

      // Mark edge as done - copy the EdgeLabel for use in costing
      edgestatus_reverse_->Update(pred2.edgeid(), kPermanent);

      // Get the opposing edge - if permanently labeled in forward search set
      // we have the shortest path
      GraphId oppedge = graphreader.GetOpposingEdgeId(pred2.edgeid());
      EdgeStatusInfo oppedgestatus = edgestatus_->Get(oppedge);
      if (oppedgestatus.status.set == kPermanent) {
        return FormPath(oppedgestatus.status.index, predindex2, graphreader);
      }

      // Get the end node of the prior directed edge. Skip if tile not found
      // (can happen with regional data sets).
      GraphId node = pred2.endnode();
      if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
        continue;
      }

      // Check access at the node
      const NodeInfo* nodeinfo = tile->node(node);
      if (!costing->Allowed(nodeinfo)) {
        continue;
      }

      // Get the opposing predecessor directed edge
      const DirectedEdge* opp_pred_edge = tile->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, opp_pred_edge++) {
        if (opp_pred_edge->localedgeidx() == pred2.opp_local_idx())
          break;
      }

      // Expand from end node in forward direction.
      GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
      const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0; i < nodeinfo->edge_count();
              i++, directededge++, edgeid++) {
        // Skip transition edges
        if (directededge->trans_up()) {
          continue;
        }

        // Get the current set. Skip this edge if permanently labeled (best
        // path already found to this directed edge).
        EdgeStatusInfo edgestatus = edgestatus_reverse_->Get(edgeid);
        if (edgestatus.status.set == kPermanent) {
          continue;
        }

        // Get opposing edge and check if allowed.
        const DirectedEdge* opp_edge = graphreader.GetOpposingEdge(edgeid);
        if (opp_edge == nullptr ||
            !costing->AllowedReverse(directededge, opp_edge, opp_pred_edge)) {
          continue;
        }

       // Get cost. TODO - do we need to use opposing edge for EdgeCost?
       Cost newcost = pred2.cost() +
              costing->EdgeCost(directededge, nodeinfo->density()) +
              costing->TransitionCostReverse(directededge->localedgeidx(),
                                             nodeinfo, opp_edge, opp_pred_edge);

       // Check if edge is temporarily labeled and this path has less cost. If
       // less cost the predecessor is updated and the sort cost is decremented
       // by the difference in real cost (A* heuristic doesn't change)
       if (edgestatus.status.set == kTemporary) {
         CheckIfLowerCostPathReverse(edgestatus.status.index, predindex2, newcost);
         continue;
       }

       // Find the sort cost (with A* heuristic) using the lat,lng at the
       // end node of the directed edge. Skip if tile not found.
       if ((tile = graphreader.GetGraphTile(directededge->endnode())) == nullptr) {
         continue;
       }
       float dist = astarheuristic_reverse_.GetDistance(tile->node(
                 directededge->endnode())->latlng());
       float sortcost = newcost.cost + astarheuristic_reverse_.Get(dist);

       // Add edge label, add to the adjacency list and set edge status
       AddToAdjacencyListReverse(edgeid, sortcost);
       edgelabels_reverse_.emplace_back(predindex2, edgeid, directededge,
                     newcost, sortcost, dist, directededge->restrictions(),
                     directededge->opp_local_idx(), mode_);
      }
    }

    // Break out of loop if neither search can be expanded
    if (predindex == kInvalidLabel && predindex2 == kInvalidLabel) {
      break;
    }
  }

  // If we are here the route failed
  return {};
}

// Convenience method to add an edge to the reverse adjacency list and
// temporarily label it.
void BidirectionalAStar::AddToAdjacencyListReverse(const GraphId& edgeid,
                                        const float sortcost) {
  uint32_t idx = edgelabels_reverse_.size();
  adjacencylist_reverse_->Add(idx, sortcost);
  edgestatus_reverse_->Set(edgeid, kTemporary, idx);
}

// Check if edge is temporarily labeled and this path has less cost. If
// less cost the predecessor is updated and the sort cost is decremented
// by the difference in real cost (A* heuristic doesn't change). This is
// done for the reverse search.
void BidirectionalAStar::CheckIfLowerCostPathReverse(const uint32_t idx,
                                         const uint32_t predindex,
                                         const Cost& newcost) {
  float dc = edgelabels_reverse_[idx].cost().cost - newcost.cost;
  if (dc > 0) {
    float oldsortcost = edgelabels_reverse_[idx].sortcost();
    float newsortcost = oldsortcost - dc;
    edgelabels_reverse_[idx].Update(predindex, newcost, newsortcost);
    adjacencylist_reverse_->DecreaseCost(idx, newsortcost, oldsortcost);
  }
}

// Add edges at the origin to the forward adjacency list.
void BidirectionalAStar::SetOrigin(GraphReader& graphreader,
                 const PathLocation& origin,
                 const std::shared_ptr<DynamicCost>& costing) {
  // Get sort heuristic based on distance from origin to destination
  float dist = astarheuristic_.GetDistance(origin.vertex());
  float heuristic = astarheuristic_.Get(dist);

  // Iterate through edges and add to adjacency list
  for (const auto& edge : origin.edges()) {
    // Get the directed edge
    GraphId edgeid = edge.id;
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get cost and sort cost
    Cost cost = (costing->EdgeCost(directededge, 0) * (1.0f - edge.dist));
    float sortcost = cost.cost + heuristic;

    // Add EdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path.
    AddToAdjacencyList(edgeid, sortcost);
    edgelabels_.emplace_back(kInvalidLabel, edgeid, directededge, cost,
            sortcost, dist, directededge->restrictions(),
            directededge->opp_local_idx(), mode_);
  }
}

// Add destination edges to the reverse path adjacency list.
void BidirectionalAStar::SetDestination(GraphReader& graphreader,
                     const PathLocation& dest,
                     const std::shared_ptr<DynamicCost>& costing) {
  // Get sort heuristic based on distance from origin to destination
  float dist = astarheuristic_reverse_.GetDistance(dest.vertex());
  float heuristic = astarheuristic_reverse_.Get(dist);

  // Iterate through edges and add to adjacency list
  for (const auto& edge : dest.edges()) {
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

    // Get cost and sort cost
    Cost cost = (costing->EdgeCost(opp_dir_edge, 0) * (1.0f - edge.dist));
    float sortcost = cost.cost + heuristic;

    // Add EdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path.
    AddToAdjacencyListReverse(opp_edge_id, sortcost);
    edgelabels_reverse_.emplace_back(kInvalidLabel, opp_edge_id,
                           opp_dir_edge, cost, sortcost, dist, 0,
                           opp_dir_edge->opp_local_idx(), mode_);
  }
}

// Form the path from the adjacency list.
std::vector<PathInfo> BidirectionalAStar::FormPath(const uint32_t idx1,
              const uint32_t idx2, GraphReader& graphreader) {
  // Metrics (TODO)
//LOG_INFO("FormPath2: path_cost::" + std::to_string(edgelabels_[dest].cost().cost));
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

  // Append the reverse path from the destination - use opposing edges
  // Get the elapsed time at the end of the forward path
  uint32_t elapsed_time = path.back().elapsed_time;
  uint32_t prior_time = 0;
  for (auto edgelabel_index = idx2; edgelabel_index != kInvalidLabel;
      edgelabel_index = edgelabels_reverse_[edgelabel_index].predecessor()) {
    const EdgeLabel& edgelabel = edgelabels_reverse_[edgelabel_index];
    if (edgelabel_index != idx2) {
    	// Add opposing edge to the path, increment cost by delta along
    	// the reverse path
    	GraphId oppedge = graphreader.GetOpposingEdgeId(edgelabel.edgeid());
    	elapsed_time += (prior_time - edgelabel.cost().secs);
    	path.emplace_back(edgelabel.mode(), elapsed_time, oppedge,
    	                  edgelabel.tripid());
    }
    prior_time = edgelabel.cost().secs;
  }
  return path;
}


}
}
