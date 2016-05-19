#include <vector>
#include <algorithm>
#include "thor/costmatrix.h"
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

// Constructor with cost threshold.
CostMatrix::CostMatrix(float cost_threshold)
    : allow_transitions_(false),
      source_count_(0),
      cost_threshold_(cost_threshold) {
}

// Clear the temporary information generated during time + distance matrix
// construction.
void CostMatrix::Clear() {
  // Clear the target edge markings
  targets_.clear();

  // Clear all source adjacency lists, edge labels, and edge status
  for (auto adj : source_adjacency_) {
    delete adj;
  }
  source_adjacency_.clear();

  for (auto el : source_edgelabel_) {
    el.clear();
  }
  source_edgelabel_.clear();

  for (auto es : source_edgestatus_) {
    es.Init();
  }
  source_edgestatus_.clear();

  source_hierarchy_limits_.clear();

  // Clear all target adjacency lists, edge labels, and edge status
  for (auto adj : target_adjacency_) {
    delete adj;
  }
  target_adjacency_.clear();

  for (auto el : target_edgelabel_) {
    el.clear();
  }
  target_edgelabel_.clear();

  for (auto es : target_edgestatus_) {
    es.Init();
  }
  target_edgestatus_.clear();

  target_hierarchy_limits_.clear();
  source_status_.clear();
  target_status_.clear();
}

// Form a time distance matrix from the set of source locations
// to the set of target locations.
std::vector<TimeDistance> CostMatrix::SourceToTarget(
        const std::vector<baldr::PathLocation>& source_location_list,
        const std::vector<baldr::PathLocation>& target_location_list,
        baldr::GraphReader& graphreader,
        const std::shared_ptr<sif::DynamicCost>* mode_costing,
        const sif::TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode_)];
  allow_transitions_ = costing->AllowTransitions();

  // Set the source and target locations
  Clear();
  SetSources(graphreader, source_location_list, costing);
  SetTargets(graphreader, target_location_list, costing);

  // Initialize best connections. Any locations that are the same
  // get set to 0 time, distance.
  Initialize(source_location_list, target_location_list);

  // TODO - create termination criteria
  int n = 0;
  while (true) {
    // Iterate all target locations in a backwards search
    for (uint32_t i = 0; i < target_location_list.size(); i++) {
      BackwardSearch(i, graphreader, costing);
    }

    // Iterate all source locations in a forward search. Mark any
    // source target pairs that are found during forward search.
    for (uint32_t i = 0; i < source_location_list.size(); i++) {
      ForwardSearch(i, graphreader, costing);
    }

    // TODO - how to break out or know we are complete?
    if (n++ > 50000) {
      break;
    }
  }

  // Form the time, distance matrix from the destinations list
  uint32_t idx = 0;
  std::vector<TimeDistance> td;
  for (auto& connection : best_connection_) {
    td.emplace_back(connection.cost.secs, connection.distance);
    idx++;
  }
  return td;
}

// Initialize all time distance to "not found". Any locations that
// are the same get set to 0 time, distance
void CostMatrix::Initialize(
        const std::vector<PathLocation>& source_locations,
        const std::vector<PathLocation>& target_locations) {
  GraphId empty;
  Cost trivial_cost(0.0f, 0.0f);
  Cost max_cost(kMaxCost, kMaxCost);
  for (auto& source : source_locations) {
    for (auto& target : target_locations) {
      if (source.latlng_ == target.latlng_) {
        best_connection_.emplace_back(empty, empty, trivial_cost, 0.0f);
      } else {
        best_connection_.emplace_back(empty, empty, max_cost, 0.0f);
      }
    }
  }
}

// Iterate the forward search from the source/origin location.
void CostMatrix::ForwardSearch(const uint32_t index,
                  baldr::GraphReader& graphreader,
                  const std::shared_ptr<sif::DynamicCost>& costing) {
  // Get the adjacency list, edge label list, and edge status
  // for this source location
  auto* adj = source_adjacency_[index];
  auto& edgelabels = source_edgelabel_[index];
  auto& edgestate = source_edgestatus_[index];
  auto& hierarchy_limits = source_hierarchy_limits_[index];

  // Get the next edge from the adjacency list
  EdgeLabel pred;
  uint32_t predindex = adj->Remove(edgelabels);
  if (predindex != kInvalidLabel) {
    pred = edgelabels[predindex];
  } else {
    // TODO: Mark this location - since cannot expand anymore
//    LOG_INFO("Cannot expand " + std::to_string(index) + " forward anymore");
    return;
  }

  // Settle this edge
  edgestate.Update(pred.edgeid(), EdgeSet::kPermanent);

  // Check for connections to backwards search.
  CheckForwardConnections(index, pred);

  // Prune path if predecessor is not a through edge
  if (pred.not_thru()) {
    return;
  }

  // Get the end node of the prior directed edge. Skip if tile not found
  // (can happen with regional data sets).
  const GraphTile* tile;
  GraphId node = pred.endnode();
  if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
    return;
  }

  // Check access at the node
  const NodeInfo* nodeinfo = tile->node(node);
  if (!costing->Allowed(nodeinfo)) {
    return;
  }

  // Check hierarchy. Count upward transitions (counted on the level
  // transitioned from). Do not expand based on hierarchy level based on
  // number of upward transitions.
  if (allow_transitions_) {
    uint32_t level = node.level();
    if (pred.trans_up()) {
      hierarchy_limits[level+1].up_transition_count++;
    }
    if (hierarchy_limits[level].StopExpanding()) {
      return;
    }
  }

  // Expand from end node in forward direction.
  uint32_t shortcuts = 0;
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
          i++, directededge++, edgeid++) {
    // Handle upward transition edges
    if (directededge->trans_up()) {
      if (allow_transitions_) {
        // Allow the transition edge. Add it to the adjacency list and
        // edge labels using the predecessor information. Transition
        // edges have no length.
        uint32_t idx = edgelabels.size();
        adj->Add(idx, pred.sortcost());
        edgestate.Set(edgeid, EdgeSet::kTemporary, idx);
        edgelabels.emplace_back(predindex, edgeid, pred.opp_edgeid(),
                        directededge, pred.cost(), pred.restrictions(),
                        pred.opp_local_idx(), mode_,
                        Cost(pred.transition_cost(), pred.transition_secs()),
                        pred.walking_distance());
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

    // Get the current set. Skip this edge if permanently labeled (best
    // path already found to this directed edge).
    EdgeStatusInfo edgestatus = edgestate.Get(edgeid);
    if (edgestatus.set() == EdgeSet::kPermanent) {
      continue;
    }

    // Get cost and accumulated distance. Update the_shortcuts mask.
    shortcuts |= directededge->shortcut();
    Cost tc = costing->TransitionCost(directededge, nodeinfo, pred);
    Cost newcost = pred.cost() + tc +
                costing->EdgeCost(directededge, nodeinfo->density());
    uint32_t distance = pred.walking_distance() + directededge->length();

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated along with new cost and distance.
    if (edgestatus.set() == EdgeSet::kTemporary) {
      uint32_t idx = edgestatus.status.index;
      if (newcost.cost < edgelabels[idx].cost().cost) {
        float oldsortcost = edgelabels[idx].sortcost();
        edgelabels[idx].Update(predindex, newcost, newcost.cost, tc, distance);
        adj->DecreaseCost(idx, newcost.cost, oldsortcost);
      }
      continue;
    }

    // Get end node tile (skip if tile is not found) and opposing edge Id
    const GraphTile* t2 = directededge->leaves_tile() ?
          graphreader.GetGraphTile(directededge->endnode()) : tile;
    if (t2 == nullptr) {
      continue;
    }
    GraphId oppedge = GetOpposingEdgeId(directededge, t2);

    // Add edge label, add to the adjacency list and set edge status
    adj->Add(edgelabels.size(), newcost.cost);
    edgestate.Set(edgeid, EdgeSet::kTemporary, edgelabels.size());
    edgelabels.emplace_back(predindex, edgeid, oppedge, directededge,
                    newcost, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_, tc, distance);
  }
}

// Check if the edge on the forward search connects to a reached edge
// on the reverse search trees.
void CostMatrix::CheckForwardConnections(const uint32_t index,
                              const EdgeLabel& pred) {
  // Get the opposing edge. An invalid opposing edge occurs for transition
  // edges - skip them.
  GraphId oppedge = pred.opp_edgeid();
  if (oppedge.Is_Valid()) {
    // Get a list of target locations whose reverse search
    // has reached this edge.
    auto targets = targets_.find(oppedge);
    if (targets == targets_.end()) {
      return;
    }

    // Iterate through the targets
    for (auto target_index : targets->second) {
      const auto& edgestate = target_edgestatus_[target_index];

      // If this edge has been reached then a shortest path has been found
      // to the end node of this directed edge. TODO - set a threshold?
      EdgeStatusInfo oppedgestatus = edgestate.Get(oppedge);
      if (oppedgestatus.set() != EdgeSet::kUnreached) {
        const auto& edgelabels = target_edgelabel_[target_index];

        uint32_t predidx = edgelabels[oppedgestatus.status.index].predecessor();
        float oppcost = (predidx == kInvalidLabel) ?
                  0 : edgelabels[predidx].cost().cost;
        float c = pred.cost().cost + oppcost +
                  edgelabels[oppedgestatus.status.index].transition_cost();

        // Check if best connection
        uint32_t idx = index * source_count_ + target_index;
        if (c < best_connection_[idx].cost.cost) {
          float oppsec = (predidx == kInvalidLabel) ?
                        0 : edgelabels[predidx].cost().secs;
          float oppdist = (predidx == kInvalidLabel) ?
                        0 : edgelabels[predidx].walking_distance();
          float s = pred.cost().secs + oppsec +
                    edgelabels[oppedgestatus.status.index].transition_secs();
          float d = pred.walking_distance() + oppdist;
          best_connection_[idx] = { pred.edgeid(), oppedge, Cost(c, s), d };
        }
      }
    }
  }
}

// Expand the backwards search trees.
void CostMatrix::BackwardSearch(const uint32_t index,
                 baldr::GraphReader& graphreader,
                 const std::shared_ptr<sif::DynamicCost>& costing) {
  // Get the adjacency list and edge label list for this target location
  auto* adj = target_adjacency_[index];
  auto& edgelabels = target_edgelabel_[index];
  auto& edgestate = target_edgestatus_[index];
  auto& hierarchy_limits = target_hierarchy_limits_[index];

  // Get the next edge from the adjacency list
  EdgeLabel pred;
  uint32_t predindex = adj->Remove(edgelabels);
  if (predindex != kInvalidLabel) {
    pred = edgelabels[predindex];
  } else {
    // TODO: Mark this location - since cannot expand anymore
 //   LOG_INFO("Cannot expand " + std::to_string(index) + " backwards anymore. n = " +
 //            std::to_string(edgelabels.size()));
    return;
  }

  // Settle this edge
  edgestate.Update(pred.edgeid(), EdgeSet::kPermanent);

  // Prune path if predecessor is not a through edge
  if (pred.not_thru()) {
    return;
  }

  // Get the end node of the prior directed edge. Skip if tile not found
  // (can happen with regional data sets).
  const GraphTile* tile;
  GraphId node = pred.endnode();
  if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
    return;
  }

  // Check access at the node
  const NodeInfo* nodeinfo = tile->node(node);
  if (!costing->Allowed(nodeinfo)) {
    return;
  }

  // Check hierarchy. Count upward transitions (counted on the level
  // transitioned from). Do not expand based on hierarchy level based on
  // number of upward transitions and distance to the destination
  if (allow_transitions_) {
    uint32_t level = node.level();
    if (pred.trans_up()) {
      hierarchy_limits[level+1].up_transition_count++;
    }
    if (hierarchy_limits[level].StopExpanding()) {
      return;
    }
  }

  // Get the opposing predecessor directed edge. Need to make sure we get
  // the correct one if a transition occurred
  const DirectedEdge* opp_pred_edge;
  if (pred.opp_edgeid().Tile_Base() == tile->id().Tile_Base()) {
    opp_pred_edge = tile->directededge(pred.opp_edgeid().id());
  } else {
    opp_pred_edge = graphreader.GetGraphTile(pred.opp_edgeid().
                     Tile_Base())->directededge(pred.opp_edgeid());
  }

  // Expand from end node in forward direction.
  uint32_t shortcuts = 0;
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count();
          i++, directededge++, edgeid++) {
    // Handle upward hierarchy transition
    if (directededge->trans_up()) {
      if (allow_transitions_) {
        // Allow the transition edge. Add it to the adjacency list and
        // edge labels using the predecessor information. Transition
        // edges have no length.
        adj->Add(edgelabels.size(), pred.sortcost());
        edgestate.Set(edgeid, EdgeSet::kTemporary, edgelabels.size());
        edgelabels.emplace_back(predindex, edgeid, pred.opp_edgeid(),
                        directededge, pred.cost(), pred.restrictions(),
                        pred.opp_local_idx(), mode_,
                        Cost(pred.transition_cost(), pred.transition_secs()),
                        pred.walking_distance());
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
    EdgeStatusInfo edgestatus = edgestate.Get(edgeid);
    if (edgestatus.set() == EdgeSet::kPermanent) {
      continue;
    }

    // Get opposing edge Id and end node tile
    const GraphTile* t2 = directededge->leaves_tile() ?
          graphreader.GetGraphTile(directededge->endnode()) : tile;
    if (t2 == nullptr) {
      continue;
    }
    GraphId oppedge = GetOpposingEdgeId(directededge, t2);

    // Get opposing directed edge and check if allowed.
    const DirectedEdge* opp_edge = t2->directededge(oppedge);
    if (!costing->AllowedReverse(directededge, pred, opp_edge,
                      tile, edgeid)) {
      continue;
    }

    // Get cost and accumulated distance. Use opposing edge for EdgeCost.
    // Update the shortcut masek
    shortcuts |= directededge->shortcut();
    Cost tc = costing->TransitionCostReverse(directededge->localedgeidx(),
                   nodeinfo, opp_edge, opp_pred_edge);
    Cost newcost = pred.cost() + tc +
                   costing->EdgeCost(opp_edge, nodeinfo->density());
    uint32_t distance = pred.walking_distance() + directededge->length();

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated along with new cost and distance.
    if (edgestatus.set() != EdgeSet::kUnreached) {
      uint32_t idx = edgestatus.status.index;
      if (newcost.cost < edgelabels[idx].cost().cost) {
        float oldsortcost = edgelabels[idx].sortcost();
        edgelabels[idx].Update(predindex, newcost, newcost.cost, tc, distance);
        adj->DecreaseCost(idx, newcost.cost, oldsortcost);
      }
      continue;
    }

    // Add edge label, add to the adjacency list and set edge status
    // Add to the list or targets that have reached this edge
    adj->Add(edgelabels.size(), newcost.cost);
    edgestate.Set(edgeid, EdgeSet::kTemporary, edgelabels.size());
    edgelabels.emplace_back(predindex, edgeid, oppedge,
       directededge, newcost, directededge->restrictions(),
       directededge->opp_local_idx(), mode_, tc, distance);
    targets_[edgeid].push_back(index);
  }
}

// Sets the source/origin locations. Search expands forward from these
// locations.
void CostMatrix::SetSources(baldr::GraphReader& graphreader,
                      const std::vector<baldr::PathLocation>& sources,
                      const std::shared_ptr<sif::DynamicCost>& costing) {
  // Allocate edge labels and edge status
  source_count_ = sources.size();
  source_edgelabel_.resize(source_count_);
  source_edgestatus_.resize(source_count_);
  source_adjacency_.resize(source_count_);
  source_hierarchy_limits_.resize(source_count_);

  // Adjacency list information
  uint32_t bucketsize = costing->UnitSize();
  float range = kBucketCount * bucketsize;

  // Go through each source location
  uint32_t index = 0;
  Cost empty_cost;
  for (const auto& origin : sources) {
    // Allocate the adjacency list and hierarchy limits for this source
    source_adjacency_[index] = new AdjacencyList(0, range, bucketsize);
    source_hierarchy_limits_[index] = costing->GetHierarchyLimits();

    // Iterate through edges and add to adjacency list
    for (const auto& edge : (origin.edges())) {
      // If origin is at a node - skip any inbound edge (dist = 1)
      if (origin.IsNode() && edge.dist == 1) {
        continue;
      }

      // Get the directed edge and the opposing edge Id
      GraphId edgeid = edge.id;
      const GraphTile* tile = graphreader.GetGraphTile(edgeid);
      const DirectedEdge* directededge = tile->directededge(edgeid);
      GraphId oppedge = graphreader.GetOpposingEdgeId(edgeid);

      // Get cost. Get distance along the remainder of this edge.
      Cost cost = costing->EdgeCost(directededge,
                    graphreader.GetEdgeDensity(edgeid)) * (1.0f - edge.dist);
      uint32_t d = static_cast<uint32_t>(directededge->length() *
                               (1.0f - edge.dist));

      // Add EdgeLabel to the adjacency list (but do not set its status).
      // Set the predecessor edge index to invalid to indicate the origin
      // of the path.
      source_adjacency_[index]->Add(source_edgelabel_[index].size(), cost.cost);
      EdgeLabel edge_label(kInvalidLabel, edgeid, oppedge, directededge, cost,
                           directededge->restrictions(),
                           directededge->opp_local_idx(), mode_, empty_cost, d);
      source_edgelabel_[index].push_back(std::move(edge_label));
    }
    index++;
  }
}

// Set the target/destination locations. Search expands backwards from
// these locations.
void CostMatrix::SetTargets(baldr::GraphReader& graphreader,
                     const std::vector<baldr::PathLocation>& targets,
                     const std::shared_ptr<sif::DynamicCost>& costing) {
  // Allocate target edge labels and edge status
  target_edgelabel_.resize(targets.size());
  target_edgestatus_.resize(targets.size());
  target_adjacency_.resize(targets.size());
  target_hierarchy_limits_.resize(targets.size());


  // Adjacency list information
  uint32_t bucketsize = costing->UnitSize();
  float range = kBucketCount * bucketsize;

  // Go through each target location
  uint32_t index = 0;
  Cost empty_cost;
  for (const auto& dest : targets) {
    // Allocate the adjacency list and hierarchy limits for target location
    target_adjacency_[index] = new AdjacencyList(0, range, bucketsize);
    target_hierarchy_limits_[index] = costing->GetHierarchyLimits();

    // Iterate through edges and add to adjacency list
    for (const auto& edge : (dest.edges())) {
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

      // Get cost. Get distance along the remainder of this edge.
      Cost cost = costing->EdgeCost(opp_dir_edge,
                      graphreader.GetEdgeDensity(opp_edge_id)) * edge.dist;
      uint32_t d = static_cast<uint32_t>(directededge->length() * edge.dist);

      // Add EdgeLabel to the adjacency list (but do not set its status).
      // Set the predecessor edge index to invalid to indicate the origin
      // of the path. Set the origin flag
      target_adjacency_[index]->Add(target_edgelabel_[index].size(), cost.cost);
      EdgeLabel edge_label(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost,
              opp_dir_edge->restrictions(), opp_dir_edge->opp_local_idx(),
              mode_, empty_cost, d);
      target_edgelabel_[index].push_back(std::move(edge_label));
    }
    index++;
  }
}

}
}
