#include "thor/bidirectional_astar.h"
#include "baldr/datetime.h"
#include "midgard/logging.h"
#include <algorithm>
#include <map>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

constexpr uint64_t kInitialEdgeLabelCountBD = 1000000;

// Threshold (seconds) to extend search once the first connection has been found.
// TODO - this is currently set based on some exceptional cases (e.g. routes taking
// the PA Turnpike which have very long edges). Using a metric based on maximum edge
// cost creates large performance drops - so perhaps some other metric can be found?
constexpr float kThresholdDelta = 420.0f;

// Default constructor
BidirectionalAStar::BidirectionalAStar() : PathAlgorithm() {
  threshold_ = 0;
  mode_ = TravelMode::kDrive;
  access_mode_ = kAutoAccess;
  travel_type_ = 0;
  cost_diff_ = 0.0f;
  adjacencylist_forward_ = nullptr;
  adjacencylist_reverse_ = nullptr;
}

// Destructor
BidirectionalAStar::~BidirectionalAStar() {
  Clear();
}

// Clear the temporary information generated during path construction.
void BidirectionalAStar::Clear() {
  edgelabels_forward_.clear();
  edgelabels_reverse_.clear();
  adjacencylist_forward_.reset();
  adjacencylist_reverse_.reset();
  edgestatus_forward_.clear();
  edgestatus_reverse_.clear();

  // Set the ferry flag to false
  has_ferry_ = false;
}

// Initialize the A* heuristic and adjacency lists for both the forward
// and reverse search.
void BidirectionalAStar::Init(const PointLL& origll, const PointLL& destll) {
  // Initialize the A* heuristics
  float factor = costing_->AStarCostFactor();
  astarheuristic_forward_.Init(destll, factor);
  astarheuristic_reverse_.Init(origll, factor);

  // Reserve size for edge labels - do this here rather than in constructor so
  // to limit how much extra memory is used for persistent objects
  edgelabels_forward_.reserve(kInitialEdgeLabelCountBD);
  edgelabels_reverse_.reserve(kInitialEdgeLabelCountBD);

  // Set up lambdas to get sort costs
  const auto forward_edgecost = [this](const uint32_t label) {
    return edgelabels_forward_[label].sortcost();
  };
  const auto reverse_edgecost = [this](const uint32_t label) {
    return edgelabels_reverse_[label].sortcost();
  };

  // Construct adjacency list and initialize edge status lookup.
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing_->UnitSize();
  float range = kBucketCount * bucketsize;
  float mincostf = astarheuristic_forward_.Get(origll);
  adjacencylist_forward_.reset(new DoubleBucketQueue(mincostf, range, bucketsize, forward_edgecost));
  float mincostr = astarheuristic_reverse_.Get(destll);
  adjacencylist_reverse_.reset(new DoubleBucketQueue(mincostr, range, bucketsize, reverse_edgecost));
  edgestatus_forward_.clear();
  edgestatus_reverse_.clear();

  // Set the cost diff between forward and reverse searches (due to distance
  // approximator differences). This is used to "even" the forward and reverse
  // searches.
  cost_diff_ = mincostf - mincostr;

  // Initialize best connection with max cost
  best_connection_ = {GraphId(), GraphId(), std::numeric_limits<float>::max()};

  // Set the cost threshold to the maximum float value. Once the initial connection is found
  // the threshold is set.
  threshold_ = std::numeric_limits<float>::max();

  // Support for hierarchy transitions
  hierarchy_limits_forward_ = costing_->GetHierarchyLimits();
  hierarchy_limits_reverse_ = costing_->GetHierarchyLimits();
}

// Expand from a node in the forward direction
void BidirectionalAStar::ExpandForward(GraphReader& graphreader,
                                       const GraphId& node,
                                       const BDEdgeLabel& pred,
                                       const uint32_t pred_idx,
                                       const bool from_transition) {
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }
  const NodeInfo* nodeinfo = tile->node(node);
  if (!costing_->Allowed(nodeinfo)) {
    return;
  }

  // Expand from end node in forward direction.
  uint32_t shortcuts = 0;
  GraphId edgeid = {node.tileid(), node.level(), nodeinfo->edge_index()};
  EdgeStatusInfo* es = edgestatus_forward_.GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(edgeid);
  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++directededge, ++edgeid, ++es) {
    // Skip shortcut edges until we have stopped expanding on the next level. Use regular
    // edges while still expanding on the next level since we can still transition down to
    // that level. If using a shortcut, set the shortcuts mask. Skip if this is a regular
    // edge superseded by a shortcut.
    if (directededge->is_shortcut()) {
      if (hierarchy_limits_forward_[edgeid.level() + 1].StopExpanding()) {
        shortcuts |= directededge->shortcut();
      } else {
        continue;
      }
    } else if (shortcuts & directededge->superseded()) {
      continue;
    }

    // Skip this edge if edge is permanently labeled (best path already found
    // to this directed edge), if no access is allowed (based on costing method),
    // or if a complex restriction prevents transition onto this edge.
    if (es->set() == EdgeSet::kPermanent ||
        !costing_->Allowed(directededge, pred, tile, edgeid, 0, 0) ||
        costing_->Restricted(directededge, pred, edgelabels_forward_, tile, edgeid, true)) {
      continue;
    }

    // Get cost. Separate out transition cost.
    Cost tc = costing_->TransitionCost(directededge, nodeinfo, pred);
    Cost newcost = pred.cost() + tc + costing_->EdgeCost(directededge, tile->GetSpeed(directededge));
    ;

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (es->set() == EdgeSet::kTemporary) {
      BDEdgeLabel& lab = edgelabels_forward_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_forward_->decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, tc);
      }
      continue;
    }

    // Get end node tile (skip if tile is not found) and opposing edge Id
    const GraphTile* t2 =
        directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
    if (t2 == nullptr) {
      continue;
    }
    GraphId oppedge = t2->GetOpposingEdgeId(directededge);

    // Find the sort cost (with A* heuristic) using the lat,lng at the
    // end node of the directed edge.
    float dist = 0.0f;
    float sortcost =
        newcost.cost + astarheuristic_forward_.Get(t2->get_node_ll(directededge->endnode()), dist);

    // Add edge label, add to the adjacency list and set edge status
    uint32_t idx = edgelabels_forward_.size();
    edgelabels_forward_.emplace_back(pred_idx, edgeid, oppedge, directededge, newcost, sortcost, dist,
                                     mode_, tc,
                                     (pred.not_thru_pruning() || !directededge->not_thru()));
    adjacencylist_forward_->add(idx);
    *es = {EdgeSet::kTemporary, idx};
  }

  // Handle transitions - expand from the end node of each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      if (trans->up()) {
        hierarchy_limits_forward_[node.level()].up_transition_count++;
        ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true);
      } else if (!hierarchy_limits_forward_[trans->endnode().level()].StopExpanding()) {
        ExpandForward(graphreader, trans->endnode(), pred, pred_idx, true);
      }
    }
  }
}

// Expand from a node in reverse direction.
void BidirectionalAStar::ExpandReverse(GraphReader& graphreader,
                                       const GraphId& node,
                                       const BDEdgeLabel& pred,
                                       const uint32_t pred_idx,
                                       const DirectedEdge* opp_pred_edge,
                                       const bool from_transition) {
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return;
  }
  const NodeInfo* nodeinfo = tile->node(node);
  if (!costing_->Allowed(nodeinfo)) {
    return;
  }

  // Expand from end node in reverse direction.
  uint32_t shortcuts = 0;
  GraphId edgeid = {node.tileid(), node.level(), nodeinfo->edge_index()};
  EdgeStatusInfo* es = edgestatus_reverse_.GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(edgeid);
  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++directededge, ++edgeid, ++es) {
    // Skip shortcut edges until we have stopped expanding on the next level. Use regular
    // edges while still expanding on the next level since we can still transition down to
    // that level. If using a shortcut, set the shortcuts mask. Skip if this is a regular
    // edge superseded by a shortcut.
    if (directededge->is_shortcut()) {
      if (hierarchy_limits_reverse_[edgeid.level() + 1].StopExpanding()) {
        shortcuts |= directededge->shortcut();
      } else {
        continue;
      }
    } else if (shortcuts & directededge->superseded()) {
      continue;
    }

    // Skip this edge if permanently labeled (best path already found to this
    // directed edge) or if no access for this mode.
    if (es->set() == EdgeSet::kPermanent || !(directededge->reverseaccess() & access_mode_)) {
      continue;
    }

    // Get end node tile, opposing edge Id, and opposing directed edge.
    const GraphTile* t2 =
        directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
    if (t2 == nullptr) {
      continue;
    }
    GraphId oppedge = t2->GetOpposingEdgeId(directededge);
    const DirectedEdge* opp_edge = t2->directededge(oppedge);

    // Skip this edge if no access is allowed (based on costing method)
    // or if a complex restriction prevents transition onto this edge.
    if (!costing_->AllowedReverse(directededge, pred, opp_edge, t2, oppedge, 0, 0) ||
        costing_->Restricted(directededge, pred, edgelabels_reverse_, tile, edgeid, false)) {
      continue;
    }

    // Get cost. Use opposing edge for EdgeCost. Separate the transition seconds so we
    // can properly recover elapsed time on the reverse path.
    Cost tc = costing_->TransitionCostReverse(directededge->localedgeidx(), nodeinfo, opp_edge,
                                              opp_pred_edge);
    Cost newcost = pred.cost() + costing_->EdgeCost(opp_edge, tile->GetSpeed(opp_edge));
    newcost.cost += tc.cost;

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (es->set() != EdgeSet::kUnreached) {
      BDEdgeLabel& lab = edgelabels_reverse_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_reverse_->decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost, tc);
      }
      continue;
    }

    // Find the sort cost (with A* heuristic) using the lat,lng at the
    // end node of the directed edge.
    float dist = 0.0f;
    float sortcost =
        newcost.cost + astarheuristic_reverse_.Get(t2->get_node_ll(directededge->endnode()), dist);

    // Add edge label, add to the adjacency list and set edge status
    uint32_t idx = edgelabels_reverse_.size();
    edgelabels_reverse_.emplace_back(pred_idx, edgeid, oppedge, directededge, newcost, sortcost, dist,
                                     mode_, tc,
                                     (pred.not_thru_pruning() || !directededge->not_thru()));
    adjacencylist_reverse_->add(idx);
    *es = {EdgeSet::kTemporary, idx};
  }

  // Handle transitions - expand from the end node of each transition
  if (!from_transition && nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      if (trans->up()) {
        hierarchy_limits_reverse_[node.level()].up_transition_count++;
        ExpandReverse(graphreader, trans->endnode(), pred, pred_idx, opp_pred_edge, true);
      } else if (!hierarchy_limits_reverse_[trans->endnode().level()].StopExpanding()) {
        ExpandReverse(graphreader, trans->endnode(), pred, pred_idx, opp_pred_edge, true);
      }
    }
  }
}

// Calculate best path using bi-directional A*. No hierarchies or time
// dependencies are used. Suitable for pedestrian routes (and bicycle?).
std::vector<PathInfo>
BidirectionalAStar::GetBestPath(odin::Location& origin,
                                odin::Location& destination,
                                GraphReader& graphreader,
                                const std::shared_ptr<DynamicCost>* mode_costing,
                                const sif::TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();
  access_mode_ = costing_->access_mode();

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  PointLL origin_new(origin.path_edges(0).ll().lng(), origin.path_edges(0).ll().lat());
  PointLL destination_new(destination.path_edges(0).ll().lng(), destination.path_edges(0).ll().lat());
  Init(origin_new, destination_new);

  // Set origin and destination locations - seeds the adj. lists
  // Note: because we can correlate to more than one place for a given
  // PathLocation using edges.front here means we are only setting the
  // heuristics to one of them alternate paths using the other correlated
  // points to may be harder to find
  SetOrigin(graphreader, origin);
  SetDestination(graphreader, destination);

  // Find shortest path. Switch between a forward direction and a reverse
  // direction search based on the current costs. Alternating like this
  // prevents one tree from expanding much more quickly (if in a sparser
  // portion of the graph) rather than strictly alternating.
  // TODO - CostMatrix alternates, maybe should try alternating here?
  int n = 1;
  uint32_t forward_pred_idx, reverse_pred_idx;
  BDEdgeLabel fwd_pred, rev_pred;
  bool expand_forward = true;
  bool expand_reverse = true;
  while (true) {
    // Allow this process to be aborted
    if (interrupt && (n % kInterruptIterationsInterval) == 0) {
      (*interrupt)();
    }
    n++;

    // Get the next predecessor (based on which direction was expanded in prior step)
    if (expand_forward) {
      forward_pred_idx = adjacencylist_forward_->pop();
      if (forward_pred_idx != kInvalidLabel) {
        fwd_pred = edgelabels_forward_[forward_pred_idx];

        // Terminate if the cost threshold has been exceeded.
        if (fwd_pred.sortcost() + cost_diff_ > threshold_) {
          return FormPath(graphreader);
        }

        // Check if the edge on the forward search connects to a settled edge on the
        // reverse search tree. Do not expand further past this edge since it will just
        // result in other connections.
        if (edgestatus_reverse_.Get(fwd_pred.opp_edgeid()).set() == EdgeSet::kPermanent) {
          if (SetForwardConnection(fwd_pred)) {
            continue;
          }
        }
      } else {
        // Search is exhausted. If a connection has been found, return it
        if (best_connection_.cost < std::numeric_limits<float>::max()) {
          return FormPath(graphreader);
        } else {
          // No route found.
          LOG_ERROR("Bi-directional route failure - forward search exhausted: n = " +
                    std::to_string(edgelabels_forward_.size()) + "," +
                    std::to_string(edgelabels_reverse_.size()));
          return {};
        }
      }
    }
    if (expand_reverse) {
      reverse_pred_idx = adjacencylist_reverse_->pop();
      if (reverse_pred_idx != kInvalidLabel) {
        rev_pred = edgelabels_reverse_[reverse_pred_idx];

        // Terminate if the cost threshold has been exceeded.
        if (rev_pred.sortcost() > threshold_) {
          return FormPath(graphreader);
        }

        // Check if the edge on the reverse search connects to a settled edge on the
        // forward search tree. Do not expand further past this edge since it will just
        // result in other connections.
        if (edgestatus_forward_.Get(rev_pred.opp_edgeid()).set() == EdgeSet::kPermanent) {
          if (SetReverseConnection(rev_pred)) {
            continue;
          }
        }
      } else {
        // Search is exhausted. If a connection has been found, return it
        if (best_connection_.cost < std::numeric_limits<float>::max()) {
          return FormPath(graphreader);
        } else {
          // No route found.
          LOG_ERROR("Bi-directional route failure - reverse search exhausted: n = " +
                    std::to_string(edgelabels_reverse_.size()) + "," +
                    std::to_string(edgelabels_forward_.size()));
          return {};
        }
      }
    }

    // Expand from the search direction with lower sort cost.
    if ((fwd_pred.sortcost() + cost_diff_) < rev_pred.sortcost()) {
      // Expand forward - set to get next edge from forward adj. list on the next pass
      expand_forward = true;
      expand_reverse = false;

      // Settle this edge.
      edgestatus_forward_.Update(fwd_pred.edgeid(), EdgeSet::kPermanent);

      // Prune path if predecessor is not a through edge or if the maximum
      // number of upward transitions has been exceeded on this hierarchy level.
      if ((fwd_pred.not_thru() && fwd_pred.not_thru_pruning()) ||
          hierarchy_limits_forward_[fwd_pred.endnode().level()].StopExpanding()) {
        continue;
      }

      // Expand from the end node in forward direction.
      ExpandForward(graphreader, fwd_pred.endnode(), fwd_pred, forward_pred_idx, false);
    } else {
      // Expand reverse - set to get next edge from reverse adj. list on the next pass
      expand_forward = false;
      expand_reverse = true;

      // Settle this edge
      edgestatus_reverse_.Update(rev_pred.edgeid(), EdgeSet::kPermanent);

      // Prune path if predecessor is not a through edge
      if ((rev_pred.not_thru() && rev_pred.not_thru_pruning()) ||
          hierarchy_limits_reverse_[rev_pred.endnode().level()].StopExpanding()) {
        continue;
      }

      // Get the opposing predecessor directed edge. Need to make sure we get
      // the correct one if a transition occurred
      const DirectedEdge* opp_pred_edge =
          graphreader.GetGraphTile(rev_pred.opp_edgeid())->directededge(rev_pred.opp_edgeid());

      // Expand from the end node in reverse direction.
      ExpandReverse(graphreader, rev_pred.endnode(), rev_pred, reverse_pred_idx, opp_pred_edge,
                    false);
    }
  }
  return {}; // If we are here the route failed
}

// The edge on the forward search connects to a reached edge on the reverse
// search tree. Check if this is the best connection so far and set the
// search threshold.
bool BidirectionalAStar::SetForwardConnection(const BDEdgeLabel& pred) {
  // Disallow connections that are part of a complex restriction.
  // TODO - validate that we do not need to "walk" the paths forward
  // and backward to see if they match a restriction.
  if (pred.on_complex_rest()) {
    return false;
  }

  // Get the opposing edge - a candidate shortest path has been found to the
  // end node of this directed edge. Get total cost.
  float c;
  GraphId oppedge = pred.opp_edgeid();
  EdgeStatusInfo oppedgestatus = edgestatus_reverse_.Get(oppedge);
  if (pred.predecessor() != kInvalidLabel) {
    // Get the start of the predecessor edge on the forward path. Cost is to
    // the end this edge, plus the cost to the end of the reverse predecessor,
    // plus the transition cost.
    c = edgelabels_forward_[pred.predecessor()].cost().cost +
        edgelabels_reverse_[oppedgestatus.index()].cost().cost + pred.transition_cost();
  } else {
    // If no predecessor on the forward path get the predecessor on
    // the reverse path to form the cost.
    uint32_t predidx = edgelabels_reverse_[oppedgestatus.index()].predecessor();
    float oppcost = (predidx == kInvalidLabel) ? 0 : edgelabels_reverse_[predidx].cost().cost;
    c = pred.cost().cost + oppcost + edgelabels_reverse_[oppedgestatus.index()].transition_cost();
  }

  // Set best_connection if cost is less than the best cost so far.
  if (c < best_connection_.cost) {
    best_connection_ = {pred.edgeid(), oppedge, c};
  }

  // Set a threshold to extend search
  if (threshold_ == std::numeric_limits<float>::max()) {
    threshold_ = pred.sortcost() + cost_diff_ + kThresholdDelta;
  }
  return true;
}

// The edge on the reverse search connects to a reached edge on the forward
// search tree. Check if this is the best connection so far and set the
// search threshold.
bool BidirectionalAStar::SetReverseConnection(const BDEdgeLabel& pred) {
  // Disallow connections that are part of a complex restriction.
  // TODO - validate that we do not need to "walk" the paths forward
  // and backward to see if they match a restriction.
  if (pred.on_complex_rest()) {
    return false;
  }

  // Get the opposing edge - a candidate shortest path has been found to the
  // end node of this directed edge. Get total cost.
  float c;
  GraphId oppedge = pred.opp_edgeid();
  EdgeStatusInfo oppedgestatus = edgestatus_forward_.Get(oppedge);
  if (pred.predecessor() != kInvalidLabel) {
    // Get the start of the predecessor edge on the reverse path. Cost is to
    // the end this edge, plus the cost to the end of the forward predecessor,
    // plus the transition cost.
    c = edgelabels_reverse_[pred.predecessor()].cost().cost +
        edgelabels_forward_[oppedgestatus.index()].cost().cost + pred.transition_cost();
  } else {
    // If no predecessor on the reverse path get the predecessor on
    // the forward path to form the cost.
    uint32_t predidx = edgelabels_forward_[oppedgestatus.index()].predecessor();
    float oppcost = (predidx == kInvalidLabel) ? 0 : edgelabels_forward_[predidx].cost().cost;
    c = pred.cost().cost + oppcost + edgelabels_forward_[oppedgestatus.index()].transition_cost();
  }

  // Set best_connection if cost is less than the best cost so far.
  if (c < best_connection_.cost) {
    best_connection_ = {oppedge, pred.edgeid(), c};
  }

  // Set a threshold to extend search
  if (threshold_ == std::numeric_limits<float>::max()) {
    threshold_ = pred.sortcost() + kThresholdDelta;
  }
  return true;
}

// Add edges at the origin to the forward adjacency list.
void BidirectionalAStar::SetOrigin(GraphReader& graphreader, odin::Location& origin) {
  // Only skip inbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(origin.path_edges().begin(), origin.path_edges().end(),
                [&has_other_edges](const odin::Location::PathEdge& e) {
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

    // Get the directed edge
    GraphId edgeid(edge.graph_id());
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
    Cost cost = costing_->EdgeCost(directededge, tile->GetSpeed(directededge)) *
                (1.0f - edge.percent_along());

    // Store the closest node info
    if (closest_ni == nullptr) {
      closest_ni = nodeinfo;
    }

    // We need to penalize this location based on its score (distance in meters from input)
    // We assume the slowest speed you could travel to cover that distance to start/end the route
    // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
    cost.cost += edge.distance();
    float dist = astarheuristic_forward_.GetDistance(nodeinfo->latlng(endtile->header()->base_ll()));
    float sortcost = cost.cost + astarheuristic_forward_.Get(dist);

    // Add EdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path.
    uint32_t idx = edgelabels_forward_.size();
    edgestatus_forward_.Set(edgeid, EdgeSet::kTemporary, idx, tile);
    edgelabels_forward_.emplace_back(kInvalidLabel, edgeid, directededge, cost, sortcost, dist,
                                     mode_);
    adjacencylist_forward_->add(idx);

    // Set the initial not_thru flag to false. There is an issue with not_thru
    // flags on small loops. Set this to false here to override this for now.
    edgelabels_forward_.back().set_not_thru(false);
  }

  // Set the origin timezone
  if (closest_ni != nullptr && origin.has_date_time() && origin.date_time() == "current") {
    origin.set_date_time(
        DateTime::iso_date_time(DateTime::get_tz_db().from_index(closest_ni->timezone())));
  }
}

// Add destination edges to the reverse path adjacency list.
void BidirectionalAStar::SetDestination(GraphReader& graphreader, const odin::Location& dest) {
  // Only skip outbound edges if we have other options
  bool has_other_edges = false;
  std::for_each(dest.path_edges().begin(), dest.path_edges().end(),
                [&has_other_edges](const odin::Location::PathEdge& e) {
                  has_other_edges = has_other_edges || !e.begin_node();
                });

  // Iterate through edges and add to adjacency list
  Cost c;
  for (const auto& edge : dest.path_edges()) {
    // If the destination is at a node, skip any outbound edges (so any
    // opposing inbound edges are not considered)
    if (has_other_edges && edge.begin_node()) {
      continue;
    }

    // Get the directed edge
    GraphId edgeid(edge.graph_id());
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Get the opposing directed edge, continue if we cannot get it
    GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid);
    if (!opp_edge_id.Is_Valid()) {
      continue;
    }
    const DirectedEdge* opp_dir_edge = graphreader.GetOpposingEdge(edgeid);

    // Get cost and sort cost (based on distance from endnode of this edge
    // to the origin. Make sure we use the reverse A* heuristic. Use the
    // directed edge for costing, as this is the forward direction along the
    // destination edge. Note that the end node of the opposing edge is in the
    // same tile as the directed edge.
    Cost cost = costing_->EdgeCost(directededge, tile->GetSpeed(directededge)) * edge.percent_along();

    // We need to penalize this location based on its score (distance in meters from input)
    // We assume the slowest speed you could travel to cover that distance to start/end the route
    // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
    cost.cost += edge.distance();
    float dist = astarheuristic_reverse_.GetDistance(tile->get_node_ll(opp_dir_edge->endnode()));
    float sortcost = cost.cost + astarheuristic_reverse_.Get(dist);

    // Add EdgeLabel to the adjacency list. Set the predecessor edge index
    // to invalid to indicate the origin of the path. Make sure the opposing
    // edge (edgeid) is set.
    uint32_t idx = edgelabels_reverse_.size();
    edgestatus_reverse_.Set(opp_edge_id, EdgeSet::kTemporary, idx,
                            graphreader.GetGraphTile(opp_edge_id));
    edgelabels_reverse_.emplace_back(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, sortcost,
                                     dist, mode_, c, false);
    adjacencylist_reverse_->add(idx);

    // Set the initial not_thru flag to false. There is an issue with not_thru
    // flags on small loops. Set this to false here to override this for now.
    edgelabels_reverse_.back().set_not_thru(false);
  }
}

// Form the path from the adjacency list.
std::vector<PathInfo> BidirectionalAStar::FormPath(GraphReader& graphreader) {
  // Get the indexes where the connection occurs.
  uint32_t idx1 = edgestatus_forward_.Get(best_connection_.edgeid).index();
  uint32_t idx2 = edgestatus_reverse_.Get(best_connection_.opp_edgeid).index();

  // Metrics (TODO - more accurate cost)
  uint32_t pathcost = edgelabels_forward_[idx1].cost().cost + edgelabels_reverse_[idx2].cost().cost;
  LOG_DEBUG("path_cost::" + std::to_string(pathcost));
  LOG_DEBUG("FormPath path_iterations::" + std::to_string(edgelabels_forward_.size()) + "," +
            std::to_string(edgelabels_reverse_.size()));

  // Work backwards on the forward path
  std::vector<PathInfo> path;
  for (auto edgelabel_index = idx1; edgelabel_index != kInvalidLabel;
       edgelabel_index = edgelabels_forward_[edgelabel_index].predecessor()) {
    const BDEdgeLabel& edgelabel = edgelabels_forward_[edgelabel_index];
    path.emplace_back(edgelabel.mode(), edgelabel.cost().secs, edgelabel.edgeid(), 0);

    // Check if this is a ferry
    if (edgelabel.use() == Use::kFerry) {
      has_ferry_ = true;
    }
  }

  // Reverse the list
  std::reverse(path.begin(), path.end());

  // Special case code if the last edge of the forward path is
  // the destination edge - update the elapsed time
  if (edgelabels_reverse_[idx2].predecessor() == kInvalidLabel) {
    if (path.size() > 1) {
      path.back().elapsed_time =
          path[path.size() - 2].elapsed_time + edgelabels_reverse_[idx2].cost().secs;
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
    const BDEdgeLabel& edgelabel = edgelabels_reverse_[edgelabel_index];
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
    path.emplace_back(edgelabel.mode(), secs, oppedge, 0);

    // Check if this is a ferry
    if (edgelabel.use() == Use::kFerry) {
      has_ferry_ = true;
    }

    // Update edgelabel_index and transition cost to apply at next iteration
    edgelabel_index = predidx;
    tc = edgelabel.transition_secs();
  }
  return path;
}

} // namespace thor
} // namespace valhalla
