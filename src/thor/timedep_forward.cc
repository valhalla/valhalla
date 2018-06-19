#include "baldr/datetime.h"
#include "midgard/logging.h"
#include "thor/timedep.h"
#include <algorithm>
#include <iostream> // TODO remove if not needed
#include <map>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

constexpr uint64_t kInitialEdgeLabelCount = 500000;

// Default constructor
TimeDepForward::TimeDepForward() : AStarPathAlgorithm() {
  mode_ = TravelMode::kDrive;
  travel_type_ = 0;
  adjacencylist_ = nullptr;
  max_label_count_ = std::numeric_limits<uint32_t>::max();
  origin_tz_index_ = 0;
}

// Destructor
TimeDepForward::~TimeDepForward() {
  Clear();
}

// Get the timezone at the origin.
int TimeDepForward::GetOriginTimezone(GraphReader& graphreader) {
  if (edgelabels_.size() == 0) {
    return -1;
  }
  GraphId node = edgelabels_[0].endnode();
  const GraphTile* tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return -1;
  }
  return tile->node(node)->timezone();
}

// Expand from the node along the forward search path. Immediately expands
// from the end node of any transition edge (so no transition edges are added
// to the adjacency list or EdgeLabel list). Does not expand transition
// edges if from_transition is false.
void TimeDepForward::ExpandForward(GraphReader& graphreader,
                                   const GraphId& node,
                                   const EdgeLabel& pred,
                                   const uint32_t pred_idx,
                                   const bool from_transition,
                                   uint64_t localtime,
                                   const odin::Location& destination,
                                   std::pair<int32_t, float>& best_path) {
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

  // Adjust for time zone (if different from timezone at the start).
  if (nodeinfo->timezone() != origin_tz_index_) {
    DateTime::timezone_diff(true, localtime, DateTime::get_tz_db().from_index(origin_tz_index_),
                            DateTime::get_tz_db().from_index(nodeinfo->timezone()));
  }

  // Expand from end node.
  uint32_t max_shortcut_length = static_cast<uint32_t>(pred.distance() * 0.5f);
  GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
  EdgeStatusInfo* es = edgestatus_.GetPtr(edgeid, tile);
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++directededge, ++edgeid, ++es) {
    // Handle transition edges - expand from the end node of the transition
    // (unless this is called from a transition).
    if (directededge->trans_up()) {
      if (!from_transition) {
        hierarchy_limits_[node.level()].up_transition_count++;
        ExpandForward(graphreader, directededge->endnode(), pred, pred_idx, true, localtime,
                      destination, best_path);
      }
      continue;
    } else if (directededge->trans_down()) {
      if (!from_transition &&
          !hierarchy_limits_[directededge->endnode().level()].StopExpanding(pred.distance())) {
        ExpandForward(graphreader, directededge->endnode(), pred, pred_idx, true, localtime,
                      destination, best_path);
      }
      continue;
    }

    // Skip shortcut edges for time dependent routes
    if (directededge->is_shortcut()) {
      continue;
    }

    // Skip this edge if permanently labeled (best path already found to this
    // directed edge), if no access is allowed to this edge (based on costing
    // method), or if a complex restriction exists.
    if (es->set() == EdgeSet::kPermanent ||
        !costing_->Allowed(directededge, pred, tile, edgeid, localtime, nodeinfo->timezone()) ||
        costing_->Restricted(directededge, pred, edgelabels_, tile, edgeid, true, localtime,
                             nodeinfo->timezone())) {
      continue;
    }

    // Compute the cost to the end of this edge
    Cost newcost = pred.cost() + costing_->EdgeCost(directededge, tile->GetSpeed(directededge)) +
                   costing_->TransitionCost(directededge, nodeinfo, pred);

    // If this edge is a destination, subtract the partial/remainder cost
    // (cost from the dest. location to the end of the edge).
    auto p = destinations_.find(edgeid);
    if (p != destinations_.end()) {
      // Subtract partial cost and time
      newcost -= p->second;

      // Find the destination edge and update cost to include the edge score.
      // Note - with high edge scores the convergence test fails some routes
      // so reduce the edge score.
      for (const auto& destination_edge : destination.path_edges()) {
        if (destination_edge.graph_id() == edgeid) {
          newcost.cost += destination_edge.distance();
        }
      }
      newcost.cost = std::max(0.0f, newcost.cost);

      // Mark this as the best connection if that applies. This allows
      // a path to be formed even if the convergence test fails (can
      // happen with large edge scores)
      if (best_path.first == -1 || newcost.cost < best_path.second) {
        best_path.first = (es->set() == EdgeSet::kTemporary) ? es->index() : edgelabels_.size();
        best_path.second = newcost.cost;
      }
    }

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (es->set() == EdgeSet::kTemporary) {
      EdgeLabel& lab = edgelabels_[es->index()];
      if (newcost.cost < lab.cost().cost) {
        float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
        adjacencylist_->decrease(es->index(), newsortcost);
        lab.Update(pred_idx, newcost, newsortcost);
      }
      continue;
    }

    // If this is a destination edge the A* heuristic is 0. Otherwise the
    // sort cost (with A* heuristic) is found using the lat,lng at the
    // end node of the directed edge.
    float dist = 0.0f;
    float sortcost = newcost.cost;
    if (p == destinations_.end()) {
      const GraphTile* t2 =
          directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : tile;
      if (t2 == nullptr) {
        continue;
      }
      sortcost += astarheuristic_.Get(t2->node(directededge->endnode())->latlng(), dist);
    }

    // Add to the adjacency list and edge labels.
    uint32_t idx = edgelabels_.size();
    edgelabels_.emplace_back(pred_idx, edgeid, directededge, newcost, sortcost, dist, mode_, 0);
    *es = {EdgeSet::kTemporary, idx};
    adjacencylist_->add(idx);
  }
}

// Calculate time-dependent best path using a forward search. Supports
// "depart-at" routes.
std::vector<PathInfo> TimeDepForward::GetBestPath(odin::Location& origin,
                                                  odin::Location& destination,
                                                  GraphReader& graphreader,
                                                  const std::shared_ptr<DynamicCost>* mode_costing,
                                                  const TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();

  // date_time must be set on the origin.
  if (!origin.has_date_time()) {
    LOG_ERROR("TimeDepForward called without time set on the origin location");
    return {};
  }

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  // Note: because we can correlate to more than one place for a given PathLocation
  // using edges.front here means we are only setting the heuristics to one of them
  // alternate paths using the other correlated points to may be harder to find
  PointLL origin_new(origin.path_edges(0).ll().lng(), origin.path_edges(0).ll().lat());
  PointLL destination_new(destination.path_edges(0).ll().lng(), destination.path_edges(0).ll().lat());
  Init(origin_new, destination_new);
  float mindist = astarheuristic_.GetDistance(origin_new);

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  uint32_t density = SetDestination(graphreader, destination);
  SetOrigin(graphreader, origin, destination);

  // Set the origin timezone to be the timezone at the end node
  origin_tz_index_ = GetOriginTimezone(graphreader);

  // Set route start time (seconds from epoch)
  uint64_t start_time =
      DateTime::seconds_since_epoch(origin.date_time(),
                                    DateTime::get_tz_db().from_index(origin_tz_index_));
  // Update hierarchy limits
  ModifyHierarchyLimits(mindist, density);

  // Find shortest path
  uint32_t nc = 0; // Count of iterations with no convergence
                   // towards destination
  std::pair<int32_t, float> best_path = std::make_pair(-1, 0.0f);
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

    // Abort if max label count is exceeded
    if (total_labels > max_label_count_) {
      return {};
    }

    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->pop();
    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " + std::to_string(edgelabels_.size()));
      return {};
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
      edgestatus_.Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Check that distance is converging towards the destination. Return route
    // failure if no convergence for TODO iterations
    float dist2dest = pred.distance();
    if (dist2dest < mindist) {
      mindist = dist2dest;
      nc = 0;
    } else if (nc++ > 50000) {
      if (best_path.first >= 0) {
        return FormPath(best_path.first);
      } else {
        LOG_ERROR("No convergence to destination after = " + std::to_string(edgelabels_.size()));
        return {};
      }
    }

    // Do not expand based on hierarchy level based on number of upward
    // transitions and distance to the destination
    if (hierarchy_limits_[pred.endnode().level()].StopExpanding(dist2dest)) {
      continue;
    }

    // Set local time.
    uint64_t localtime = start_time + (double)pred.cost().secs;

    // Expand forward from the end node of the predecessor edge.
    ExpandForward(graphreader, pred.endnode(), pred, predindex, false, localtime, destination,
                  best_path);
  }
  return {}; // Should never get here
}

} // namespace thor
} // namespace valhalla
