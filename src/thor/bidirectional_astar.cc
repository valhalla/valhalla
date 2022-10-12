#include "thor/bidirectional_astar.h"
#include "baldr/datetime.h"
#include "baldr/directededge.h"
#include "baldr/graphid.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "sif/edgelabel.h"
#include "sif/recost.h"
#include "thor/alternates.h"
#include <algorithm>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

constexpr uint32_t kInitialEdgeLabelCountBD = 1000000;

// Threshold (seconds) to extend search once the first connection has been found.
// TODO - this is currently set based on some exceptional cases (e.g. routes taking
// the PA Turnpike which have very long edges). Using a metric based on maximum edge
// cost creates large performance drops - so perhaps some other metric can be found?
constexpr float kThresholdDelta = 420.0f;

// Relative cost extension to find alternative routes. It's a multiplier that we apply
// to the optimal route cost in order to get a new cost threshold. This threshold indicates
// an upper bound value cost for alternative routes we're looking for. Due to the fact that
// we can't estimate route cost that goes through some particular edge very precisely, we
// can find alternatives with costs greater than the threshold.
constexpr float kAlternativeCostExtend = 1.2f;
// Maximum number of additional iterations allowed once the first connection has been found.
// For alternative routes we use bigger cost extension than in the case with one route. This
// may lead to a significant increase in the number of iterations (~time). So, we should limit
// iterations in order no to drop performance too much.
constexpr uint32_t kAlternativeIterationsDelta = 100000;

inline float find_percent_along(const valhalla::Location& location, const GraphId& edge_id) {
  for (const auto& e : location.correlation().edges()) {
    if (e.graph_id() == edge_id)
      return e.percent_along();
  }
  throw std::logic_error("Could not find candidate edge for the location");
}

} // namespace

namespace valhalla {
namespace thor {

// Default constructor
BidirectionalAStar::BidirectionalAStar(const boost::property_tree::ptree& config)
    : PathAlgorithm(config.get<uint32_t>("max_reserved_labels_count", kInitialEdgeLabelCountBD),
                    config.get<bool>("clear_reserved_memory", false)),
      extended_search_(config.get<bool>("extended_search", false)) {
  cost_threshold_ = 0;
  iterations_threshold_ = 0;
  desired_paths_count_ = 1;
  mode_ = travel_mode_t::kDrive;
  access_mode_ = kAutoAccess;
  travel_type_ = 0;
  cost_diff_ = 0.0f;
  pruning_disabled_at_origin_ = false;
  pruning_disabled_at_destination_ = false;
  ignore_hierarchy_limits_ = false;
}

// Destructor
BidirectionalAStar::~BidirectionalAStar() {
}

// Clear the temporary information generated during path construction.
void BidirectionalAStar::Clear() {
  auto reservation = clear_reserved_memory_ ? 0 : max_reserved_labels_count_;
  if (edgelabels_forward_.size() > reservation) {
    edgelabels_forward_.resize(reservation);
    edgelabels_forward_.shrink_to_fit();
  }
  if (edgelabels_reverse_.size() > reservation) {
    edgelabels_reverse_.resize(reservation);
    edgelabels_reverse_.shrink_to_fit();
  }
  edgelabels_forward_.clear();
  edgelabels_reverse_.clear();

  adjacencylist_forward_.clear();
  adjacencylist_reverse_.clear();
  edgestatus_forward_.clear();
  edgestatus_reverse_.clear();

  // Set the ferry flag to false
  has_ferry_ = false;
  // Set not thru pruning to true
  not_thru_pruning_ = true;
  // reset origin & destination pruning states
  pruning_disabled_at_origin_ = false;
  pruning_disabled_at_destination_ = false;
  ignore_hierarchy_limits_ = false;
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
  edgelabels_forward_.reserve(std::min(max_reserved_labels_count_, kInitialEdgeLabelCountBD));
  edgelabels_reverse_.reserve(std::min(max_reserved_labels_count_, kInitialEdgeLabelCountBD));

  // Construct adjacency list and initialize edge status lookup.
  // Set bucket size and cost range based on DynamicCost.
  const uint32_t bucketsize = costing_->UnitSize();
  const float range = kBucketCount * bucketsize;

  const float mincostf = astarheuristic_forward_.Get(origll);
  adjacencylist_forward_.reuse(mincostf, range, bucketsize, &edgelabels_forward_);
  const float mincostr = astarheuristic_reverse_.Get(destll);
  adjacencylist_reverse_.reuse(mincostr, range, bucketsize, &edgelabels_reverse_);

  edgestatus_forward_.clear();
  edgestatus_reverse_.clear();

  // Set the cost diff between forward and reverse searches (due to distance
  // approximator differences). This is used to "even" the forward and reverse
  // searches.
  cost_diff_ = mincostf - mincostr;

  // Initialize best connections as having none
  best_connections_ = {};

  // Set the cost threshold to the maximum float value. Once the initial connection is found
  // the threshold is set.
  cost_threshold_ = std::numeric_limits<float>::max();
  iterations_threshold_ = std::numeric_limits<uint32_t>::max();

  // Support for hierarchy transitions
  hierarchy_limits_forward_ = costing_->GetHierarchyLimits();
  hierarchy_limits_reverse_ = costing_->GetHierarchyLimits();
  bool ignore_forward_limits =
      std::all_of(hierarchy_limits_forward_.begin() + 1,
                  hierarchy_limits_forward_.begin() + TileHierarchy::levels().size(),
                  [](const HierarchyLimits& limits) {
                    return limits.max_up_transitions == kUnlimitedTransitions;
                  });
  bool ignore_reverse_limits =
      std::all_of(hierarchy_limits_reverse_.begin() + 1,
                  hierarchy_limits_reverse_.begin() + TileHierarchy::levels().size(),
                  [](const HierarchyLimits& limits) {
                    return limits.max_up_transitions == kUnlimitedTransitions;
                  });
  // Set this flag to 'true' if we can expand edges at all hierarchy levels without limits
  ignore_hierarchy_limits_ = ignore_forward_limits && ignore_reverse_limits;
}

// Runs in the inner loop of `Expand`, essentially evaluating if
// the edge described in `meta` should be placed on the stack
// as well as doing just that.
//
// Returns false if uturns are allowed.
// Returns true if we will expand or have expanded from this edge. In that case we disallow uturns.
// Some edges we won't expand from, but we will still put them on the adjacency list in order to
// connect the forward and reverse paths. In that case we return false to allow uturns only if this
// edge is a not-thru edge that will be pruned.
//
template <const ExpansionType expansion_direction>
inline bool BidirectionalAStar::ExpandInner(baldr::GraphReader& graphreader,
                                            const sif::BDEdgeLabel& pred,
                                            const baldr::DirectedEdge* opp_pred_edge,
                                            const baldr::NodeInfo* nodeinfo,
                                            const uint32_t pred_idx,
                                            const EdgeMetadata& meta,
                                            uint32_t& shortcuts,
                                            const graph_tile_ptr& tile,
                                            const baldr::TimeInfo& time_info) {
  // Skip if this is a regular edge superseded by a shortcut.
  if (shortcuts & meta.edge->superseded()) {
    return false;
  }

  graph_tile_ptr t2 = nullptr;
  baldr::GraphId opp_edge_id;
  const auto get_opp_edge_data = [&t2, &opp_edge_id, &graphreader, &meta, &tile]() {
    // Get end node tile, opposing edge Id, and opposing directed edge.
    t2 = meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
    if (t2 == nullptr) {
      return false;
    }

    opp_edge_id = t2->GetOpposingEdgeId(meta.edge);
    return true;
  };

  constexpr bool FORWARD = expansion_direction == ExpansionType::forward;
  auto& hierarchy_limits = FORWARD ? hierarchy_limits_forward_ : hierarchy_limits_reverse_;
  // Skip shortcut edges until we have stopped expanding on the next level. Use regular
  // edges while still expanding on the next level since we can still transition down to
  // that level. If using a shortcut, set the shortcuts mask.
  if (meta.edge->is_shortcut()) {
    // Skip shortcuts if hierarchy limits are disabled
    if (ignore_hierarchy_limits_ || !get_opp_edge_data())
      return false;

    const auto& opp_edgestatus = FORWARD ? edgestatus_reverse_ : edgestatus_forward_;
    const auto opp_edge_set = opp_edgestatus.Get(opp_edge_id).set();
    // Synchronize shortcuts for both directions. If this shortcut has been already
    // encountered on the opposing search we should do the same now: skip or traverse.
    if ((opp_edge_set != EdgeSet::kSkipped &&
         hierarchy_limits[meta.edge_id.level() + 1].StopExpanding(pred.distance())) ||
        opp_edge_set == EdgeSet::kPermanent || opp_edge_set == EdgeSet::kTemporary) {
      shortcuts |= meta.edge->shortcut();
    } else {
      // Mark this edge as "skipped".
      *meta.edge_status = {EdgeSet::kSkipped, 0};
      return false;
    }
  }

  // Skip this edge if edge is permanently labeled (best path already found
  // to this directed edge), if no access is allowed (based on costing method),
  // or if a complex restriction prevents transition onto this edge.
  if (meta.edge_status->set() == EdgeSet::kPermanent) {
    return true; // This is an edge we _could_ have expanded, so return true
  }

  const baldr::DirectedEdge* opp_edge = nullptr;

  if (!FORWARD) {
    // Check the access mode and skip this edge if access is not allowed in the reverse
    // direction. This avoids the (somewhat expensive) retrieval of the opposing directed
    // edge when no access is allowed in the reverse direction.
    if (!(meta.edge->reverseaccess() & access_mode_)) {
      return false;
    }

    if (t2 == nullptr && !get_opp_edge_data()) {
      return false;
    }

    opp_edge = t2->directededge(opp_edge_id);
  }

  // Skip this edge if no access is allowed (based on costing method)
  // or if a complex restriction prevents transition onto this edge.
  // if its not time dependent set to 0 for Allowed and Restricted methods below
  const uint64_t localtime = time_info.valid ? time_info.local_time : 0;
  uint8_t restriction_idx = -1;
  if (FORWARD) {
    // Why is is_dest false?
    // We have to consider next cases:
    //  1) At least one step of reverse search was done -> forward search will never reach the
    //  destination edge. 2) There were no steps of the reverse search -> the destination edge is a
    //  connection edge.
    // We can set is_dest incorrectly in the second case, but it is the rare case.
    // The result path will be correct, because there are cosing.Allowed calls inside recost_forward
    // function in second time.
    if (!costing_->Allowed(meta.edge, false, pred, tile, meta.edge_id, localtime,
                           time_info.timezone_index, restriction_idx) ||
        costing_->Restricted(meta.edge, pred, edgelabels_forward_, tile, meta.edge_id, true,
                             &edgestatus_forward_, localtime, time_info.timezone_index)) {
      return false;
    }
  } else {
    if (!costing_->AllowedReverse(meta.edge, pred, opp_edge, t2, opp_edge_id, localtime,
                                  time_info.timezone_index, restriction_idx) ||
        costing_->Restricted(meta.edge, pred, edgelabels_reverse_, tile, meta.edge_id, false,
                             &edgestatus_reverse_, localtime, time_info.timezone_index)) {
      return false;
    }
  }

  // Get cost
  uint8_t flow_sources;
  sif::Cost newcost =
      pred.cost() + (FORWARD ? costing_->EdgeCost(meta.edge, tile, time_info, flow_sources)
                             : costing_->EdgeCost(opp_edge, t2, time_info, flow_sources));

  // Separate out transition cost.
  sif::Cost transition_cost =
      FORWARD ? costing_->TransitionCost(meta.edge, nodeinfo, pred)
              : costing_->TransitionCostReverse(meta.edge->localedgeidx(), nodeinfo, opp_edge,
                                                opp_pred_edge,
                                                static_cast<bool>(flow_sources & kDefaultFlowMask),
                                                pred.internal_turn());
  newcost += transition_cost;

  // Check if edge is temporarily labeled and this path has less cost. If
  // less cost the predecessor is updated and the sort cost is decremented
  // by the difference in real cost (A* heuristic doesn't change)
  if (meta.edge_status->set() == EdgeSet::kTemporary) {
    BDEdgeLabel& lab = FORWARD ? edgelabels_forward_[meta.edge_status->index()]
                               : edgelabels_reverse_[meta.edge_status->index()];
    if (newcost.cost < lab.cost().cost) {
      float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
      if (FORWARD) {
        adjacencylist_forward_.decrease(meta.edge_status->index(), newsortcost);
      } else {
        adjacencylist_reverse_.decrease(meta.edge_status->index(), newsortcost);
      }
      lab.Update(pred_idx, newcost, newsortcost, transition_cost, restriction_idx);
    }
    // Returning true since this means we approved the edge
    return true;
  }

  // Get end node tile (skip if tile is not found) and opposing edge Id
  if (t2 == nullptr && !get_opp_edge_data())
    return false;

  // Find the sort cost (with A* heuristic) using the lat,lng at the
  // end node of the directed edge.
  float dist = 0.0f;
  float sortcost =
      newcost.cost + (FORWARD
                          ? astarheuristic_forward_.Get(t2->get_node_ll(meta.edge->endnode()), dist)
                          : astarheuristic_reverse_.Get(t2->get_node_ll(meta.edge->endnode()), dist));

  // not_thru_pruning_ is only set to false on the 2nd pass in route_action.
  bool thru = not_thru_pruning_ ? (pred.not_thru_pruning() || !meta.edge->not_thru()) : false;

  // Add edge label, add to the adjacency list and set edge status
  uint32_t idx = 0;
  if (FORWARD) {
    idx = edgelabels_forward_.size();
    if (hierarchy_limits_forward_[meta.edge_id.level()].max_up_transitions != kUnlimitedTransitions) {
      // Override distance to the destination with a distance from the origin.
      // It will be used by hierarchy limits
      dist = astarheuristic_reverse_.GetDistance(t2->get_node_ll(meta.edge->endnode()));
    }
    edgelabels_forward_.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, newcost,
                                     sortcost, dist, mode_, transition_cost, thru,
                                     (pred.closure_pruning() || !costing_->IsClosed(meta.edge, tile)),
                                     static_cast<bool>(flow_sources & kDefaultFlowMask),
                                     costing_->TurnType(pred.opp_local_idx(), nodeinfo, meta.edge),
                                     restriction_idx);
    adjacencylist_forward_.add(idx);
  } else {
    idx = edgelabels_reverse_.size();
    if (hierarchy_limits_reverse_[meta.edge_id.level()].max_up_transitions != kUnlimitedTransitions) {
      // Override distance to the origin with a distance from the destination.
      // It will be used by hierarchy limits
      dist = astarheuristic_forward_.GetDistance(t2->get_node_ll(meta.edge->endnode()));
    }
    edgelabels_reverse_.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, newcost,
                                     sortcost, dist, mode_, transition_cost, thru,
                                     (pred.closure_pruning() || !costing_->IsClosed(meta.edge, tile)),
                                     static_cast<bool>(flow_sources & kDefaultFlowMask),
                                     costing_->TurnType(meta.edge->localedgeidx(), nodeinfo, opp_edge,
                                                        opp_pred_edge),
                                     restriction_idx);
    adjacencylist_reverse_.add(idx);
  }

  *meta.edge_status = {EdgeSet::kTemporary, idx};

  // setting this edge as reached
  if (expansion_callback_) {
    expansion_callback_(graphreader, FORWARD ? meta.edge_id : opp_edge_id, "bidirectional_astar", "r",
                        pred.cost().secs, pred.path_distance(), pred.cost().cost);
  }

  // we've just added this edge to the queue, but we won't expand from it if it's a not-thru edge that
  // will be pruned. In that case we want to allow uturns.
  return !(pred.not_thru_pruning() && meta.edge->not_thru());
}

template <const ExpansionType expansion_direction>
bool BidirectionalAStar::Expand(baldr::GraphReader& graphreader,
                                const baldr::GraphId& node,
                                sif::BDEdgeLabel& pred,
                                const uint32_t pred_idx,
                                const baldr::DirectedEdge* opp_pred_edge,
                                const baldr::TimeInfo& time_info,
                                const bool invariant) {
  constexpr bool FORWARD = expansion_direction == ExpansionType::forward;
  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  graph_tile_ptr tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return false;
  }
  const NodeInfo* nodeinfo = tile->node(node);

  // Keep track of superseded edges
  uint32_t shortcuts = 0;

  // Update the time information even if time is invariant to account for timezones
  auto seconds_offset = invariant ? 0.f : pred.cost().secs;
  auto offset_time = FORWARD
                         ? time_info.forward(seconds_offset, static_cast<int>(nodeinfo->timezone()))
                         : time_info.reverse(seconds_offset, static_cast<int>(nodeinfo->timezone()));

  auto& edgestatus = FORWARD ? edgestatus_forward_ : edgestatus_reverse_;

  // If we encounter a node with an access restriction like a barrier we allow a uturn
  if (!costing_->Allowed(nodeinfo)) {
    const DirectedEdge* opp_edge = nullptr;
    const GraphId opp_edge_id = graphreader.GetOpposingEdgeId(pred.edgeid(), opp_edge, tile);
    // Mark the predecessor as a deadend to be consistent with how the
    // edgelabels are set when an *actual* deadend (i.e. some dangling OSM geometry)
    // is labelled
    pred.set_deadend(true);
    // Check if edge is null before using it (can happen with regional data sets)
    return opp_edge &&
           ExpandInner<expansion_direction>(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx,
                                            {opp_edge, opp_edge_id,
                                             edgestatus.GetPtr(opp_edge_id, tile)},
                                            shortcuts, tile, offset_time);
  }

  bool disable_uturn = false;
  EdgeMetadata meta = EdgeMetadata::make(node, nodeinfo, tile, edgestatus);
  EdgeMetadata uturn_meta{};

  // Expand from end node in <expansion_direction> direction.
  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++meta) {

    // Begin by checking if this is the opposing edge to pred.
    // If so, it means we are attempting a u-turn. In that case, lets wait with evaluating
    // this edge until last. If any other edges were emplaced, it means we should not
    // even try to evaluate a u-turn since u-turns should only happen for deadends
    uturn_meta = pred.opp_local_idx() == meta.edge->localedgeidx() ? meta : uturn_meta;

    // Expand but only if this isnt the uturn, we'll try that later if nothing else works out
    disable_uturn =
        (pred.opp_local_idx() != meta.edge->localedgeidx() &&
         ExpandInner<expansion_direction>(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx, meta,
                                          shortcuts, tile, offset_time)) ||
        disable_uturn;
  }

  // Handle transitions - expand from the end node of each transition
  if (nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    auto& hierarchy_limits = FORWARD ? hierarchy_limits_forward_ : hierarchy_limits_reverse_;
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      // if this is a downward transition (ups are always allowed) AND we are no longer allowed OR
      // we cant get the tile at that level (local extracts could have this problem) THEN bail
      graph_tile_ptr trans_tile = nullptr;
      if ((!trans->up() && !ignore_hierarchy_limits_ &&
           hierarchy_limits[trans->endnode().level()].StopExpanding(pred.distance())) ||
          !(trans_tile = graphreader.GetGraphTile(trans->endnode()))) {
        continue;
      }

      // setup for expansion at this level
      hierarchy_limits[node.level()].up_transition_count += trans->up();
      const auto* trans_node = trans_tile->node(trans->endnode());
      EdgeMetadata trans_meta =
          EdgeMetadata::make(trans->endnode(), trans_node, trans_tile, edgestatus);
      uint32_t trans_shortcuts = 0;
      // expand the edges from this node at this level
      for (uint32_t i = 0; i < trans_node->edge_count(); ++i, ++trans_meta) {
        disable_uturn =
            ExpandInner<expansion_direction>(graphreader, pred, opp_pred_edge, trans_node, pred_idx,
                                             trans_meta, trans_shortcuts, trans_tile, offset_time) ||
            disable_uturn;
      }
    }
  }

  // Now, after having looked at all the edges, including edges on other levels,
  // we can say if this is a deadend or not, and if so, evaluate the uturn-edge (if it exists)
  if (!disable_uturn && uturn_meta) {
    // If we found no suitable edge to add, it means we're at a deadend
    // so lets go back and re-evaluate a potential u-turn
    pred.set_deadend(true);

    // TODO Is there a shortcut that supersedes our u-turn?
    // Decide if we should expand a shortcut or the non-shortcut edge...

    // Expand the uturn possiblity
    disable_uturn =
        ExpandInner<expansion_direction>(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx,
                                         uturn_meta, shortcuts, tile, offset_time) ||
        disable_uturn;
  }

  return disable_uturn;
}

// Calculate best path using bi-directional A*. No hierarchies or time
// dependencies are used. Suitable for pedestrian routes (and bicycle?).
std::vector<std::vector<PathInfo>>
BidirectionalAStar::GetBestPath(valhalla::Location& origin,
                                valhalla::Location& destination,
                                GraphReader& graphreader,
                                const sif::mode_costing_t& mode_costing,
                                const sif::travel_mode_t mode,
                                const Options& options) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  travel_type_ = costing_->travel_type();
  access_mode_ = costing_->access_mode();

  desired_paths_count_ = 1;
  if (options.has_alternates_case() && options.alternates())
    desired_paths_count_ += options.alternates();

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  PointLL origin_new(origin.correlation().edges(0).ll().lng(),
                     origin.correlation().edges(0).ll().lat());
  PointLL destination_new(destination.correlation().edges(0).ll().lng(),
                          destination.correlation().edges(0).ll().lat());
  Init(origin_new, destination_new);

  // we use a non varying time for all time dependent routes until we can figure out how to vary the
  // time during the path computation in the bidirectional algorithm
  bool invariant = options.date_time_type() != Options::no_time;
  // Get time information for forward and backward searches
  auto forward_time_info = TimeInfo::make(origin, graphreader, &tz_cache_);
  auto reverse_time_info = TimeInfo::make(destination, graphreader, &tz_cache_);

  // When a timedependent route is too long in distance it gets sent to this algorithm. It used to be
  // the case that this algorithm called EdgeCost without a time component. This would result in
  // timedependent routes falling back to time independent routing. Now that this algorithm is time
  // aware we will be tracking time in one direction of the search. To revert to previous behavior
  // you can uncomment the code below and get a time independent route in the fallback scenario.
  //  if (!invariant) {
  //    auto o = origin; o.mutable_date_time()->clear();
  //    forward_time_info = TimeInfo::make(o, graphreader, &tz_cache_);
  //    auto d = destination; d.mutable_date_time()->clear();
  //    reverse_time_info = TimeInfo::make(d, graphreader, &tz_cache_);
  //  }

  // Set origin and destination locations - seeds the adj. lists
  // Note: because we can correlate to more than one place for a given
  // PathLocation using edges.front here means we are only setting the
  // heuristics to one of them alternate paths using the other correlated
  // points to may be harder to find
  SetOrigin(graphreader, origin, forward_time_info);
  SetDestination(graphreader, destination, reverse_time_info);

  // Update hierarchy limits
  if (!ignore_hierarchy_limits_)
    ModifyHierarchyLimits();

  // Find shortest path. Switch between a forward direction and a reverse
  // direction search based on the current costs. Alternating like this
  // prevents one tree from expanding much more quickly (if in a sparser
  // portion of the graph) rather than strictly alternating.
  // TODO - CostMatrix alternates, maybe should try alternating here?
  int n = 0;
  uint32_t forward_pred_idx, reverse_pred_idx;
  BDEdgeLabel fwd_pred, rev_pred;
  bool expand_forward = true;
  bool expand_reverse = true;
  while (true) {
    // Allow this process to be aborted
    if (interrupt && (++n % kInterruptIterationsInterval) == 0) {
      (*interrupt)();
    }

    // Terminate if the iterations threshold has been exceeded.
    if ((edgelabels_reverse_.size() + edgelabels_forward_.size()) > iterations_threshold_) {
      return FormPath(graphreader, options, origin, destination, forward_time_info);
    }

    // Get the next predecessor (based on which direction was expanded in prior step)
    if (expand_forward) {
      forward_pred_idx = adjacencylist_forward_.pop();
      if (forward_pred_idx != kInvalidLabel) {
        fwd_pred = edgelabels_forward_[forward_pred_idx];

        // Forward path to this edge can't be improved, so we can settle it right now.
        edgestatus_forward_.Update(fwd_pred.edgeid(), EdgeSet::kPermanent);

        // Terminate if the cost threshold has been exceeded.
        if (fwd_pred.sortcost() + cost_diff_ > cost_threshold_) {
          return FormPath(graphreader, options, origin, destination, forward_time_info);
        }

        // Check if the edge on the forward search connects to a settled edge on the
        // reverse search tree. Do not expand further past this edge since it will just
        // result in other connections. Handle special edge case when we encountered the
        // destination edge that wasn't still pulled out of the queue.
        const auto opp_status = edgestatus_reverse_.Get(fwd_pred.opp_edgeid());
        if (opp_status.set() == EdgeSet::kPermanent ||
            (opp_status.set() == EdgeSet::kTemporary &&
             edgelabels_reverse_[opp_status.index()].predecessor() == kInvalidLabel)) {
          if (SetForwardConnection(graphreader, fwd_pred) &&
              opp_status.set() == EdgeSet::kPermanent) {
            continue;
          }
        }
      } else {
        // Search is exhausted. If a connection has been found, return it
        if (!best_connections_.empty()) {
          return FormPath(graphreader, options, origin, destination, forward_time_info);
        }
        LOG_ERROR("Forward search exhausted: n = " + std::to_string(edgelabels_forward_.size()) +
                  "," + std::to_string(edgelabels_reverse_.size()));
        // Search might've exhausted if it hit a closure or not_thru edge leading upto the
        // destination. Instead of tracking if any of the other edges is within a not_thru/closure
        // region (indicated by the pruning state of the edge label), we simply check if we started
        // from the other end on a closed or not_thru edge. If either is true, we extend the search in
        // the other direction (if allowed by the config option "thor.extended_search")
        //
        // Caveat: This assumption is not true if for eg the search from other end has pruning turned
        // on later, causing us to needlessly expand when we could have aborted sooner. However, it
        // ensures that most impossible route will fail fast provided one of the locations didn't
        // start from a not_thru/closed edge
        if (!extended_search_ || !pruning_disabled_at_destination_) {
          return {};
        }
        LOG_DEBUG("Extending search in reverse direction. Destination pruning disabled? " +
                  std::to_string(pruning_disabled_at_destination_));
      }
    }
    if (expand_reverse) {
      reverse_pred_idx = adjacencylist_reverse_.pop();
      if (reverse_pred_idx != kInvalidLabel) {
        rev_pred = edgelabels_reverse_[reverse_pred_idx];

        // Reverse path to this edge can't be improved, so we can settle it right now.
        edgestatus_reverse_.Update(rev_pred.edgeid(), EdgeSet::kPermanent);

        // Terminate if the cost threshold has been exceeded.
        if (rev_pred.sortcost() > cost_threshold_) {
          return FormPath(graphreader, options, origin, destination, forward_time_info);
        }

        // Check if the edge on the reverse search connects to a settled edge on the
        // forward search tree. Do not expand further past this edge since it will just
        // result in other connections. Handle special edge case when we encountered the
        // destination edge that wasn't still pulled out of the queue.
        const auto opp_status = edgestatus_forward_.Get(rev_pred.opp_edgeid());
        if (opp_status.set() == EdgeSet::kPermanent ||
            (opp_status.set() == EdgeSet::kTemporary &&
             edgelabels_forward_[opp_status.index()].predecessor() == kInvalidLabel)) {
          if (SetReverseConnection(graphreader, rev_pred) &&
              opp_status.set() == EdgeSet::kPermanent) {
            continue;
          }
        }
      } else {
        // Search is exhausted. If a connection has been found, return it
        if (!best_connections_.empty()) {
          return FormPath(graphreader, options, origin, destination, forward_time_info);
        }
        LOG_ERROR("Reverse search exhausted: n = " + std::to_string(edgelabels_reverse_.size()) +
                  "," + std::to_string(edgelabels_forward_.size()));
        // Search might've exhausted if it hit a closure or not_thru edge leading upto the origin.
        // Instead of tracking if any of the other edges is within a not_thru/closure region
        // (indicated by the pruning state of the edge label), we simply check if we started from the
        // other end on a closed or not_thru edge. If either is true, we extend the search in the
        // other direction (if allowed by the config option "thor.extended_search")
        //
        // Caveat: This assumption is not true if for eg the search from other end has pruning turned
        // on later, causing us to needlessly expand when we could have aborted sooner. However, it
        // ensures that most impossible route will fail fast provided one of the locations didn't end
        // on a not_thru/closed edge
        if (!extended_search_ || !pruning_disabled_at_origin_) {
          return {};
        }
        LOG_DEBUG("Extending search in forward direction. Origin pruning disabled? " +
                  std::to_string(pruning_disabled_at_origin_));
      }
    }

    bool forward_exhausted = forward_pred_idx == kInvalidLabel;
    bool reverse_exhausted = reverse_pred_idx == kInvalidLabel;
    // If both directions have exhausted, we've failed to find a route. Abort
    if (forward_exhausted && reverse_exhausted) {
      LOG_ERROR("Bi-directional route failure - search exhausted: n = " +
                std::to_string(edgelabels_forward_.size()) + "," +
                std::to_string(edgelabels_reverse_.size()));
      return {};
    }

    // Exhaust hierarchy limits simultaneously in both directions. As soon as forward/reverse
    // search exhausts limits on the particular level - it should stop and wait until reverse/forward
    // search exhausts limits on the same hierarchy level. This logic ensures local optimality near
    // the origin and destination and provides valid conditions for the reach-based pruning.
    bool force_forward = false;
    bool force_reverse = false;
    if (!ignore_hierarchy_limits_) {
      for (size_t level = TileHierarchy::levels().size() - 1; level > 0; --level) {
        if (hierarchy_limits_reverse_[level].StopExpanding(rev_pred.distance()) &&
            !hierarchy_limits_forward_[level].StopExpanding(fwd_pred.distance())) {
          force_forward = true;
          break;
        } else if (hierarchy_limits_forward_[level].StopExpanding(fwd_pred.distance()) &&
                   !hierarchy_limits_reverse_[level].StopExpanding(rev_pred.distance())) {
          force_reverse = true;
          break;
        }
      }
    }

    // Expand from the search direction with lower sort cost
    // Note: If one direction is exhausted, we force search in the remaining
    // direction
    if (!forward_exhausted &&
        ((!force_reverse && (fwd_pred.sortcost() + cost_diff_) < rev_pred.sortcost()) ||
         force_forward || reverse_exhausted)) {
      // Expand forward - set to get next edge from forward adj. list on the next pass
      expand_forward = true;
      expand_reverse = false;

      // setting this edge as settled
      if (expansion_callback_) {
        expansion_callback_(graphreader, fwd_pred.edgeid(), "bidirectional_astar", "s",
                            fwd_pred.cost().secs, fwd_pred.path_distance(), fwd_pred.cost().cost);
      }

      // Prune path if predecessor is not a through edge or if the maximum
      // number of upward transitions has been exceeded on this hierarchy level.
      if ((fwd_pred.not_thru() && fwd_pred.not_thru_pruning()) ||
          (!ignore_hierarchy_limits_ &&
           hierarchy_limits_forward_[fwd_pred.endnode().level()].StopExpanding(
               fwd_pred.distance()))) {
        continue;
      }

      // Check if this branch can be pruned. It's implementation of the reach-based pruning technique
      // for bidirectional astar: https://repub.eur.nl/pub/16100/ei2009-10.pdf .
      if (cost_threshold_ != std::numeric_limits<float>::max() &&
          fwd_pred.predecessor() != kInvalidLabel) {
        const auto tile = graphreader.GetGraphTile(fwd_pred.endnode());
        if (tile != nullptr) {
          // Estimate lower bound cost for the shortest path that goes through the current edge.
          float route_lower_bound =
              edgelabels_forward_[fwd_pred.predecessor()].cost().cost +
              fwd_pred.transition_cost().cost + rev_pred.sortcost() -
              astarheuristic_reverse_.Get(tile->get_node_ll(fwd_pred.endnode()));
          // Prune this edge if estimated lower bound cost exceeds the cost threshold.
          if (route_lower_bound > cost_threshold_) {
            continue;
          }
        } else {
          // Failed to get tile for the endnode. Skip expansion from this node.
          continue;
        }
      }

      // Expand from the end node in forward direction.
      Expand<ExpansionType::forward>(graphreader, fwd_pred.endnode(), fwd_pred, forward_pred_idx,
                                     nullptr, forward_time_info, invariant);
    } else {
      // Expand reverse - set to get next edge from reverse adj. list on the next pass
      expand_forward = false;
      expand_reverse = true;

      // setting this edge as settled, sending the opposing because this is the reverse tree
      if (expansion_callback_) {
        expansion_callback_(graphreader, rev_pred.opp_edgeid(), "bidirectional_astar", "s",
                            rev_pred.cost().secs, rev_pred.path_distance(), rev_pred.cost().cost);
      }

      // Prune path if predecessor is not a through edge
      if ((rev_pred.not_thru() && rev_pred.not_thru_pruning()) ||
          (!ignore_hierarchy_limits_ &&
           hierarchy_limits_reverse_[rev_pred.endnode().level()].StopExpanding(
               rev_pred.distance()))) {
        continue;
      }

      // Get the opposing predecessor directed edge. Need to make sure we get
      // the correct one if a transition occurred
      const auto rev_pred_tile = graphreader.GetGraphTile(rev_pred.opp_edgeid());
      if (rev_pred_tile == nullptr) {
        continue;
      }
      const DirectedEdge* opp_pred_edge = rev_pred_tile->directededge(rev_pred.opp_edgeid());

      // Check if this branch can be pruned. It's implementation of the reach-based pruning technique
      // for bidirectional astar: https://repub.eur.nl/pub/16100/ei2009-10.pdf .
      if (cost_threshold_ != std::numeric_limits<float>::max() &&
          rev_pred.predecessor() != kInvalidLabel) {
        const auto tile = graphreader.GetGraphTile(rev_pred.endnode());
        if (tile != nullptr) {
          // Estimate lower bound cost for the shortest path that goes through the current edge.
          float route_lower_bound =
              edgelabels_reverse_[rev_pred.predecessor()].cost().cost +
              rev_pred.transition_cost().cost + fwd_pred.sortcost() -
              astarheuristic_forward_.Get(tile->get_node_ll(rev_pred.endnode()));
          // Prune this edge if estimated lower bound cost exceeds the cost threshold.
          if (route_lower_bound > cost_threshold_) {
            continue;
          }
        } else {
          // Failed to get tile for the endnode. Skip expansion from this node.
          continue;
        }
      }

      // Expand from the end node in reverse direction.
      Expand<ExpansionType::reverse>(graphreader, rev_pred.endnode(), rev_pred, reverse_pred_idx,
                                     opp_pred_edge, reverse_time_info, invariant);
    }
  }
  return {}; // If we are here the route failed
}

// The edge on the forward search connects to a reached edge on the reverse
// search tree. Check if this is the best connection so far and set the
// search threshold.
bool BidirectionalAStar::SetForwardConnection(GraphReader& graphreader, const BDEdgeLabel& pred) {
  // Find pred on opposite side
  GraphId oppedge = pred.opp_edgeid();
  EdgeStatusInfo oppedgestatus = edgestatus_reverse_.Get(oppedge);
  auto opp_pred = edgelabels_reverse_[oppedgestatus.index()];

  // Disallow connections that are part of an uturn on an internal edge
  if (pred.internal_turn() != InternalTurn::kNoTurn) {
    return false;
  }

  // Disallow connections that are part of a complex restriction
  if (pred.on_complex_rest()) {
    // Lets dig deeper and test if we are really triggering these restrictions
    // since the complex restriction can span many edges
    if (IsBridgingEdgeRestricted(graphreader, edgelabels_forward_, edgelabels_reverse_, pred,
                                 opp_pred, costing_)) {
      return false;
    }
  }

  // Get the opposing edge - a candidate shortest path has been found to the
  // end node of this directed edge. Get total cost.
  float c;
  if (pred.predecessor() != kInvalidLabel) {
    // Get the start of the predecessor edge on the forward path. Cost is to
    // the end this edge, plus the cost to the end of the reverse predecessor,
    // plus the transition cost.
    c = edgelabels_forward_[pred.predecessor()].cost().cost + opp_pred.cost().cost +
        pred.transition_cost().cost;
  } else {
    // If no predecessor on the forward path get the predecessor on
    // the reverse path to form the cost.
    uint32_t predidx = opp_pred.predecessor();
    float oppcost = (predidx == kInvalidLabel) ? 0 : edgelabels_reverse_[predidx].cost().cost;
    c = pred.cost().cost + oppcost + opp_pred.transition_cost().cost;
  }

  // Set thresholds to extend search
  if (cost_threshold_ == std::numeric_limits<float>::max() || c < best_connections_.front().cost) {
    if (desired_paths_count_ == 1) {
      cost_threshold_ = c + kThresholdDelta;
    } else {
      // For short routes it may be not enough to use just scale to extend the cost threshold.
      // So, we also add the delta to find more alternatives.
      // TODO: use different constants to extend the search based on route distance.
      cost_threshold_ = kAlternativeCostExtend * c + kThresholdDelta;
      iterations_threshold_ =
          edgelabels_forward_.size() + edgelabels_reverse_.size() + kAlternativeIterationsDelta;
    }
  }

  // Keep the best ones at the front all others to the back
  best_connections_.emplace_back(CandidateConnection{pred.edgeid(), oppedge, c});

  if (c < best_connections_.front().cost)
    std::swap(best_connections_.front(), best_connections_.back());

  // setting this edge as connected
  if (expansion_callback_) {
    expansion_callback_(graphreader, pred.edgeid(), "bidirectional_astar", "c", pred.cost().secs,
                        pred.path_distance(), pred.cost().cost);
  }

  return true;
}

// The edge on the reverse search connects to a reached edge on the forward
// search tree. Check if this is the best connection so far and set the
// search threshold.
bool BidirectionalAStar::SetReverseConnection(GraphReader& graphreader, const BDEdgeLabel& rev_pred) {
  GraphId fwd_edge_id = rev_pred.opp_edgeid();
  EdgeStatusInfo fwd_edge_status = edgestatus_forward_.Get(fwd_edge_id);
  auto fwd_pred = edgelabels_forward_[fwd_edge_status.index()];

  // Disallow connections that are part of an uturn on an internal edge
  if (rev_pred.internal_turn() != InternalTurn::kNoTurn) {
    return false;
  }

  // Disallow connections that are part of a complex restriction
  if (rev_pred.on_complex_rest()) {
    // Lets dig deeper and test if we are really triggering these restrictions
    // since the complex restriction can span many edges
    if (IsBridgingEdgeRestricted(graphreader, edgelabels_forward_, edgelabels_reverse_, fwd_pred,
                                 rev_pred, costing_)) {
      return false;
    }
  }

  // Get the opposing edge - a candidate shortest path has been found to the
  // end node of this directed edge. Get total cost.
  float c;
  if (rev_pred.predecessor() != kInvalidLabel) {
    // Get the start of the predecessor edge on the reverse path. Cost is to
    // the end this edge, plus the cost to the end of the forward predecessor,
    // plus the transition cost.
    c = edgelabels_reverse_[rev_pred.predecessor()].cost().cost + fwd_pred.cost().cost +
        rev_pred.transition_cost().cost;
  } else {
    // If no predecessor on the reverse path get the predecessor on
    // the forward path to form the cost.
    uint32_t predidx = fwd_pred.predecessor();
    float oppcost = (predidx == kInvalidLabel) ? 0 : edgelabels_forward_[predidx].cost().cost;
    c = rev_pred.cost().cost + oppcost + fwd_pred.transition_cost().cost;
  }

  // Set thresholds to extend search
  if (cost_threshold_ == std::numeric_limits<float>::max() || c < best_connections_.front().cost) {
    if (desired_paths_count_ == 1) {
      cost_threshold_ = c + kThresholdDelta;
    } else {
      // For short routes it may be not enough to use just scale to extend the cost threshold.
      // So, we also add the delta to find more alternatives.
      // TODO: use different constants to extend the search based on route distance.
      cost_threshold_ = kAlternativeCostExtend * c + kThresholdDelta;
      iterations_threshold_ =
          edgelabels_forward_.size() + edgelabels_reverse_.size() + kAlternativeIterationsDelta;
    }
  }

  // Keep the best ones at the front all others to the back
  best_connections_.emplace_back(CandidateConnection{fwd_edge_id, rev_pred.edgeid(), c});

  if (c < best_connections_.front().cost)
    std::swap(best_connections_.front(), best_connections_.back());

  // setting this edge as connected, sending the opposing because this is the reverse tree
  if (expansion_callback_) {
    expansion_callback_(graphreader, fwd_edge_id, "bidirectional_astar", "c", fwd_pred.cost().secs,
                        fwd_pred.path_distance(), fwd_pred.cost().cost);
  }

  return true;
}

// Add edges at the origin to the forward adjacency list.
void BidirectionalAStar::SetOrigin(GraphReader& graphreader,
                                   valhalla::Location& origin,
                                   const TimeInfo& time_info) {
  // Only skip inbound edges if we have other options
  bool has_other_edges =
      std::any_of(origin.correlation().edges().begin(), origin.correlation().edges().end(),
                  [](const valhalla::PathEdge& e) { return !e.end_node(); });

  // Iterate through edges and add to adjacency list
  const NodeInfo* nodeinfo = nullptr;
  const NodeInfo* closest_ni = nullptr;
  for (const auto& edge : origin.correlation().edges()) {
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

    // Get the tile at the end node. Skip if tile not found as we won't be
    // able to expand from this origin edge.
    graph_tile_ptr endtile = graphreader.GetGraphTile(directededge->endnode());
    if (!endtile) {
      continue;
    }

    // Get cost and sort cost (based on distance from endnode of this edge
    // to the destination
    nodeinfo = endtile->node(directededge->endnode());
    uint8_t flow_sources;
    Cost cost = costing_->EdgeCost(directededge, tile, time_info, flow_sources) *
                (1.0f - edge.percent_along());

    // Store a node-info for later timezone retrieval (approximate for closest)
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
    if (hierarchy_limits_forward_[edgeid.level()].max_up_transitions != kUnlimitedTransitions) {
      // Override distance to the destination with a distance from the origin.
      // It will be used by hierarchy limits
      dist = astarheuristic_reverse_.GetDistance(nodeinfo->latlng(endtile->header()->base_ll()));
    }
    edgelabels_forward_.emplace_back(kInvalidLabel, edgeid, directededge, cost, sortcost, dist, mode_,
                                     -1, !(costing_->IsClosed(directededge, tile)),
                                     static_cast<bool>(flow_sources & kDefaultFlowMask),
                                     sif::InternalTurn::kNoTurn);
    adjacencylist_forward_.add(idx);

    // setting this edge as reached
    if (expansion_callback_) {
      expansion_callback_(graphreader, edgeid, "bidirectional_astar", "r", cost.secs, edge.distance(),
                          cost.cost);
    }

    // Set the initial not_thru flag to false. There is an issue with not_thru
    // flags on small loops. Set this to false here to override this for now.
    edgelabels_forward_.back().set_not_thru(false);

    pruning_disabled_at_origin_ = pruning_disabled_at_origin_ ||
                                  !edgelabels_forward_.back().closure_pruning() ||
                                  !edgelabels_forward_.back().not_thru_pruning();
  }

  // Set the origin timezone
  if (closest_ni != nullptr && !origin.date_time().empty() && origin.date_time() == "current") {
    origin.set_date_time(
        DateTime::iso_date_time(DateTime::get_tz_db().from_index(closest_ni->timezone())));
  }
}

// Add destination edges to the reverse path adjacency list.
void BidirectionalAStar::SetDestination(GraphReader& graphreader,
                                        const valhalla::Location& dest,
                                        const TimeInfo& time_info) {
  // Only skip outbound edges if we have other options
  bool has_other_edges =
      std::any_of(dest.correlation().edges().begin(), dest.correlation().edges().end(),
                  [](const valhalla::PathEdge& e) { return !e.begin_node(); });

  // Iterate through edges and add to adjacency list
  Cost c;
  for (const auto& edge : dest.correlation().edges()) {
    // If the destination is at a node, skip any outbound edges (so any
    // opposing inbound edges are not considered)
    if (has_other_edges && edge.begin_node()) {
      continue;
    }

    // Disallow any user avoided edges if the avoid location is behind the destination along the
    // edge
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
    if (!opp_dir_edge) {
      continue;
    }

    // Get cost and sort cost (based on distance from endnode of this edge
    // to the origin. Make sure we use the reverse A* heuristic. Use the
    // directed edge for costing, as this is the forward direction along the
    // destination edge. Note that the end node of the opposing edge is in the
    // same tile as the directed edge.
    uint8_t flow_sources;
    Cost cost =
        costing_->EdgeCost(directededge, tile, time_info, flow_sources) * edge.percent_along();

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
    edgestatus_reverse_.Set(opp_edge_id, EdgeSet::kTemporary, idx, opp_tile);
    if (hierarchy_limits_reverse_[opp_edge_id.level()].max_up_transitions != kUnlimitedTransitions) {
      // Override distance to the origin with a distance from the destination.
      // It will be used by hierarchy limits
      dist = astarheuristic_forward_.GetDistance(tile->get_node_ll(opp_dir_edge->endnode()));
    }
    edgelabels_reverse_.emplace_back(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, sortcost,
                                     dist, mode_, c, !opp_dir_edge->not_thru(),
                                     !(costing_->IsClosed(directededge, tile)),
                                     static_cast<bool>(flow_sources & kDefaultFlowMask),
                                     sif::InternalTurn::kNoTurn, -1);
    adjacencylist_reverse_.add(idx);

    // setting this edge as reached, sending the opposing because this is the reverse tree
    if (expansion_callback_) {
      expansion_callback_(graphreader, edgeid, "bidirectional_astar", "r", cost.secs, edge.distance(),
                          cost.cost);
    }

    // Set the initial not_thru flag to false. There is an issue with not_thru
    // flags on small loops. Set this to false here to override this for now.
    edgelabels_reverse_.back().set_not_thru(false);

    pruning_disabled_at_destination_ = pruning_disabled_at_destination_ ||
                                       !edgelabels_reverse_.back().closure_pruning() ||
                                       !edgelabels_reverse_.back().not_thru_pruning();
  }
}

// Form the path from the adjacency list.
std::vector<std::vector<PathInfo>> BidirectionalAStar::FormPath(GraphReader& graphreader,
                                                                const Options& options,
                                                                const valhalla::Location& origin,
                                                                const valhalla::Location& dest,
                                                                const baldr::TimeInfo& time_info) {
  LOG_DEBUG("Found connections before stretch filter: " + std::to_string(best_connections_.size()));

  if (desired_paths_count_ > 1) {
    // Cull alternate paths longer than maximum stretch
    // TODO: we should skip adding the connection at all if it's greater than stretch
    filter_alternates_by_stretch(best_connections_);
  }
  // For looking up edge ids on previously chosen best paths
  std::vector<std::unordered_set<GraphId>> shared_edgeids;

  // get maximum amount of sharing parameter based on origin->destination distance
  float max_sharing = desired_paths_count_ > 1 ? get_max_sharing(origin, dest) : 0.f;

  LOG_DEBUG("Connections after stretch filter: " + std::to_string(best_connections_.size()));

#ifdef LOGGING_LEVEL_TRACE
  LOG_TRACE("CONNECTIONS FOUND " + std::to_string(best_connections_.size()));
  for (const auto& b : best_connections_) {
    auto tile = graphreader.GetGraphTile(b.edgeid);
    if (tile == nullptr) {
      printf("graphreader.GetGraphTile(b.edgeid) is null\n");
      continue;
    }
    auto nodes = graphreader.GetDirectedEdgeNodes(b.edgeid, tile);
    auto first_node_tile = graphreader.GetGraphTile(nodes.first);
    if (first_node_tile == nullptr) {
      printf("graphreader.GetGraphTile(nodes.first) is null\n");
      continue;
    }

    auto sll = first_node_tile->node(nodes.first)->latlng(first_node_tile->header()->base_ll());

    auto second_node_tile = graphreader.GetGraphTile(nodes.second);
    if (second_node_tile == nullptr) {
      printf("graphreader.GetGraphTile(nodes.second) is null\n");
      continue;
    }
    auto ell = second_node_tile->node(nodes.second)->latlng(second_node_tile->header()->base_ll());
    printf("[[%.6f,%.6f],[%.6f,%.6f]],\n", sll.lng(), sll.lat(), ell.lng(), ell.lat());
  }
#endif

  // we quit making paths as soon as we've reached the number of paths
  // that were requested or we run out of paths that we can actually make
  std::vector<std::vector<PathInfo>> paths;
  for (auto best_connection = best_connections_.cbegin();
       paths.size() < desired_paths_count_ && best_connection != best_connections_.cend();
       ++best_connection) {
    // Get the indexes where the connection occurs.
    uint32_t idx1 = edgestatus_forward_.Get(best_connection->edgeid).index();
    uint32_t idx2 = edgestatus_reverse_.Get(best_connection->opp_edgeid).index();

    // Metrics (TODO - more accurate cost)
    uint32_t pathcost = edgelabels_forward_[idx1].cost().cost + edgelabels_reverse_[idx2].cost().cost;
    LOG_DEBUG("path_cost::" + std::to_string(pathcost));
    LOG_DEBUG("FormPath path_iterations::" + std::to_string(edgelabels_forward_.size()) + "," +
              std::to_string(edgelabels_reverse_.size()));

    // set of edges recovered from shortcuts (excluding shortcut's start edges)
    std::unordered_set<GraphId> recovered_inner_edges;

    // A place to keep the path
    std::vector<GraphId> path_edges;
    path_edges.reserve(static_cast<size_t>(paths.empty() ? 0.f : paths.back().size() * 1.2f));

    // Work backwards on the forward path
    graph_tile_ptr tile;
    for (auto edgelabel_index = idx1; edgelabel_index != kInvalidLabel;
         edgelabel_index = edgelabels_forward_[edgelabel_index].predecessor()) {
      const BDEdgeLabel& edgelabel = edgelabels_forward_[edgelabel_index];

      const DirectedEdge* edge = graphreader.directededge(edgelabel.edgeid(), tile);
      if (edge == nullptr) {
        throw tile_gone_error_t("BidirectionalAStar::FormPath failed", edgelabel.edgeid());
      }

      if (edge->is_shortcut()) {
        auto superseded = graphreader.RecoverShortcut(edgelabel.edgeid());
        recovered_inner_edges.insert(superseded.begin() + 1, superseded.end());
        std::move(superseded.rbegin(), superseded.rend(), std::back_inserter(path_edges));
      } else
        path_edges.push_back(edgelabel.edgeid());

      // Check if this is a ferry
      if (edgelabel.use() == Use::kFerry) {
        has_ferry_ = true;
      }
    }

    // Reverse the list
    std::reverse(path_edges.begin(), path_edges.end());

    // Append the reverse path from the destination - use opposing edges
    // The first edge on the reverse path is the same as the last on the forward
    // path, so get the predecessor.
    for (auto edgelabel_index = edgelabels_reverse_[idx2].predecessor();
         edgelabel_index != kInvalidLabel;
         edgelabel_index = edgelabels_reverse_[edgelabel_index].predecessor()) {
      const BDEdgeLabel& edgelabel = edgelabels_reverse_[edgelabel_index];
      const DirectedEdge* opp_edge = nullptr;
      GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgelabel.edgeid(), opp_edge, tile);
      if (opp_edge == nullptr) {
        throw tile_gone_error_t("BidirectionalAStar::FormPath failed", edgelabel.edgeid());
      }

      if (opp_edge->is_shortcut()) {
        auto superseded = graphreader.RecoverShortcut(opp_edge_id);
        recovered_inner_edges.insert(superseded.begin() + 1, superseded.end());
        std::move(superseded.begin(), superseded.end(), std::back_inserter(path_edges));
      } else
        path_edges.emplace_back(std::move(opp_edge_id));

      // Check if this is a ferry
      if (edgelabel.use() == Use::kFerry) {
        has_ferry_ = true;
      }
    }

    // bidirectional a* has a bug where it fails trivial routes in which you are on a one way edge and
    // the origin is near the end of the edge and the destination is near the beginning, in other
    // words a route that looks trivial but actually needs to go around the block to complete
    if (path_edges.size() == 1)
      LOG_WARN("Trivial route with bidirectional A* should not be allowed");

    // once we recovered the whole path we should construct list of PathInfo objects
    std::vector<PathInfo> path;
    path.reserve(path_edges.size());

    auto edge_itr = path_edges.begin();
    const auto edge_cb = [&edge_itr, &path_edges]() {
      return (edge_itr == path_edges.end()) ? GraphId{} : (*edge_itr++);
    };

    const auto label_cb = [&path, &recovered_inner_edges](const EdgeLabel& label) {
      path.emplace_back(label.mode(), label.cost(), label.edgeid(), 0, label.path_distance(),
                        label.restriction_idx(), label.transition_cost(),
                        recovered_inner_edges.count(label.edgeid()));
    };

    float source_pct;
    try {
      source_pct = find_percent_along(origin, path_edges.front());
    } catch (...) { throw std::logic_error("Could not find candidate edge used for origin label"); }

    float target_pct;
    try {
      target_pct = find_percent_along(dest, path_edges.back());
    } catch (...) {
      throw std::logic_error("Could not find candidate edge used for destination label");
    }

    // recost edges in final path; ignore access restrictions
    try {
      bool invariant = options.date_time_type() == Options::invariant;
      sif::recost_forward(graphreader, *costing_, edge_cb, label_cb, source_pct, target_pct,
                          time_info, invariant, true);
    } catch (const std::exception& e) {
      LOG_ERROR(std::string("Bi-directional astar failed to recost final path: ") + e.what());
      continue;
    }

    // For the first path just add it for subsequent paths only add if it passes viability tests
    if (paths.empty() || (validate_alternate_by_sharing(shared_edgeids, paths, path, max_sharing) &&
                          validate_alternate_by_stretch(paths.front(), path) &&
                          validate_alternate_by_local_optimality(path))) {
      paths.emplace_back(std::move(path));
    }
  }
  // give back the paths
  return paths;
}

void BidirectionalAStar::ModifyHierarchyLimits() {
  // Distance threshold optimized for unidirectional search. For bidirectional case
  // they can be lowered.
  // Decrease distance thresholds only for arterial roads for now
  if (hierarchy_limits_forward_[1].max_up_transitions != kUnlimitedTransitions)
    hierarchy_limits_forward_[1].expansion_within_dist /= 5.f;

  if (hierarchy_limits_reverse_[1].max_up_transitions != kUnlimitedTransitions)
    hierarchy_limits_reverse_[1].expansion_within_dist /= 5.f;
}

bool IsBridgingEdgeRestricted(GraphReader& graphreader,
                              std::vector<sif::BDEdgeLabel>& edge_labels_fwd,
                              std::vector<sif::BDEdgeLabel>& edge_labels_rev,
                              const BDEdgeLabel& fwd_pred,
                              const BDEdgeLabel& rev_pred,
                              const std::shared_ptr<sif::DynamicCost>& costing) {

  const uint8_t M = 10;                 // TODO Look at data to figure this out
  const uint8_t PATCH_PATH_SIZE = M * 2 // Expand M in both directions
                                  + 1;  // Also need space for pred in the middle

  // Begin by building the "patch" path
  std::vector<GraphId> patch_path;
  patch_path.reserve(PATCH_PATH_SIZE);

  patch_path.push_back(fwd_pred.edgeid());

  auto next_fwd_pred = fwd_pred;
  for (int i = 0; i < M; ++i) {
    // Walk M edges back and add each to patch path
    const uint32_t next_pred_idx = next_fwd_pred.predecessor();
    if (next_pred_idx == baldr::kInvalidLabel) {
      break;
    }
    next_fwd_pred = edge_labels_fwd[next_pred_idx];
    if (!next_fwd_pred.on_complex_rest()) {
      // We can actually stop here if this edge is no longer part of any complex restriction
      break;
    }

    patch_path.push_back(next_fwd_pred.edgeid());
  }

  // Reverse patch_path so that the leftmost edge is first and original `pred`
  // at the end before pushing the right-hand edges (opposite direction) onto the back
  std::reverse(patch_path.begin(), patch_path.end());

  graph_tile_ptr tile = nullptr; // Used for later hinting

  auto next_rev_pred = rev_pred;
  // Now push_back the edges from opposite direction onto our patch_path
  for (int n = 0; n < PATCH_PATH_SIZE; ++n) {
    auto next_rev_pred_idx = next_rev_pred.predecessor();
    if (next_rev_pred_idx == kInvalidLabel) {
      // We reached the end of the opposing tree, i.e. destination or origin
      break;
    }
    next_rev_pred = edge_labels_rev[next_rev_pred_idx];
    if (!next_rev_pred.on_complex_rest()) {
      // We can actually stop here if this edge is no longer path of any complex restriction
      break;
    }

    // Check for double u-turn. It might happen in the following case:
    // forward and reverse searches traverse edges that belong to the same complex
    // restriction. Then forward/reverse search reaches last edge and due to the
    // restriction can't go futher and make a u-turn. After that forward and reverse
    // searches meet at some edge and compose double u-turn.
    if (std::find(patch_path.begin(), patch_path.end(), next_rev_pred.opp_edgeid()) !=
        patch_path.end())
      return true;

    // Since we are on the reverse expansion here, we want the opp_edgeid
    // since we're tracking everything in the forward direction
    const auto edgeid = next_rev_pred.opp_edgeid();
    patch_path.push_back(edgeid);

    // Also grab restrictions while walking for later comparison against patch_path
    tile = graphreader.GetGraphTile(edgeid, tile);
    if (tile == nullptr) {
      throw std::logic_error("Tile pointer was null in IsBridgingEdgeRestricted");
    }
    const auto* edge = tile->directededge(edgeid);
    if (edge->end_restriction() & costing->access_mode()) {
      auto restrictions = tile->GetRestrictions(true, edgeid, costing->access_mode());
      if (restrictions.size() == 0) {
        // TODO Should we actually throw here? Or assert to gracefully continue in release?
        // This implies corrupt data or logic bug
        throw std::logic_error(
            "Found no restrictions in tile even though edge-label.on_complex_rest() == true");
        break;
      }
      for (auto cr : restrictions) {
        // For each restriction `cr`, grab the end id PLUS vias PLUS beginning
        std::vector<GraphId> restriction_ids;
        // We must add beginning and ending edge as well, not just the vias,
        // to track the full restriction
        restriction_ids.push_back(cr->to_graphid());
        cr->WalkVias([&restriction_ids](const GraphId* id) {
          restriction_ids.push_back(*id);
          return WalkingVia::KeepWalking;
        });
        // We must add beginning and ending edge as well, not just the vias,
        // to track the full restriction
        restriction_ids.push_back(cr->from_graphid());

        // Now, lets see if this restriction matches part of our patch_path
        if (std::search(patch_path.cbegin(), patch_path.cend(), restriction_ids.crbegin(),
                        restriction_ids.crend()) != patch_path.cend()) {
          // The restriction matches! This path is restricted and we can exit
          return true;
        }
      }
    }
  }

  // No restrictions matched our patch path
  return false;
}

} // namespace thor
} // namespace valhalla
