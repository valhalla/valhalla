#ifndef VALHALLA_THOR_BIDIRECTIONAL_ASTAR_H_
#define VALHALLA_THOR_BIDIRECTIONAL_ASTAR_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/time_info.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/hierarchylimits.h>
#include <valhalla/thor/astarheuristic.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathalgorithm.h>

namespace valhalla {
namespace thor {

/**
 * Candidate connections - a directed edge and its opposing directed edge
 * are both temporarily labeled. Store the edge Ids and its cost.
 */
struct CandidateConnection {
  baldr::GraphId edgeid;
  baldr::GraphId opp_edgeid;
  float cost;
  bool operator<(const CandidateConnection& o) const {
    return cost < o.cost;
  }
  bool operator<(float c) const {
    return cost < c;
  }
};

/**
 * Bidirectional A* algorithm. Method for finding least-cost path.
 */
class BidirectionalAStar : public PathAlgorithm {
public:
  /**
   * Constructor.
   * @param config A config object of key, value pairs
   */
  explicit BidirectionalAStar(const boost::property_tree::ptree& config = {});

  /**
   * Destructor
   */
  virtual ~BidirectionalAStar();

  /**
   * Form path between and origin and destination location using
   * the supplied mode and costing method.
   * @param  origin  Origin location
   * @param  dest    Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  mode_costing  An array of costing methods, one per TravelMode.
   * @param  mode     Travel mode from the origin.
   * @return  Returns the path edges (and elapsed time/modes at end of
   *          each edge).
   */
  std::vector<std::vector<PathInfo>>
  GetBestPath(valhalla::Location& origin,
              valhalla::Location& dest,
              baldr::GraphReader& graphreader,
              const sif::mode_costing_t& mode_costing,
              const sif::TravelMode mode,
              const Options& options = Options::default_instance()) override;

  /**
   * Returns the name of the algorithm
   * @return the name of the algorithm
   */
  virtual const char* name() const override {
    return "bidirectional_a*";
  }

  /**
   * Clear the temporary information generated during path construction.
   */
  void Clear() override;

protected:
  enum class ExpansionType { forward = 0, reverse = 1 };

  // Access mode used by the costing method
  uint32_t access_mode_;

  // Current travel mode
  sif::TravelMode mode_;

  // Current travel type
  uint8_t travel_type_;

  // Current costing mode
  std::shared_ptr<sif::DynamicCost> costing_;

  // Hierarchy limits
  std::vector<sif::HierarchyLimits> hierarchy_limits_forward_;
  std::vector<sif::HierarchyLimits> hierarchy_limits_reverse_;

  // A* heuristic
  float cost_diff_;
  AStarHeuristic astarheuristic_forward_;
  AStarHeuristic astarheuristic_reverse_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::BDEdgeLabel> edgelabels_forward_;
  std::vector<sif::BDEdgeLabel> edgelabels_reverse_;
  uint32_t max_reserved_labels_count_;

  // Adjacency list - approximate double bucket sort
  baldr::DoubleBucketQueue<sif::BDEdgeLabel> adjacencylist_forward_;
  baldr::DoubleBucketQueue<sif::BDEdgeLabel> adjacencylist_reverse_;

  // Edge status. Mark edges that are in adjacency list or settled.
  EdgeStatus edgestatus_forward_;
  EdgeStatus edgestatus_reverse_;

  // Best candidate connection and threshold to extend search.
  float cost_threshold_;
  uint32_t iterations_threshold_;
  uint32_t desired_paths_count_;
  std::vector<CandidateConnection> best_connections_;

  // Extends search in one direction if the other direction exhausted, but only if the non-exhausted
  // end started on a not_thru or closed (due to live-traffic) edge
  bool extended_search_;
  // Stores the pruning state at origin & destination. Its true if _any_ of the candidate edges at
  // these locations has pruning turned off (pruning is off if starting from a closed or not_thru
  // edge)
  bool pruning_disabled_at_origin_, pruning_disabled_at_destination_;

  /**
   * Initialize the A* heuristic and adjacency lists for both the forward
   * and reverse search.
   * @param  origll  Lat,lng of the origin.
   * @param  destll  Lat,lng of the destination.
   */
  void Init(const midgard::PointLL& origll, const midgard::PointLL& destll);

  /**
   * Expand from the node along the forward search path
   *
   * @param graphreader        to access graph data
   * @param node               the node from which to expand
   * @param pred               the previous edge label in the forward expansion
   * @param pred_idx           the index of the label in the label set
   * @param time_info          time tracking information about the start of the route
   * @param invariant          static date_time, dont offset the time as the path lengthens
   * @return returns true if the expansion continued from this node
   */
  template <const ExpansionType expansion_direction,
            const bool FORWARD = expansion_direction == ExpansionType::forward>
  bool Expand(baldr::GraphReader& graphreader,
              const baldr::GraphId& node,
              sif::BDEdgeLabel& pred,
              const uint32_t pred_idx,
              const baldr::DirectedEdge* opp_pred_edge,
              const baldr::TimeInfo& time_info,
              const bool invariant) {
    // Get the tile and the node info. Skip if tile is null (can happen
    // with regional data sets) or if no access at the node.
    graph_tile_ptr tile = graphreader.GetGraphTile(node);
    if (tile == nullptr) {
      return false;
    }
    const baldr::NodeInfo* nodeinfo = tile->node(node);

    // Keep track of superseded edges
    uint32_t shortcuts = 0;

    // Update the time information even if time is invariant to account for timezones
    auto seconds_offset = invariant ? 0.f : pred.cost().secs;
    auto offset_time =
        FORWARD ? time_info.forward(seconds_offset, static_cast<int>(nodeinfo->timezone()))
                : time_info.reverse(seconds_offset, static_cast<int>(nodeinfo->timezone()));

    auto& edgestatus = FORWARD ? edgestatus_forward_ : edgestatus_reverse_;

    // If we encounter a node with an access restriction like a barrier we allow a uturn
    if (!costing_->Allowed(nodeinfo)) {
      const baldr::DirectedEdge* opp_edge = nullptr;
      const baldr::GraphId opp_edge_id = graphreader.GetOpposingEdgeId(pred.edgeid(), opp_edge, tile);
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

    // Expand from end node in forward direction.
    for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++meta) {

      // Begin by checking if this is the opposing edge to pred.
      // If so, it means we are attempting a u-turn. In that case, lets wait with evaluating
      // this edge until last. If any other edges were emplaced, it means we should not
      // even try to evaluate a u-turn since u-turns should only happen for deadends
      uturn_meta = pred.opp_local_idx() == meta.edge->localedgeidx() ? meta : uturn_meta;

      // Expand but only if this isnt the uturn, we'll try that later if nothing else works out
      disable_uturn =
          (pred.opp_local_idx() != meta.edge->localedgeidx() &&
           ExpandInner<expansion_direction>(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx,
                                            meta, shortcuts, tile, offset_time)) ||
          disable_uturn;
    }

    // Handle transitions - expand from the end node of each transition
    if (nodeinfo->transition_count() > 0) {
      const baldr::NodeTransition* trans = tile->transition(nodeinfo->transition_index());
      auto& hierarchy_limits = FORWARD ? hierarchy_limits_forward_ : hierarchy_limits_reverse_;
      for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
        // if this is a downward transition (ups are always allowed) AND we are no longer allowed OR
        // we cant get the tile at that level (local extracts could have this problem) THEN bail
        graph_tile_ptr trans_tile = nullptr;
        if ((!trans->up() && hierarchy_limits[trans->endnode().level()].StopExpanding()) ||
            !(trans_tile = graphreader.GetGraphTile(trans->endnode()))) {
          continue;
        }

        hierarchy_limits[node.level()].up_transition_count += trans->up();
        // setup for expansion at this level
        const auto* trans_node = trans_tile->node(trans->endnode());
        EdgeMetadata trans_meta =
            EdgeMetadata::make(trans->endnode(), trans_node, trans_tile, edgestatus);
        uint32_t trans_shortcuts = 0;
        // expand the edges from this node at this level
        for (uint32_t i = 0; i < trans_node->edge_count(); ++i, ++trans_meta) {
          disable_uturn =
              ExpandInner<expansion_direction>(graphreader, pred, opp_pred_edge, trans_node, pred_idx,
                                               trans_meta, trans_shortcuts, trans_tile,
                                               offset_time) ||
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
  template <const ExpansionType expansion_direction,
            const bool FORWARD = expansion_direction == ExpansionType::forward,
            const bool REVERSE = expansion_direction == ExpansionType::reverse>
  bool ExpandInner(baldr::GraphReader& graphreader,
                   const sif::BDEdgeLabel& pred,
                   const baldr::DirectedEdge* opp_pred_edge,
                   const baldr::NodeInfo* nodeinfo,
                   const uint32_t pred_idx,
                   const EdgeMetadata& meta,
                   uint32_t& shortcuts,
                   const graph_tile_ptr& tile,
                   const baldr::TimeInfo& time_info) {
    auto& hierarchy_limits = FORWARD ? hierarchy_limits_forward_ : hierarchy_limits_reverse_;
    // Skip shortcut edges until we have stopped expanding on the next level. Use regular
    // edges while still expanding on the next level since we can still transition down to
    // that level. If using a shortcut, set the shortcuts mask. Skip if this is a regular
    // edge superseded by a shortcut.
    if (meta.edge->is_shortcut()) {
      if (hierarchy_limits[meta.edge_id.level() + 1].StopExpanding()) {
        shortcuts |= meta.edge->shortcut();
      } else {
        return false;
      }
    } else if (shortcuts & meta.edge->superseded()) {
      return false;
    }

    // Skip this edge if edge is permanently labeled (best path already found
    // to this directed edge), if no access is allowed (based on costing method),
    // or if a complex restriction prevents transition onto this edge.
    if (meta.edge_status->set() == EdgeSet::kPermanent) {
      return true; // This is an edge we _could_ have expanded, so return true
    }

    graph_tile_ptr t2 = nullptr;
    baldr::GraphId opp_edge_id;
    const baldr::DirectedEdge* opp_edge = nullptr;

    if (REVERSE) {
      // TODO Why is this check necessary? opp_edge.forwardaccess() is checked in Allowed(...)
      if (!(meta.edge->reverseaccess() & access_mode_)) {
        return false;
      }

      // Get end node tile, opposing edge Id, and opposing directed edge.
      t2 = meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
      if (t2 == nullptr) {
        return false;
      }

      opp_edge_id = t2->GetOpposingEdgeId(meta.edge);
      opp_edge = t2->directededge(opp_edge_id);
    }

    // Skip this edge if no access is allowed (based on costing method)
    // or if a complex restriction prevents transition onto this edge.
    // if its not time dependent set to 0 for Allowed and Restricted methods below
    const uint64_t localtime = time_info.valid ? time_info.local_time : 0;
    uint8_t restriction_idx = -1;
    if (FORWARD) {
      if (!costing_->Allowed(meta.edge, pred, tile, meta.edge_id, localtime, time_info.timezone_index,
                             restriction_idx) ||
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

    // Get cost. Separate out transition cost.
    sif::Cost transition_cost =
        FORWARD ? costing_->TransitionCost(meta.edge, nodeinfo, pred)
                : costing_->TransitionCostReverse(meta.edge->localedgeidx(), nodeinfo, opp_edge,
                                                  opp_pred_edge, pred.has_measured_speed(),
                                                  pred.internal_turn());
    uint8_t flow_sources;
    sif::Cost newcost =
        pred.cost() + transition_cost +
        (FORWARD ? costing_->EdgeCost(meta.edge, tile, time_info.second_of_week, flow_sources)
                 : costing_->EdgeCost(opp_edge, t2, time_info.second_of_week, flow_sources));

    // Check if edge is temporarily labeled and this path has less cost. If
    // less cost the predecessor is updated and the sort cost is decremented
    // by the difference in real cost (A* heuristic doesn't change)
    if (meta.edge_status->set() == EdgeSet::kTemporary) {
      sif::BDEdgeLabel& lab = FORWARD ? edgelabels_forward_[meta.edge_status->index()]
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
    if (FORWARD) {
      t2 = meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
      if (t2 == nullptr) {
        return false;
      }
      opp_edge_id = t2->GetOpposingEdgeId(meta.edge);
    }

    // Find the sort cost (with A* heuristic) using the lat,lng at the
    // end node of the directed edge.
    float dist = 0.0f;
    float sortcost =
        newcost.cost +
        (FORWARD ? astarheuristic_forward_.Get(t2->get_node_ll(meta.edge->endnode()), dist)
                 : astarheuristic_reverse_.Get(t2->get_node_ll(meta.edge->endnode()), dist));

    // Add edge label, add to the adjacency list and set edge status
    uint32_t idx = 0;
    if (FORWARD) {
      idx = edgelabels_forward_.size();
      edgelabels_forward_.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, newcost,
                                       sortcost, dist, mode_, transition_cost,
                                       (pred.not_thru_pruning() || !meta.edge->not_thru()),
                                       (pred.closure_pruning() ||
                                        !costing_->IsClosed(meta.edge, tile)),
                                       static_cast<bool>(flow_sources & baldr::kDefaultFlowMask),
                                       costing_->TurnType(pred.opp_local_idx(), nodeinfo, meta.edge),
                                       restriction_idx);
      adjacencylist_forward_.add(idx);
    } else {
      idx = edgelabels_reverse_.size();
      edgelabels_reverse_.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, newcost,
                                       sortcost, dist, mode_, transition_cost,
                                       (pred.not_thru_pruning() || !meta.edge->not_thru()),
                                       (pred.closure_pruning() ||
                                        !costing_->IsClosed(meta.edge, tile)),
                                       static_cast<bool>(flow_sources & baldr::kDefaultFlowMask),
                                       costing_->TurnType(meta.edge->localedgeidx(), nodeinfo,
                                                          opp_edge, opp_pred_edge),
                                       restriction_idx);
      adjacencylist_reverse_.add(idx);
    }

    *meta.edge_status = {EdgeSet::kTemporary, idx};

    // setting this edge as reached
    if (expansion_callback_) {
      expansion_callback_(graphreader, "bidirectional_astar", FORWARD ? meta.edge_id : opp_edge_id,
                          "r", false);
    }

    // we've just added this edge to the queue, but we won't expand from it if it's a not-thru edge
    // that will be pruned. In that case we want to allow uturns.
    return !(pred.not_thru_pruning() && meta.edge->not_thru());
  }

  /**
   * Add edges at the origin to the forward adjacency list.
   * @param graphreader  Graph tile reader.
   * @param origin       Location information of the destination
   * @param time_info    What time is it when we start the route
   */
  void SetOrigin(baldr::GraphReader& graphreader,
                 valhalla::Location& origin,
                 const baldr::TimeInfo& time_info);

  /**
   * Add destination edges to the reverse path adjacency list.
   * @param graphreader  Graph tile reader.
   * @param dest         Location information of the destination
   * @param time_info    What time is it when we end the route
   */
  void SetDestination(baldr::GraphReader& graphreader,
                      const valhalla::Location& dest,
                      const baldr::TimeInfo& time_info);

  /**
   * The edge on the forward search connects to a reached edge on the reverse
   * search tree. Check if this is the best connection so far and set the
   * search threshold.
   * @param  pred  Edge label of the predecessor.
   * @return Returns true if a connection was set, false if not (if on a complex restriction).
   */
  bool SetForwardConnection(baldr::GraphReader& graphreader, const sif::BDEdgeLabel& pred);

  /**
   * The edge on the reverse search connects to a reached edge on the forward
   * search tree. Check if this is the best connection so far and set the
   * search threshold.
   * @param  pred  Edge label of the predecessor.
   * @return Returns true if a connection was set, false if not (if on a complex restriction).
   */
  bool SetReverseConnection(baldr::GraphReader& graphreader, const sif::BDEdgeLabel& pred);

  /**
   * Form the path from the adjacency lists. Recovers the path from the
   * where the paths meet back towards the origin then reverses this path.
   * The path from where the paths meet to the destination is then appended
   * using the opposing edges (so the path is traversed forward).
   * @param   graphreader  Graph tile reader (for getting opposing edges).
   * @param   options      Controls whether or not we get alternatives
   * @param   origin       The origin location
   * @param   destination  The destination location
   * @param   time_info    What time is it when we start the route
   * @param   invariant    Static date_time, dont offset the time as the path lengthens
   * @return  Returns the path infos, a list of GraphIds representing the
   *          directed edges along the path - ordered from origin to
   *          destination - along with travel modes and elapsed time.
   */
  std::vector<std::vector<PathInfo>> FormPath(baldr::GraphReader& graphreader,
                                              const Options& options,
                                              const valhalla::Location& origin,
                                              const valhalla::Location& dest,
                                              const baldr::TimeInfo& time_info,
                                              const bool invariant);
};

// This function checks if the path formed by the two expanding trees
// when connected by `pred` triggers a complex restriction.
//
// At a high level, the code forms a smaller path of the full path to
// be formed by `pred` if accepted. This "patch" path is then tested for
// restrictions.
//
// To do this, the code walks M edges back into `edge_labels`
// while tracking the vias from edges with starting complex restrictions.
//
// These M edges, plus the current `pred`, plus M edges from `edge_labels_opposite_direction`
// form a "patch" path. The vias that were tracked earlier are then compared
// to this patch path to see if they match anywhere in the larger patch path.
//
// M must be chosen apriori to be larger than the length of known complex restrictions.
// TODO Implement a lazy approach where we don't need a priori knowledge. This
// could be done by storing number of complex restrictions in each edge-label.
//
//   edge_labels |        | edge_labels_opposite_direction
//               |        |
//   E-2    E-1  | pred   |  E1    E2
// N ---> N ---> N -----> N ---> N ---> N
//                opp_pred
//
// |<-------   PATCH_PATH -------------->|
//
// If no restriction triggers, it returns true and the edge is allowed
bool IsBridgingEdgeRestricted(valhalla::baldr::GraphReader& graphreader,
                              std::vector<sif::BDEdgeLabel>& edge_labels_fwd,
                              std::vector<sif::BDEdgeLabel>& edge_labels_rev,
                              const sif::BDEdgeLabel& fwd_pred,
                              const sif::BDEdgeLabel& rev_pred,
                              const std::shared_ptr<sif::DynamicCost>& costing);

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_BIDIRECTIONAL_ASTAR_H_
