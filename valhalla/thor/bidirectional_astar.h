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
  template <const ExpansionType expansion_direction>
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
    auto offset_time = (expansion_direction == ExpansionType::forward) ?
        time_info.forward(seconds_offset, static_cast<int>(nodeinfo->timezone())) :
        time_info.reverse(seconds_offset, static_cast<int>(nodeinfo->timezone()));

    // If we encounter a node with an access restriction like a barrier we allow a uturn
    if (!costing_->Allowed(nodeinfo)) {
      const baldr::DirectedEdge* opp_edge = nullptr;
      const baldr::GraphId opp_edge_id = graphreader.GetOpposingEdgeId(pred.edgeid(), opp_edge, tile);
      // Mark the predecessor as a deadend to be consistent with how the
      // edgelabels are set when an *actual* deadend (i.e. some dangling OSM geometry)
      // is labelled
      pred.set_deadend(true);
      // Check if edge is null before using it (can happen with regional data sets)
      if (expansion_direction == ExpansionType::forward) {
        return opp_edge && ExpandForwardInner(graphreader, pred, nodeinfo, pred_idx,
                                            {opp_edge, opp_edge_id,
                                             edgestatus_forward_.GetPtr(opp_edge_id, tile)},
                                            shortcuts, tile, offset_time);
      } else {
        return opp_edge &&
                  ExpandReverseInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx,
                              {opp_edge, opp_edge_id, edgestatus_reverse_.GetPtr(opp_edge_id, tile)},
                              shortcuts, tile, offset_time);
      }
    }

    bool disable_uturn = false;
    EdgeMetadata meta = (expansion_direction == ExpansionType::forward) ?
      EdgeMetadata::make(node, nodeinfo, tile, edgestatus_forward_) :
      EdgeMetadata::make(node, nodeinfo, tile, edgestatus_reverse_);
    EdgeMetadata uturn_meta{};

    // Expand from end node in forward direction.
    for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++meta) {

      // Begin by checking if this is the opposing edge to pred.
      // If so, it means we are attempting a u-turn. In that case, lets wait with evaluating
      // this edge until last. If any other edges were emplaced, it means we should not
      // even try to evaluate a u-turn since u-turns should only happen for deadends
      uturn_meta = pred.opp_local_idx() == meta.edge->localedgeidx() ? meta : uturn_meta;

      // Expand but only if this isnt the uturn, we'll try that later if nothing else works out
      disable_uturn = (pred.opp_local_idx() != meta.edge->localedgeidx() &&
      (expansion_direction == ExpansionType::forward ?
                       ExpandForwardInner(graphreader, pred, nodeinfo, pred_idx, meta, shortcuts,
                                          tile, offset_time) :
                     ExpandReverseInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx, meta,
                                        shortcuts, tile, offset_time)
      ))
                                        || disable_uturn;
    }

    // Handle transitions - expand from the end node of each transition
    if (nodeinfo->transition_count() > 0) {
      const baldr::NodeTransition* trans = tile->transition(nodeinfo->transition_index());
      for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
        // if this is a downward transition (ups are always allowed) AND we are no longer allowed OR
        // we cant get the tile at that level (local extracts could have this problem) THEN bail
        graph_tile_ptr trans_tile = nullptr;
        if (expansion_direction == ExpansionType::forward) {
          if ((!trans->up() && hierarchy_limits_forward_[trans->endnode().level()].StopExpanding()) ||
              !(trans_tile = graphreader.GetGraphTile(trans->endnode()))) {
            continue;
          }
          hierarchy_limits_forward_[node.level()].up_transition_count += trans->up();
        } else {
          if ((!trans->up() && hierarchy_limits_reverse_[trans->endnode().level()].StopExpanding()) ||
              !(trans_tile = graphreader.GetGraphTile(trans->endnode()))) {
            continue;
          }
          hierarchy_limits_reverse_[node.level()].up_transition_count += trans->up();
        }
        // setup for expansion at this level
        const auto* trans_node = trans_tile->node(trans->endnode());
        EdgeMetadata trans_meta = (expansion_direction == ExpansionType::forward) ?
            EdgeMetadata::make(trans->endnode(), trans_node, trans_tile, edgestatus_forward_) :
            EdgeMetadata::make(trans->endnode(), trans_node, trans_tile, edgestatus_reverse_);
        uint32_t trans_shortcuts = 0;
        // expand the edges from this node at this level
        for (uint32_t i = 0; i < trans_node->edge_count(); ++i, ++trans_meta) {
          disable_uturn = ((expansion_direction == ExpansionType::forward) ?
                          ExpandForwardInner(graphreader, pred, trans_node, pred_idx, trans_meta,
                                             trans_shortcuts, trans_tile, offset_time) :
                          ExpandReverseInner(graphreader, pred, opp_pred_edge, trans_node, pred_idx,
                                           trans_meta, trans_shortcuts, trans_tile, offset_time)) ||
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
      disable_uturn = ((expansion_direction == ExpansionType::forward) ?
                        ExpandForwardInner(graphreader, pred, nodeinfo, pred_idx, uturn_meta, shortcuts,
                                         tile, offset_time) :
                        ExpandReverseInner(graphreader, pred, opp_pred_edge, nodeinfo, pred_idx,
                                       uturn_meta, shortcuts, tile, offset_time)) ||
                      disable_uturn;
    }

    return disable_uturn;
  }

  // Private helper function for `ExpandForward`
  bool ExpandForwardInner(baldr::GraphReader& graphreader,
                          const sif::BDEdgeLabel& pred,
                          const baldr::NodeInfo* nodeinfo,
                          const uint32_t pred_idx,
                          const EdgeMetadata& meta,
                          uint32_t& shortcuts,
                          const graph_tile_ptr& tile,
                          const baldr::TimeInfo& time_info);

  /**
   * Expand from the node along the reverse search path
   *
   * @param graphreader        to access graph data
   * @param node               the node from which to expand
   * @param pred               the previous edge label in the reverse expansion
   * @param pred_idx           the index of the label in the label set
   * @param time_info          time tracking information about the end of the route
   * @param invariant          static date_time, dont offset the time as the path lengthens
   * @return returns true if the expansion continued from this node in this direction
   */
  bool ExpandReverse(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     sif::BDEdgeLabel& pred,
                     const uint32_t pred_idx,
                     const baldr::DirectedEdge* opp_pred_edge,
                     const baldr::TimeInfo& time_info,
                     const bool invariant);
  // Private helper function for `ExpandReverse`
  bool ExpandReverseInner(baldr::GraphReader& graphreader,
                          const sif::BDEdgeLabel& pred,
                          const baldr::DirectedEdge* opp_pred_edge,
                          const baldr::NodeInfo* nodeinfo,
                          const uint32_t pred_idx,
                          const EdgeMetadata& meta,
                          uint32_t& shortcuts,
                          const graph_tile_ptr& tile,
                          const baldr::TimeInfo& time_info);
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
