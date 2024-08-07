#ifndef VALHALLA_THOR_BIDIRECTIONAL_ASTAR_H_
#define VALHALLA_THOR_BIDIRECTIONAL_ASTAR_H_

#include <cstdint>
#include <memory>
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
  bool ignore_hierarchy_limits_;

  // A* heuristic
  float cost_diff_;
  AStarHeuristic astarheuristic_forward_;
  AStarHeuristic astarheuristic_reverse_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::BDEdgeLabel> edgelabels_forward_;
  std::vector<sif::BDEdgeLabel> edgelabels_reverse_;

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
  void Expand(baldr::GraphReader& graphreader,
              const baldr::GraphId& node,
              sif::BDEdgeLabel& pred,
              const uint32_t pred_idx,
              const baldr::DirectedEdge* opp_pred_edge,
              const baldr::TimeInfo& time_info,
              const bool invariant);

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
  inline bool ExpandInner(baldr::GraphReader& graphreader,
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
   * @return  Returns the path infos, a list of GraphIds representing the
   *          directed edges along the path - ordered from origin to
   *          destination - along with travel modes and elapsed time.
   */
  std::vector<std::vector<PathInfo>> FormPath(baldr::GraphReader& graphreader,
                                              const Options& options,
                                              const valhalla::Location& origin,
                                              const valhalla::Location& dest,
                                              const baldr::TimeInfo& time_info);

  /**
   * Modify default (optimized for unidirectional search) hierarchy limits.
   */
  void ModifyHierarchyLimits();
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
