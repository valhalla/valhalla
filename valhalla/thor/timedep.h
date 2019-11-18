#ifndef VALHALLA_THOR_TIMEDEP_H_
#define VALHALLA_THOR_TIMEDEP_H_

#include <valhalla/thor/astar.h>

namespace valhalla {
namespace thor {

/**
 * Forward direction A* algorithm to create the shortest / least cost path.
 * This algorithm is used for "depart-at", time-dependent routes.
 * For driving routes it uses a highway hierarchy with shortcut edges to
 * improve performance.
 */
class TimeDepForward : public AStarPathAlgorithm {
public:
  /**
   * Constructor.
   */
  TimeDepForward();

  /**
   * Destructor
   */
  virtual ~TimeDepForward();

  /**
   * Form path between and origin and destination location using the supplied
   * costing method.
   * @param  origin       Origin location
   * @param  dest         Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  mode_costing Costing methods for each mode.
   * @param  mode         Travel mode to use.
   * @return Returns the path edges (and elapsed time/modes at end of
   *          each edge).
   */
  std::vector<std::vector<PathInfo>>
  GetBestPath(valhalla::Location& origin,
              valhalla::Location& dest,
              baldr::GraphReader& graphreader,
              const std::shared_ptr<sif::DynamicCost>* mode_costing,
              const sif::TravelMode mode,
              const Options& options = Options::default_instance());

protected:
  uint32_t origin_tz_index_;
  uint32_t seconds_of_week_;

  /**
   * Expand from the node along the forward search path. Immediately expands
   * from the end node of any transition edge (so no transition edges are added
   * to the adjacency list or EdgeLabel list). Does not expand transition
   * edges if from_transition is false.
   * @param  graphreader  Graph tile reader.
   * @param  node         Graph Id of the node being expanded.
   * @param  pred         Predecessor edge label (for costing).
   * @param  pred_idx     Predecessor index into the EdgeLabel list.
   * @param  from_transition True if this method is called from a transition
   *                         edge.
   * @param  localtime    Current local time.  Seconds since epoch
   * @param  seconds_of_week Seconds from start of the week (local time).
   * @param  dest         Location information of the destination.
   * @param  best_path    Best path found so far. Includes the index into
   *                      EdgeLabels and the cost.
   */
  bool ExpandForward(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     sif::EdgeLabel& pred,
                     const uint32_t pred_idx,
                     const bool from_transition,
                     uint64_t localtime,
                     int32_t seconds_of_week,
                     const valhalla::Location& dest,
                     std::pair<int32_t, float>& best_path);

  // Private helper function for `ExpandReverse`
  inline bool ExpandForwardInner(baldr::GraphReader& graphreader,
                                 const sif::EdgeLabel& pred,
                                 const baldr::NodeInfo* nodeinfo,
                                 const uint32_t pred_idx,
                                 const EdgeMetadata& meta,
                                 const baldr::GraphTile* tile,
                                 uint64_t localtime,
                                 uint32_t seconds_of_week,
                                 const valhalla::Location& destination,
                                 std::pair<int32_t, float>& best_path);
};

/**
 * Reverse direction A* algorithm to create the shortest / least cost path.
 * This algorithm is used for "arrive-by", time-dependent routes. For driving
 * routes it uses a highway hierarchy with shortcut edges to improve
 * performance.
 */
class TimeDepReverse : public AStarPathAlgorithm {
public:
  /**
   * Constructor.
   */
  TimeDepReverse();

  /**
   * Destructor
   */
  virtual ~TimeDepReverse();

  /**
   * Form path between and origin and destination location using the supplied
   * costing method. Forms the path by seaching in reverse order.
   * @param  origin       Origin location
   * @param  dest         Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  mode_costing Costing methods for each mode.
   * @param  mode         Travel mode to use.
   * @return Returns the path edges (and elapsed time/modes at end of
   *          each edge).
   */
  virtual std::vector<std::vector<PathInfo>>
  GetBestPath(valhalla::Location& origin,
              valhalla::Location& dest,
              baldr::GraphReader& graphreader,
              const std::shared_ptr<sif::DynamicCost>* mode_costing,
              const sif::TravelMode mode,
              const Options& options = Options::default_instance());

protected:
  uint32_t dest_tz_index_;
  uint32_t seconds_of_week_;

  // Access mode used by the costing method
  uint32_t access_mode_;

  // Vector of edge labels that support reverse search (so use the
  // bidirectional edge label structure.
  std::vector<sif::BDEdgeLabel> edgelabels_rev_;

  /**
   * Initializes the hierarchy limits, A* heuristic, and adjacency list.
   * @param  origll  Lat,lng of the origin.
   * @param  destll  Lat,lng of the destination.
   */
  void Init(const midgard::PointLL& origll, const midgard::PointLL& destll);

  /**
   * Expand from the node along the reverse search path. Immediately expands
   * from the end node of any transition edge (so no transition edges are added
   * to the adjacency list or EdgeLabel list). Does not expand transition
   * edges if from_transition is false.
   * @param  graphreader  Graph tile reader.
   * @param  node         Graph Id of the node being expanded.
   * @param  pred         Predecessor edge label (for costing).
   * @param  pred_idx     Predecessor index into the EdgeLabel list.
   * @param  opp_pred_edge Opposing predecessor directed edge.
   * @param  from_transition True if this method is called from a transition
   *                         edge.
   * @param  localtime    Current local time.  Seconds since epoch
   * @param  seconds_of_week Seconds from start of the week (local time).
   * @param  dest         Location information of the destination.
   * @param  best_path    Best path found so far. Includes the index into
   *                      EdgeLabels and the cost.
   */
  bool ExpandReverse(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     sif::BDEdgeLabel& pred,
                     const uint32_t pred_idx,
                     const baldr::DirectedEdge* opp_pred_edge,
                     const bool from_transition,
                     uint64_t localtime,
                     int32_t seconds_of_week,
                     const valhalla::Location& dest,
                     std::pair<int32_t, float>& best_path);

  // Private helper function for `ExpandReverse`
  bool ExpandReverseInner(baldr::GraphReader& graphreader,
                          const sif::BDEdgeLabel& pred,
                          const baldr::DirectedEdge* opp_pred_edge,
                          const baldr::NodeInfo* nodeinfo,
                          const uint32_t pred_idx,
                          const EdgeMetadata& meta,
                          const baldr::GraphTile* tile,
                          uint64_t localtime,
                          uint32_t seconds_of_week,
                          const valhalla::Location& destination,
                          std::pair<int32_t, float>& best_path);

  /**
   * The origin of the reverse path is the destination location. Add edges at the
   * destination to the adjacency list to start the reverse path search.
   * @param  graphreader  Graph tile reader.
   * @param  origin       Location information of the origin.
   * @param  dest         Location information of the destination.
   */
  void SetOrigin(baldr::GraphReader& graphreader,
                 valhalla::Location& origin,
                 valhalla::Location& destination);

  /**
   * The destination of the reverse path is the origin location. Set the
   * destination edge(s) so we know when to terminate the search.
   * @param   graphreader  Graph tile reader.
   * @param   dest         Location information of the destination.
   * @return  Returns the relative density near the destination (0-15)
   */
  uint32_t SetDestination(baldr::GraphReader& graphreader, const valhalla::Location& dest);

  /**
   * Form the path from the adjacency list. Recovers the path from the
   * destination (true origin) backwards towards the origin (true destination)
   * (using predecessor information). This path is then reversed.
   * @param   graphreader  Graph reader.
   * @param   dest  Index in the edge labels of the destination edge.
   * @return  Returns the path info, a list of GraphIds representing the
   *          directed edges along the path - ordered from origin to
   *          destination - along with travel modes and elapsed time.
   */
  std::vector<PathInfo> FormPath(baldr::GraphReader& graphreader, const uint32_t dest);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_TIMEDEP_H_
