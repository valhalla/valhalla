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
  virtual std::vector<PathInfo> GetBestPath(odin::Location& origin,
                                            odin::Location& dest,
                                            baldr::GraphReader& graphreader,
                                            const std::shared_ptr<sif::DynamicCost>* mode_costing,
                                            const sif::TravelMode mode);

protected:
  uint32_t origin_tz_index_;

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
   * @param  dest         Location information of the destination.
   * @param  best_path    Best path found so far. Includes the index into
   *                      EdgeLabels and the cost.
   */
  void ExpandForward(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     const sif::EdgeLabel& pred,
                     const uint32_t pred_idx,
                     const bool from_transition,
                     uint64_t localtime,
                     const odin::Location& dest,
                     std::pair<int32_t, float>& best_path);

  /**
   * Get the timezone at the origin. Uses the timezone at the end node
   * of the lowest cost edge.
   * @param  graphreader  Graph tile reader.
   * @return Returns the timezone index or -1 if an error occurs.
   */
  int GetOriginTimezone(baldr::GraphReader& graphreader);
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
  virtual std::vector<PathInfo> GetBestPath(odin::Location& origin,
                                            odin::Location& dest,
                                            baldr::GraphReader& graphreader,
                                            const std::shared_ptr<sif::DynamicCost>* mode_costing,
                                            const sif::TravelMode mode);

protected:
  uint32_t dest_tz_index_;

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
  void Init(const PointLL& origll, const PointLL& destll);

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
   * @param  dest         Location information of the destination.
   * @param  best_path    Best path found so far. Includes the index into
   *                      EdgeLabels and the cost.
   */
  void ExpandReverse(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     const sif::BDEdgeLabel& pred,
                     const uint32_t pred_idx,
                     const baldr::DirectedEdge* opp_pred_edge,
                     const bool from_transition,
                     uint64_t localtime,
                     const odin::Location& dest,
                     std::pair<int32_t, float>& best_path);

  /**
   * Get the timezone at the destination. Uses the timezone at the end node
   * of the lowest cost edge.
   * @param  graphreader  Graph tile reader.
   * @return Returns the timezone index or -1 if an error occurs.
   */
  int GetDestinationTimezone(baldr::GraphReader& graphreader);

  /**
   * The origin of the reverse path is the destination location. Add edges at the
   * destination to the adjacency list to start the reverse path search.
   * @param  graphreader  Graph tile reader.
   * @param  origin       Location information of the origin.
   * @param  dest         Location information of the destination.
   */
  void
  SetOrigin(baldr::GraphReader& graphreader, odin::Location& origin, odin::Location& destination);

  /**
   * The destination of the reverse path is the origin location. Set the
   * destination edge(s) so we know when to terminate the search.
   * @param   graphreader  Graph tile reader.
   * @param   dest         Location information of the destination.
   * @return  Returns the relative density near the destination (0-15)
   */
  uint32_t SetDestination(baldr::GraphReader& graphreader, const odin::Location& dest);

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
