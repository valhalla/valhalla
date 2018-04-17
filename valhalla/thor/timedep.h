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
          odin::Location& dest, baldr::GraphReader& graphreader,
          const std::shared_ptr<sif::DynamicCost>* mode_costing,
          const sif::TravelMode mode);

 protected:

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
   * @param  localtime    Current local time.
   * @param  dest         Location information of the destination.
   * @param  best_path    Best path found so far. Includes the index into
   *                      EdgeLabels and the cost.
   */
  void ExpandForward(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node, const sif::EdgeLabel& pred,
                     const uint32_t pred_idx, const bool from_transition,
                     const uint32_t localtime, const odin::Location& dest,
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
  virtual std::vector<PathInfo> GetBestPath(odin::Location& origin,
          odin::Location& dest, baldr::GraphReader& graphreader,
          const std::shared_ptr<sif::DynamicCost>* mode_costing,
          const sif::TravelMode mode);

 protected:

  /**
   * Expand from the node along the reverse search path. Immediately expands
   * from the end node of any transition edge (so no transition edges are added
   * to the adjacency list or EdgeLabel list). Does not expand transition
   * edges if from_transition is false.
   * @param  graphreader  Graph tile reader.
   * @param  node         Graph Id of the node being expanded.
   * @param  pred         Predecessor edge label (for costing).
   * @param  pred_idx     Predecessor index into the EdgeLabel list.
   * @param  from_transition True if this method is called from a transition
   *                         edge.
   * @param  localtime    Current local time.
   * @param  dest         Location information of the destination.
   * @param  best_path    Best path found so far. Includes the index into
   *                      EdgeLabels and the cost.
   */
  void ExpandReverse(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node, const sif::EdgeLabel& pred,
                     const uint32_t pred_idx, const bool from_transition,
                     const uint32_t localtime, const odin::Location& dest,
                     std::pair<int32_t, float>& best_path);
};

}
}

#endif  // VALHALLA_THOR_TIMEDEP_H_
