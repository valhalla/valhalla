#ifndef VALHALLA_THOR_BIDIRECTIONAL_ASTAR_H_
#define VALHALLA_THOR_BIDIRECTIONAL_ASTAR_H_

#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <memory>

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
};

/**
 * Bidirectional A* algorithm. Use for pedestrian routes (and bicycle?) only
 * at this time due to probable difficulties joining whe highway hierarchies
 * and shortcuts are used.
 */
class BidirectionalAStar : public PathAlgorithm {
 public:
  /**
   * Constructor.
   */
  BidirectionalAStar();

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
  std::vector<PathInfo> GetBestPath(baldr::PathLocation& origin,
           baldr::PathLocation& dest, baldr::GraphReader& graphreader,
           const std::shared_ptr<sif::DynamicCost>* mode_costing,
           const sif::TravelMode mode);

  /**
   * Clear the temporary information generated during path construction.
   */
  void Clear();

 protected:
  // A*, edge labels, adjacency list, and edge status for the reverse path
  AStarHeuristic astarheuristic_reverse_;
  std::vector<sif::HierarchyLimits> hierarchy_limits_reverse_;
  std::vector<sif::EdgeLabel> edgelabels_reverse_;
  std::shared_ptr<AdjacencyList> adjacencylist_reverse_;
  std::shared_ptr<EdgeStatus> edgestatus_reverse_;

  // Best candidate connection
  CandidateConnection best_connection_;

  /**
   * Initialize the A* heuristic and adjacency lists for both the forward
   * and reverse search.
   * @param  origll  Lat,lng of the origin.
   * @param  destll  Lat,lng of the destination.
   * @param  costing Dynamic costing method.
   */
  void Init(const PointLL& origll, const PointLL& destll,
            const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Add edges at the origin to the forward adjacency list.
   * @param  graphreader  Graph tile reader.
   * @param  origin       Location information of the destination
   * @param  costing      Dynamic costing
   */
  void SetOrigin(baldr::GraphReader& graphreader,
                 baldr::PathLocation& origin,
                 const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Add destination edges to the reverse path adjacency list.
   * @param   dest         Location information of the destination
   * @param   costing      Dynamic costing
   */
  void SetDestination(baldr::GraphReader& graphreader,
                       const baldr::PathLocation& dest,
                       const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Modify hierarchy limits based on distance between origin and destination
   * and the relative road density at the destination. For shorter routes
   * we stay on arterial roads further from the destination. Also for lower
   * road densities near the destination the hierarchy transition distances
   * are increased.
   * @param   dist     Distance between origin and destination.
   * @param   density  Relative road density near the destination.
   */
  void ModifyHierarchyLimitsReverse(const float dist, const uint32_t density);

  /**
   * Convenience method to add an edge to the adjacency list and temporarily
   * label it. This must be called before adding the edge label (so it uses
   * the correct index). Adds to the reverse path from destination towards
   * the origin.
   * @param  edgeid    Edge to add to the adjacency list.
   * @param  sortcost  Sort cost.
   */
  void AddToAdjacencyListReverse(const baldr::GraphId& edgeid,
                                 const float sortcost);

  /**
    * Check if edge is temporarily labeled and this path has less cost. If
    * less cost the predecessor is updated and the sort cost is decremented
    * by the difference in real cost (A* heuristic doesn't change). This
    * method applies to the reverse path portion of the bidirectional search.
    * @param  idx        Index into the edge status list.
    * @param  predindex  Index of the predecessor edge.
    * @param  newcost    Cost of the new path.
    */
   void CheckIfLowerCostPathReverse(const uint32_t idx,
                             const uint32_t predindex,
                             const sif::Cost& newcost);

   /**
    * Handle transition edges. Will add any that are allowed to the
    * adjacency list.
    * @param level      Current hierarchy level
    * @param edge       Directed edge (a transition edge)
    * @param pred       Predecessor information
    * @param predindex  Predecessor index in the edge labels.
    * @param dist       Distance to the destination.
    */
   void HandleTransitionEdgeReverse(const uint32_t level,const baldr::GraphId& edgeid,
                       const baldr::DirectedEdge* edge,
                       const sif::EdgeLabel& pred, const uint32_t predindex,
                       const float dist);

   /**
     * Form the path from the adjacency lists. Recovers the path from the
     * where the paths meet back towards the origin then reverses this path.
     * The path from where the paths meet to the destination is then appended
     * using the opposing edges (so the path is traversed forward).
     * @param   idx1  Index in the forward search where the paths meet.
     * @param   idx2  Index in the reverse search where the paths meet.
     * @param   graphreader  Graph tile reader (for getting opposing edges).
     * @return  Returns the path info, a list of GraphIds representing the
     *          directed edges along the path - ordered from origin to
     *          destination - along with travel modes and elapsed time.
     */
  std::vector<PathInfo> FormPath(const uint32_t idx1, const uint32_t idx2,
             baldr::GraphReader& graphreader);
};

}
}

#endif  // VALHALLA_THOR_BIDIRECTIONAL_ASTAR_H_
