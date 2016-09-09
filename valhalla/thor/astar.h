#ifndef VALHALLA_THOR_ASTAR_H_
#define VALHALLA_THOR_ASTAR_H_

#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <memory>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/hierarchylimits.h>
#include <valhalla/thor/astarheuristic.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathinfo.h>
#include <valhalla/thor/pathalgorithm.h>

namespace valhalla {
namespace thor {

/**
 * Single direction A* algorithm to create the shortest / least cost path.
 * For driving routes it uses a highway hierarchy with shortcut edges to
 * improve performance.
 */
class AStarPathAlgorithm : public PathAlgorithm {
 public:
  /**
   * Constructor.
   */
  AStarPathAlgorithm();

  /**
   * Destructor
   */
  virtual ~AStarPathAlgorithm();

  /**
   * Form path between and origin and destination location using the supplied
   * costing method.
   * @param  origin       Origin location
   * @param  dest         Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  costing      Costing methods.
   * @param  mode         Travel mode to use.
   * @return Returns the path edges (and elapsed time/modes at end of
   *          each edge).
   */
  virtual std::vector<PathInfo> GetBestPath(baldr::PathLocation& origin,
          baldr::PathLocation& dest, baldr::GraphReader& graphreader,
          const std::shared_ptr<sif::DynamicCost>* mode_costing,
          const sif::TravelMode mode);

  /**
   * Clear the temporary information generated during path construction.
   */
  virtual void Clear();

 protected:
  // Allow transitions (set from the costing model)
  bool allow_transitions_;

  // Current travel mode
  sif::TravelMode mode_;

  // Tile creation date
  uint32_t tile_creation_date_;

  // Hierarchy limits.
  std::vector<sif::HierarchyLimits> hierarchy_limits_;

  // A* heuristic
  AStarHeuristic astarheuristic_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::EdgeLabel> edgelabels_;

  // Adjacency list - approximate double bucket sort
  std::shared_ptr<baldr::DoubleBucketQueue> adjacencylist_;

  // Edge status. Mark edges that are in adjacency list or settled.
  std::shared_ptr<EdgeStatus> edgestatus_;

  // Destinations, id and cost
  std::map<uint64_t, sif::Cost> destinations_;

  /**
   * Initializes the hierarchy limits, A* heuristic, and adjacency list.
   * @param  origll  Lat,lng of the origin.
   * @param  destll  Lat,lng of the destination.
   * @param  costing Dynamic costing method.
   */
  virtual void Init(const PointLL& origll, const PointLL& destll,
            const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Convenience method to add an edge to the adjacency list and temporarily
   * label it. This must be called before adding the edge label (so it uses
   * the correct index).
   * @param  edgeid    Edge to add to the adjacency list.
   * @param  sortcost  Sort cost.
   */
  void AddToAdjacencyList(const baldr::GraphId& edgeid, const float sortcost);

  /**
   * Check if edge is temporarily labeled and this path has less cost. If
   * less cost the predecessor is updated and the sort cost is decremented
   * by the difference in real cost (A* heuristic doesn't change).
   * @param  idx        Index into the edge status list.
   * @param  predindex  Index of the predecessor edge.
   * @param  newcost    Cost of the new path.
   */
  void CheckIfLowerCostPath(const uint32_t idx,
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
  void HandleTransitionEdge(const uint32_t level,const baldr::GraphId& edgeid,
                      const baldr::DirectedEdge* edge,
                      const sif::EdgeLabel& pred, const uint32_t predindex,
                      const float dist);

  /**
   * Modify hierarchy limits based on distance between origin and destination
   * and the relative road density at the destination. For shorter routes
   * we stay on arterial roads further from the destination. Also for lower
   * road densities near the destination the hierarchy transition distances
   * are increased.
   * @param   dist     Distance between origin and destination.
   * @param   density  Relative road density near the destination.
   */
  void ModifyHierarchyLimits(const float dist, const uint32_t density);

  /**
   * Add edges at the origin to the adjacency list.
   * @param  graphreader  Graph tile reader.
   * @param  origin       Location information of the origin.
   * @param  dest         Location information of the destination.
   * @param  costing      Dynamic costing.
   */
  void SetOrigin(baldr::GraphReader& graphreader,
                 baldr::PathLocation& origin,
                 const baldr::PathLocation& dest,
                 const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Set the destination edge(s).
   * @param   graphreader  Graph tile reader.
   * @param   dest         Location information of the destination.
   * @param   costing      Dynamic costing.
   * @return  Returns the relative density near the destination (0-15)
   */
  uint32_t SetDestination(baldr::GraphReader& graphreader,
                          const baldr::PathLocation& dest,
                          const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Test if the completed path is trivial (on the same edge in a forward
   * direction from the origin to the destination.
   * @param  edgeid   Edge where path completion occurs.
   * @param  orig     Location information of the origin.
   * @param  origin   Location information of the destination
   * @return Returns true if the path is trivial.
   */
  bool IsTrivial(const baldr::GraphId& edgeid,
                 const baldr::PathLocation& orig,
                 const baldr::PathLocation& dest) const;

  /**
   * Form the path from the adjacency list. Recovers the path from the
   * destination backwards towards the origin (using predecessor information)
   * @param   dest  Index in the edge labels of the destination edge.
   * @return  Returns the path info, a list of GraphIds representing the
   *          directed edges along the path - ordered from origin to
   *          destination - along with travel modes and elapsed time.
   */
  std::vector<PathInfo> FormPath(const uint32_t dest);
};

}
}

#endif  // VALHALLA_THOR_ASTAR_H_
