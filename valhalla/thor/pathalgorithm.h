#ifndef VALHALLA_THOR_PATHALGORITHM_H_
#define VALHALLA_THOR_PATHALGORITHM_H_

#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <memory>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/hierarchylimits.h>
#include <valhalla/thor/adjacencylist.h>
#include <valhalla/thor/astarheuristic.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

constexpr uint32_t kBucketCount = 20000;

// If the destination is at a node we want the incoming edge Ids
// with distance = 1.0 (the full edge). This returns and updated
// destination PathLocation.
// TODO - move this logic into Loki
baldr::PathLocation update_destinations(baldr::GraphReader& graphreader,
                                 const baldr::PathLocation& destination,
                                 const sif::EdgeFilter& filter);

/**
 * Algorithm to create shortest path. This is a single direction A* algorithm.
 * For driving routes it uses a highway hierarchy with shortcut edges to
 * improve performance.
 */
class PathAlgorithm {
 public:
  /**
   * Constructor.
   */
  PathAlgorithm();

  /**
   * Destructor
   */
  virtual ~PathAlgorithm();

  /**
   * Form path between and origin and destination location using the supplied
   * costing method.
   * @param  origin  Origin location
   * @param  dest    Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  costing  Costing methods.
   * @param  mode     Travel mode to use.
   * @return  Returns the path edges (and elapsed time/modes at end of
   *          each edge).
   */
  virtual std::vector<PathInfo> GetBestPath(const baldr::PathLocation& origin,
          const baldr::PathLocation& dest, baldr::GraphReader& graphreader,
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

  // Current walking distance. TODO - make this distance or mode distance?
  uint32_t walking_distance_;

  // Hierarchy limits.
  std::vector<sif::HierarchyLimits> hierarchy_limits_;

  // A* heuristic
  AStarHeuristic astarheuristic_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::EdgeLabel> edgelabels_;

  // Adjacency list - approximate double bucket sort
  std::shared_ptr<AdjacencyList> adjacencylist_;

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
                 const baldr::PathLocation& origin,
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

/**
 * Multi-modal pathfinding algorithm. Currently supports walking and
 * transit (bus, subway, light-rail, etc.).
 */
class MultiModalPathAlgorithm : public PathAlgorithm {
 public:
  /**
   * Constructor.
   */
  MultiModalPathAlgorithm();

  /**
   * Destructor
   */
  virtual ~MultiModalPathAlgorithm();

  /**
   * Form multi-modal path between and origin and destination location using
   * the supplied costing method.
   * @param  origin  Origin location
   * @param  dest    Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  costing  An array of costing methods, one per TravelMode.
   * @param  mode     Travel mode from the origin.
   * @return  Returns the path edges (and elapsed time/modes at end of
   *          each edge).
   */
  std::vector<PathInfo> GetBestPath(const baldr::PathLocation& origin,
           const baldr::PathLocation& dest, baldr::GraphReader& graphreader,
           const std::shared_ptr<sif::DynamicCost>* mode_costing,
           const sif::TravelMode mode);

 protected:

  /**
   * Initializes the hierarchy limits, A* heuristic, and adjacency list.
   * @param  origll  Lat,lng of the origin.
   * @param  destll  Lat,lng of the destination.
   * @param  costing Dynamic costing method.
   */
  void Init(const PointLL& origll, const PointLL& destll,
            const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Check if destination can be reached if walking is the last mode. Checks
   * if there are any transit stops within maximum walking distance from
   * the destination. This is used to reject impossible routes given the
   * modes allowed.
   * TODO - once auto/bicycle are allowed modes we need to check if parking
   * or bikeshare locations are within walking distance.
   */
  bool CanReachDestination(const baldr::PathLocation& destination,
           baldr::GraphReader& graphreader, const sif::TravelMode dest_mode,
           const std::shared_ptr<sif::DynamicCost>& costing);
};

}
}

#endif  // VALHALLA_THOR_PATHALGORITHM_H_
