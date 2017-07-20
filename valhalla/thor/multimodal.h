#ifndef VALHALLA_THOR_MULTIMODAL_H_
#define VALHALLA_THOR_MULTIMODAL_H_

#include <cstdint>
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
#include <valhalla/thor/astar.h>

namespace valhalla {
namespace thor {

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
  std::vector<PathInfo> GetBestPath(baldr::PathLocation& origin,
           baldr::PathLocation& dest, baldr::GraphReader& graphreader,
           const std::shared_ptr<sif::DynamicCost>* mode_costing,
           const sif::TravelMode mode);

  /**
   * Clear the temporary information generated during path construction.
   */
  void Clear();

 protected:
  // Current walking distance.
  uint32_t walking_distance_;

  uint32_t max_label_count_;    // Max label count to allow
  sif::TravelMode mode_;        // Current travel mode
  uint8_t travel_type_;         // Current travel type

  // Hierarchy limits.
  std::vector<sif::HierarchyLimits> hierarchy_limits_;

  // A* heuristic
  AStarHeuristic astarheuristic_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::MMEdgeLabel> edgelabels_;

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
  void Init(const PointLL& origll, const PointLL& destll,
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

#endif  // VALHALLA_THOR_MULTIMODAL_H_
