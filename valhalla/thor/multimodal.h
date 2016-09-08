#ifndef VALHALLA_THOR_MULTIMODAL_H_
#define VALHALLA_THOR_MULTIMODAL_H_

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
class MultiModalPathAlgorithm : public AStarPathAlgorithm {
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

 protected:
  // Current walking distance.
  uint32_t walking_distance_;

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

#endif  // VALHALLA_THOR_MULTIMODAL_H_
