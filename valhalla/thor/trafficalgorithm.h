#ifndef VALHALLA_THOR_TRAFFICALGORITHM_H_
#define VALHALLA_THOR_TRAFFICALGORITHM_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/thor/astar.h>

namespace valhalla {
namespace thor {

/**
 * Traffic pathfinding algorithm. Note that this is just a quick proof of
 * concept for how tiled speeds could be used to influence route paths and
 * estimated times. Ideally this would be integrated into sif (costing) and
 * baldr (tile access).
 */
class TrafficAlgorithm : public AStarPathAlgorithm {
public:
  /**
   * Constructor.
   */
  TrafficAlgorithm();

  /**
   * Destructor
   */
  virtual ~TrafficAlgorithm();

  /**
   * Form path between and origin and destination location using
   * the supplied costing method and real-time speed tiles.
   * @param  origin  Origin location
   * @param  dest    Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  mode_costing  An array of costing methods, one per TravelMode.
   * @param  mode     Travel mode from the origin.
   * @return  Returns the path edges (and elapsed time/modes at end of
   *          each edge).
   */
  std::multimap<float, std::vector<PathInfo>>
  GetBestPath(valhalla::Location& origin,
              valhalla::Location& dest,
              baldr::GraphReader& graphreader,
              const std::shared_ptr<sif::DynamicCost>* mode_costing,
              const sif::TravelMode mode);

protected:
  // Map of real-time speeds
  std::unordered_map<uint32_t, std::vector<uint8_t>> real_time_speeds_;
  std::vector<uint8_t> empty_speeds_;

  /**
   * Get address of the real-time speed table for the specified tile.
   * Loads the speeds if they are not yet loaded.
   */
  std::vector<uint8_t>& GetRealTimeSpeeds(const uint32_t tileid, baldr::GraphReader& graphreader);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_TRAFFICALGORITHM_H_
