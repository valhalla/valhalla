#ifndef VALHALLA_THOR_PATHALGORITHM_H_
#define VALHALLA_THOR_PATHALGORITHM_H_

#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <memory>
#include <functional>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

constexpr uint32_t kBucketCount = 20000;
constexpr size_t kInterruptIterationsInterval = 5000;

/**
 * Pure virtual class defining the interface for PathAlgorithm - the algorithm
 * to create shortest path.
 */
class PathAlgorithm {
 public:
  /**
   * Constructor
   */
  PathAlgorithm():interrupt(nullptr) { }

  /**
   * Destructor
   */
  virtual ~PathAlgorithm() { }

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
          const sif::TravelMode mode) = 0;

  /**
   * Clear the temporary information generated during path construction.
   */
  virtual void Clear() = 0;

  /**
   * Set a callback that will throw when the path computation should be aborted
   *
   * @param interrupt_callback  the function to periodically call to see if we should abort
   */
  void set_interrupt(const std::function<void ()>* interrupt_callback) { interrupt = interrupt_callback; }

 protected:
  const std::function<void()>* interrupt;
};

}
}

#endif  // VALHALLA_THOR_PATHALGORITHM_H_
