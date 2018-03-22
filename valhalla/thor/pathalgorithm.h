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
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/thor/pathinfo.h>
#include <valhalla/proto/tripcommon.pb.h>

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
  PathAlgorithm()
     : interrupt(nullptr),
       has_ferry_(false) {
  }

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
  virtual std::vector<PathInfo> GetBestPath(odin::Location& origin,
          odin::Location& dest, baldr::GraphReader& graphreader,
          const std::shared_ptr<sif::DynamicCost>* mode_costing,
          const sif::TravelMode mode) = 0;

  /**
   * Clear the temporary information generated during path construction.
   */
  virtual void Clear() = 0;

  /**
   * Set a callback that will throw when the path computation should be aborted
   * @param interrupt_callback  the function to periodically call to see if
   *                            we should abort
   */
  void set_interrupt(const std::function<void ()>* interrupt_callback) {
    interrupt = interrupt_callback;
  }

  /**
   * Does the path include a ferry?
   * @return  Returns true if the path includes a ferry.
   */
  bool has_ferry() const {
    return has_ferry_;
  }

 protected:
  const std::function<void()>* interrupt;

  bool has_ferry_;    // Indicates whether the path has a ferry

  /**
   * Check for path completion along the same edge. Edge ID in question
   * is along both an origin and destination and origin shows up at the
   * beginning of the edge while the destination shows up at the end of
   * the edge.
   * @param  edgeid       Edge id.
   * @param  origin       Origin path location information.
   * @param  destination  Destination path location information.
   */
  bool IsTrivial(const baldr::GraphId& edgeid,
                 const odin::Location& origin,
                 const odin::Location& destination) const {
    for (const auto& destination_edge : destination.path_edges()) {
      if (destination_edge.graph_id() == edgeid) {
        for (const auto& origin_edge : origin.path_edges()) {
          if (origin_edge.graph_id() == edgeid &&
              origin_edge.percent_along() <= destination_edge.percent_along()) {
            return true;
          }
        }
      }
    }
    return false;
  }
};

}
}

#endif  // VALHALLA_THOR_PATHALGORITHM_H_
