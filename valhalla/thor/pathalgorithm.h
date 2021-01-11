#pragma once

#include <functional>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/edgestatus.h>
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
  PathAlgorithm() : interrupt(nullptr), has_ferry_(false), expansion_callback_() {
  }

  /**
   * Destructor
   */
  virtual ~PathAlgorithm() {
  }

  /**
   * Form path between and origin and destination location using the supplied
   * costing method.
   * @param  origin       Origin location
   * @param  dest         Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  mode_costing Costing methods.
   * @param  mode         Travel mode to use.
   * @return Returns the path edges (and elapsed time/modes at end of
   *          each edge).
   */
  virtual std::vector<std::vector<PathInfo>>
  GetBestPath(valhalla::Location& origin,
              valhalla::Location& dest,
              baldr::GraphReader& graphreader,
              const sif::mode_costing_t& mode_costing,
              const sif::TravelMode mode,
              const Options& options = Options::default_instance()) = 0;

  /**
   * Returns the name of the algorithm
   * @return the name of the algorithm
   */
  virtual const char* name() const = 0;

  /**
   * Clear the temporary information generated during path construction.
   */
  virtual void Clear() = 0;

  /**
   * Set a callback that will throw when the path computation should be aborted
   * @param interrupt_callback  the function to periodically call to see if
   *                            we should abort
   */
  void set_interrupt(const std::function<void()>* interrupt_callback) {
    interrupt = interrupt_callback;
  }

  /**
   * Does the path include a ferry?
   * @return  Returns true if the path includes a ferry.
   */
  bool has_ferry() const {
    return has_ferry_;
  }

  /**
   * Sets the functor which will track the algorithms expansion.
   *
   * @param  expansion_callback  the functor to call back when the algorithm makes progress
   *                             on a given edge
   */
  using expansion_callback_t =
      std::function<void(baldr::GraphReader&, const char*, baldr::GraphId, const char*, bool)>;
  void set_track_expansion(const expansion_callback_t& expansion_callback) {
    expansion_callback_ = expansion_callback;
  }

protected:
  const std::function<void()>* interrupt;

  bool has_ferry_; // Indicates whether the path has a ferry

  // for tracking the expansion of the algorithm visually
  expansion_callback_t expansion_callback_;

  // when doing timezone differencing a timezone cache speeds up the computation
  baldr::DateTime::tz_sys_info_cache_t tz_cache_;

  /**
   * Check for path completion along the same edge. Edge ID in question
   * is along both an origin and destination and origin shows up at the
   * beginning of the edge while the destination shows up at the end of
   * the edge.
   * @param  edgeid       Edge id.
   * @param  origin       Origin path location information.
   * @param  destination  Destination path location information.
   */
  virtual bool IsTrivial(const baldr::GraphId& edgeid,
                         const valhalla::Location& origin,
                         const valhalla::Location& destination) const {
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

// Container for the data we iterate over in Expand* function
struct EdgeMetadata {
  const baldr::DirectedEdge* edge;
  baldr::GraphId edge_id;
  EdgeStatusInfo* edge_status;

  inline static EdgeMetadata make(const baldr::GraphId& node,
                                  const baldr::NodeInfo* nodeinfo,
                                  const graph_tile_ptr& tile,
                                  EdgeStatus& edge_status_) {
    baldr::GraphId edge_id = {node.tileid(), node.level(), nodeinfo->edge_index()};
    EdgeStatusInfo* edge_status = edge_status_.GetPtr(edge_id, tile);
    const baldr::DirectedEdge* directededge = tile->directededge(edge_id);
    return {directededge, edge_id, edge_status};
  }

  inline EdgeMetadata& operator++() {
    ++edge;
    ++edge_id;
    ++edge_status;
    return *this;
  }

  inline operator bool() const {
    return edge;
  }

  inline bool operator!() const {
    return !edge;
  }
};

} // namespace thor
} // namespace valhalla
