#pragma once

#include <functional>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/api.pb.h>
// TODO(nils): should abstract more so we don't pull this in
#include <valhalla/thor/pathalgorithm.h>

namespace valhalla {
namespace thor {

/**
 * Pure virtual class defining the interface for MatrixAlgorithm
 */
class MatrixAlgorithm {
public:
  /**
   * Constructor
   */
  MatrixAlgorithm(const boost::property_tree::ptree& config)
      : interrupt_(nullptr), expansion_callback_(),
        clear_reserved_memory_(config.get<bool>("clear_reserved_memory", false)) {
  }

  MatrixAlgorithm(const MatrixAlgorithm&) = delete;
  MatrixAlgorithm& operator=(const MatrixAlgorithm&) = delete;

  /**
   * Destructor
   */
  virtual ~MatrixAlgorithm() {
  }

  /**
   * Forms a time distance matrix from the set of source locations
   * to the set of target locations.
   * @param  request               the full request
   * @param  graphreader           List of source/origin locations.
   * @param  mode_costing          List of target/destination locations.
   * @param  mode                  Graph reader for accessing routing graph.
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   * @param  has_time              whether time-dependence was requested
   * @param  invariant             whether invariant time-dependence was requested
   * @param  shape_format          which shape_format, if any
   */
  virtual void SourceToTarget(Api& request,
                              baldr::GraphReader& graphreader,
                              const sif::mode_costing_t& mode_costing,
                              const sif::travel_mode_t mode,
                              const float max_matrix_distance) = 0;

  /**
   * Returns the name of the algorithm
   * @return the name of the algorithm
   */
  void set_has_time(const bool has_time) {
    has_time_ = has_time;
  };

  /**
   * Set a callback that will throw when the path computation should be aborted
   * @param interrupt_callback  the function to periodically call to see if
   *                            we should abort
   */
  void set_interrupt(const std::function<void()>* interrupt_callback) {
    interrupt_ = interrupt_callback;
  }

  /**
   * Sets the functor which will track the algorithms expansion.
   *
   * @param  expansion_callback  the functor to call back when the algorithm makes progress
   *                             on a given edge
   */
  using expansion_callback_t = std::function<void(baldr::GraphReader&,
                                                  const baldr::GraphId,
                                                  const baldr::GraphId,
                                                  const char*,
                                                  const char*,
                                                  float,
                                                  uint32_t,
                                                  float)>;
  void set_track_expansion(const expansion_callback_t& expansion_callback) {
    expansion_callback_ = expansion_callback;
  }

protected:
  const std::function<void()>* interrupt_;

  // whether time was specified
  bool has_time_;

  // for tracking the expansion of the algorithm visually
  expansion_callback_t expansion_callback_;

  uint32_t max_reserved_labels_count_;

  // if `true` clean reserved memory for edge labels
  bool clear_reserved_memory_;

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
    for (const auto& destination_edge : destination.correlation().edges()) {
      if (destination_edge.graph_id() == edgeid) {
        for (const auto& origin_edge : origin.correlation().edges()) {
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

} // namespace thor
} // namespace valhalla
