#ifndef __VALHALLA_THOR_MATRIXALGORITHM_H__
#define __VALHALLA_THOR_MATRIXALGORITHM_H__

#include <functional>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/api.pb.h>
// TODO(nils): should abstract more so we don't pull this in
#include <valhalla/thor/pathalgorithm.h>
#include <valhalla/worker.h>

namespace valhalla {
namespace thor {

// Default for time distance matrix is to find all locations
constexpr uint32_t kAllLocations = std::numeric_limits<uint32_t>::max();
constexpr float kMaxCost = 99999999.9999f;

/**
 * Pure virtual class defining the interface for MatrixAlgorithm
 */
class MatrixAlgorithm {
public:
  /**
   * Constructor
   */
  MatrixAlgorithm(const boost::property_tree::ptree& config)
      : interrupt_(nullptr), has_time_(false), expansion_callback_(),
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
   * Clear the temporary information.
   */
  virtual void Clear() = 0;

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

  // resizes all PBF sequences except for date_times
  inline static void reserve_pbf_arrays(valhalla::Matrix& matrix, size_t size) {
    matrix.mutable_from_indices()->Resize(size, 0U);
    matrix.mutable_to_indices()->Resize(size, 0U);
    matrix.mutable_distances()->Resize(size, 0U);
    matrix.mutable_times()->Resize(size, 0U);
    matrix.mutable_date_times()->Reserve(size);
    matrix.mutable_shapes()->Reserve(size);
    matrix.mutable_time_zone_offsets()->Reserve(size);
    matrix.mutable_time_zone_names()->Reserve(size);
  }
};

// Structure to hold information about each destination.
struct Destination {
  // per-origin information
  bool settled;        // Has the best time/distance to this destination
                       // been found?
  sif::Cost best_cost; // Current best cost to this destination
  // Set of still available correlated edges;
  std::unordered_set<uint64_t> dest_edges_available;

  // global information which only needs to be set once or is reset for every origin in the algorithm
  uint32_t distance; // Path distance for the best cost path
  float threshold;   // Threshold above current best cost where no longer
                     // need to search for this destination.
  // partial distance of correlated edges
  std::unordered_map<uint64_t, float> dest_edges_percent_along;

  // Constructor - set best_cost to an absurdly high value so any new cost
  // will be lower.
  Destination() : settled(false), best_cost{kMaxCost, kMaxCost}, distance(0), threshold(0.0f) {
  }

  // clears the per-origin information
  void reset() {
    settled = false;
    best_cost = {kMaxCost, kMaxCost};
    dest_edges_available.clear();
  }
};

// return true if any location had a valid time set
// return false if it doesn't make sense computationally and add warnings accordingly
inline bool check_matrix_time(Api& request, const Matrix::Algorithm algo) {
  const auto& options = request.options();
  bool less_sources = options.sources().size() <= options.targets().size();

  for (const auto& source : options.sources()) {
    if (!source.date_time().empty()) {
      if (!less_sources && algo == Matrix::TimeDistanceMatrix) {
        add_warning(request, 201);
        return false;
      }
      return true;
    }
  }
  for (const auto& target : options.targets()) {
    if (!target.date_time().empty()) {
      if (less_sources && algo == Matrix::TimeDistanceMatrix) {
        add_warning(request, 202);
        return false;
      } else if (algo == Matrix::CostMatrix) {
        add_warning(request, 206);
        return false;
      }
      return true;
    }
  }

  return false;
}

} // namespace thor
} // namespace valhalla

#endif // __VALHALLA_THOR_MATRIXALGORITHM_H__
