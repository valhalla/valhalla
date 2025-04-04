#ifndef VALHALLA_THOR_TIMEDISTANCEMATRIX_H_
#define VALHALLA_THOR_TIMEDISTANCEMATRIX_H_

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto_conversions.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/matrixalgorithm.h>

namespace valhalla {
namespace thor {

// Class to compute time + distance matrices among locations.
class TimeDistanceMatrix : public MatrixAlgorithm {
public:
  /**
   * Default constructor. Most internal values are set when a query is made so
   * the constructor mainly just sets some internals to a default empty value.
   */
  TimeDistanceMatrix(const boost::property_tree::ptree& config = {});

  /**
   * Forms a time distance matrix from the set of source locations
   * to the set of target locations.
   * @param  request               the full request
   * @param  graphreader           Graph reader for accessing routing graph.
   * @param  mode_costing          Costing methods.
   * @param  mode                  Travel mode to use.
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   *
   * @return time/distance from all sources to all targets
   */
  inline bool SourceToTarget(Api& request,
                             baldr::GraphReader& graphreader,
                             const sif::mode_costing_t& mode_costing,
                             const sif::travel_mode_t mode,
                             const float max_matrix_distance) override {

    request.mutable_matrix()->set_algorithm(Matrix::TimeDistanceMatrix);

    if (request.options().shape_format() != no_shape)
      add_warning(request, 207);

    // Set the mode and costing
    mode_ = mode;
    costing_ = mode_costing[static_cast<uint32_t>(mode_)];

    const bool forward_search =
        request.options().sources().size() <= request.options().targets().size();
    if (forward_search) {
      return ComputeMatrix<ExpansionType::forward>(request, graphreader, max_matrix_distance);
    } else {
      return ComputeMatrix<ExpansionType::reverse>(request, graphreader, max_matrix_distance);
    }
  };

  /**
   * Clear the temporary information generated during time+distance
   * matrix construction.
   */
  inline void Clear() override {
    auto reservation = clear_reserved_memory_ ? 0 : max_reserved_labels_count_;
    if (edgelabels_.size() > reservation) {
      edgelabels_.resize(reservation);
      edgelabels_.shrink_to_fit();
    }
    reset();
    destinations_.clear();
    dest_edges_.clear();
  };

  /**
   * Get the algorithm's name
   * @return the name of the algorithm
   */
  inline const std::string& name() override {
    return MatrixAlgoToString(Matrix::TimeDistanceMatrix);
  }

protected:
  // Number of destinations that have been found and settled (least cost path
  // computed).
  uint32_t settled_count_;

  // The cost threshold being used for the currently executing query
  float current_cost_threshold_;

  uint32_t max_reserved_labels_count_;

  // List of destinations
  std::vector<Destination> destinations_;

  // Current costing mode
  std::shared_ptr<sif::DynamicCost> costing_;

  // List of edges that have potential destinations. Each "marked" edge
  // has a vector of indexes into the destinations vector
  std::unordered_map<uint64_t, std::vector<uint32_t>> dest_edges_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::EdgeLabel> edgelabels_;

  // Adjacency list - approximate double bucket sort
  baldr::DoubleBucketQueue<sif::EdgeLabel> adjacencylist_;

  // Edge status. Mark edges that are in adjacency list or settled.
  EdgeStatus edgestatus_;

  sif::travel_mode_t mode_;

  // when doing timezone differencing a timezone cache speeds up the computation
  baldr::DateTime::tz_sys_info_cache_t tz_cache_;

  /**
   * Reset all origin-specific information
   */
  inline void reset() {
    // Clear the per-origin information
    for (auto& dest : destinations_) {
      dest.reset();
    }

    // Clear the edge labels
    edgelabels_.clear();

    // Clear elements from the adjacency list
    adjacencylist_.clear();

    // Clear the edge status flags
    edgestatus_.clear();
  };

  /**
   * Computes the matrix after SourceToTarget decided which direction
   * the algorithm should traverse.
   */
  template <const ExpansionType expansion_direction,
            const bool FORWARD = expansion_direction == ExpansionType::forward>
  bool ComputeMatrix(Api& request, baldr::GraphReader& graphreader, const float max_matrix_distance);

  /**
   * Expand from the node along the forward search path. Immediately expands
   * from the end node of any transition edge (so no transition edges are added
   * to the adjacency list or EdgeLabel list). Does not expand transition
   * edges if from_transition is false.
   * @param  graphreader  Graph tile reader.
   * @param  node         Graph Id of the node being expanded.
   * @param  pred         Predecessor edge label (for costing).
   * @param  pred_idx     Predecessor index into the EdgeLabel list.
   * @param  from_transition True if this method is called from a transition
   *                         edge.
   * @param  invariant    Whether invariant time was requested.
   */
  template <const ExpansionType expansion_direction,
            const bool FORWARD = expansion_direction == ExpansionType::forward>
  void Expand(baldr::GraphReader& graphreader,
              const baldr::GraphId& node,
              const sif::EdgeLabel& pred,
              const uint32_t pred_idx,
              const bool from_transition,
              const baldr::TimeInfo& time_info,
              const bool invariant = false);

  /**
   * Get the cost threshold based on the current mode and the max arc-length distance
   * for that mode.
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   */
  float GetCostThreshold(const float max_matrix_distance) const;

  /**
   * Sets the origin for a many to one time+distance matrix computation.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  origin        Origin location information.
   */
  template <const ExpansionType expansion_direction,
            const bool FORWARD = expansion_direction == ExpansionType::forward>
  void SetOrigin(baldr::GraphReader& graphreader,
                 const valhalla::Location& origin,
                 const baldr::TimeInfo& time_info);

  /**
   * Add destinations.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  locations     List of locations.
   */
  template <const ExpansionType expansion_direction,
            const bool FORWARD = expansion_direction == ExpansionType::forward>
  void InitDestinations(baldr::GraphReader& graphreader,
                        const google::protobuf::RepeatedPtrField<valhalla::Location>& locations);

  /**
   * Set the available destination edges for each origin.
   * @param locations List of destination locations.
   */
  void SetDestinationEdges() {
    // the percent_along is set once at the beginning
    for (auto& dest : destinations_) {
      for (const auto& idx : dest.dest_edges_percent_along) {
        dest.dest_edges_available.emplace(idx.first);
      }
    }
  };

  /**
   * Update destinations along an edge that has been settled (lowest cost path
   * found to the end of edge).
   * @param   origin        Location of the origin.
   * @param   locations     List of locations.
   * @param   destinations  Vector of destination indexes along this edge.
   * @param   edge          Directed edge
   * @param   pred          Predecessor information in shortest path.
   * @param   matrix_locations Count of locations that must be found. When provided it allows
   *                           a partial result to be returned (e.g. best 20 out of 50 locations).
   *                           When not supplied in the request this is set to max uint32_t value
   *                           so that all supplied locations must be settled.
   * @return  Returns true if all destinations have been settled.
   */
  bool UpdateDestinations(const valhalla::Location& origin,
                          const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                          std::vector<uint32_t>& destinations,
                          const baldr::DirectedEdge* edge,
                          const graph_tile_ptr& tile,
                          const sif::EdgeLabel& pred,
                          const baldr::TimeInfo& time_info,
                          const uint32_t matrix_locations);

  /**
   * Sets the date_time on the origin locations.
   *
   * @param origins            the origins (sources or targets)
   * @param reader             the reader for looking up timezone information
   * @returns                  time info for each location
   */
  std::vector<baldr::TimeInfo>
  SetTime(google::protobuf::RepeatedPtrField<valhalla::Location>& origins,
          baldr::GraphReader& reader) {
    // loop over all locations setting the date time with timezone
    std::vector<baldr::TimeInfo> infos;
    infos.reserve(origins.size());
    for (auto& origin : origins) {
      infos.emplace_back(baldr::TimeInfo::make(origin, reader, &tz_cache_));
    }

    return infos;
  };

  /**
   * Form a time/distance matrix from the results.
   *
   * @param request   The full request object
   * @param reader    GraphReader instance
   * @param origin_dt The origin's date_time string
   * @param origin_tz The origin's timezone index
   * @param pred_id   The destination edge's GraphId
   */
  void FormTimeDistanceMatrix(Api& request,
                              baldr::GraphReader& reader,
                              const bool forward,
                              const uint32_t origin_index,
                              const std::string& origin_dt,
                              const uint64_t& origin_tz,
                              std::unordered_map<uint32_t, baldr::GraphId>& edge_ids);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_TIMEDISTANCEMATRIX_H_
