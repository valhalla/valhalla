#ifndef VALHALLA_THOR_COSTMATRIX_H_
#define VALHALLA_THOR_COSTMATRIX_H_

#include <cstdint>
#include <memory>
#include <set>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/common.pb.h>
#include <valhalla/proto_conversions.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/astarheuristic.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/matrixalgorithm.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

enum class MatrixExpansionType { reverse = 0, forward = 1 };
constexpr bool MATRIX_FORW = static_cast<bool>(MatrixExpansionType::forward);
constexpr bool MATRIX_REV = static_cast<bool>(MatrixExpansionType::reverse);

// These cost thresholds are in addition to the distance thresholds. If either forward or reverse
// costs exceed the threshold the search is terminated.
constexpr float kCostThresholdAutoDivisor =
    56.0f; // 400 km distance threshold will result in a cost threshold of ~7200 (2 hours)
constexpr float kCostThresholdBicycleDivisor =
    56.0f; // 200 km distance threshold will result in a cost threshold of ~3600 (1 hour)
constexpr float kCostThresholdPedestrianDivisor =
    28.0f; // 200 km distance threshold will result in a cost threshold of ~7200 (2 hours)

/**
 * Status of a location. Tracks remaining locations to be found
 * and a threshold or iterations. When threshold goes to 0 expansion
 * stops for this location.
 */
struct LocationStatus {
  int threshold;
  std::set<uint32_t> unfound_connections;

  LocationStatus(const int t) : threshold(t) {
  }
};

/**
 * Best connection. Information about the best connection found between
 * a source and target pair.
 */
struct BestCandidate {
  bool found;
  baldr::GraphId edgeid;
  baldr::GraphId opp_edgeid;
  sif::Cost cost;
  uint32_t distance;
  uint32_t max_iterations;

  BestCandidate(const baldr::GraphId& e1, baldr::GraphId& e2, const sif::Cost& c, const uint32_t d)
      : found(false), edgeid(e1), opp_edgeid(e2), cost(c), distance(d), max_iterations(0) {
  }

  void
  Update(const baldr::GraphId& e1, const baldr::GraphId& e2, const sif::Cost& c, const uint32_t d) {
    edgeid = e1;
    opp_edgeid = e2;
    cost = c;
    distance = d;
  }
};

/**
 * Class to compute cost (cost + time + distance) matrices among locations.
 * This uses a bidirectional search with highway hierarchies. This is a
 * method described by Sebastian Knopp, "Efficient Computation of Many-to-Many
 * Shortest Paths".
 * https://i11www.iti.uni-karlsruhe.de/_media/teaching/theses/files/da-sknopp-06.pdf
 */
class CostMatrix : public MatrixAlgorithm {
public:
  /**
   * Default constructor. Most internal values are set when a query is made so
   * the constructor mainly just sets some internals to a default empty value.
   */
  CostMatrix(const boost::property_tree::ptree& config = {});

  ~CostMatrix();

  /**
   * Forms a time distance matrix from the set of source locations
   * to the set of target locations.
   * @param  request               the full request
   * @param  graphreader           List of source/origin locations.
   * @param  mode_costing          List of target/destination locations.
   * @param  mode                  Graph reader for accessing routing graph.
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   */
  bool SourceToTarget(Api& request,
                      baldr::GraphReader& graphreader,
                      const sif::mode_costing_t& mode_costing,
                      const sif::travel_mode_t mode,
                      const float max_matrix_distance) override;

  /**
   * Clear the temporary information generated during time+distance
   * matrix construction.
   */
  void Clear() override;

  /**
   * Get the algorithm's name
   * @return the name of the algorithm
   */
  inline const std::string& name() override {
    return MatrixAlgoToString(Matrix::CostMatrix);
  }

protected:
  uint32_t max_reserved_labels_count_;
  uint32_t max_reserved_locations_count_;
  bool check_reverse_connections_;

  // Access mode used by the costing method
  uint32_t access_mode_;

  // Current travel mode
  sif::TravelMode mode_;

  // Current costing mode
  std::shared_ptr<sif::DynamicCost> costing_;

  // TODO(nils): instead of these array based structures, rather do this:
  // https://github.com/valhalla/valhalla/pull/4372#discussion_r1402163444
  // Number of source and target locations that can be expanded
  std::array<uint32_t, 2> locs_count_;
  std::array<uint32_t, 2> locs_remaining_;

  // The path distance threshold being used for the currently executing query
  float current_pathdist_threshold_;

  // Status
  std::array<std::vector<LocationStatus>, 2> locs_status_;

  // Adjacency lists, EdgeLabels, EdgeStatus, and hierarchy limits for each location
  std::array<std::vector<std::vector<sif::HierarchyLimits>>, 2> hierarchy_limits_;
  std::array<std::vector<baldr::DoubleBucketQueue<sif::BDEdgeLabel>>, 2> adjacency_;
  std::array<std::vector<std::vector<sif::BDEdgeLabel>>, 2> edgelabel_;
  std::array<std::vector<EdgeStatus>, 2> edgestatus_;

  // A* heuristics for both trees and each location
  std::array<std::vector<AStarHeuristic>, 2> astar_heuristics_;

  // List of best connections found so far
  std::vector<BestCandidate> best_connection_;

  bool ignore_hierarchy_limits_;

  // when doing timezone differencing a timezone cache speeds up the computation
  baldr::DateTime::tz_sys_info_cache_t tz_cache_;

  /**
   * Form the initial time distance matrix given the sources
   * and destinations.
   * @param  source_location_list   List of source/origin locations.
   * @param  target_location_list   List of target/destination locations.
   */
  void Initialize(const google::protobuf::RepeatedPtrField<valhalla::Location>& source_location_list,
                  const google::protobuf::RepeatedPtrField<valhalla::Location>& target_location_list,
                  const valhalla::Matrix& matrix);

  /**
   * Iterate the forward search from the source/origin location.
   * @param  index        Index of the source location.
   * @param  n            Iteration counter.
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  time_info    The origin's timeinfo object
   * @param  invariant    Whether time should be treated as invariant
   */
  void ForwardSearch(const uint32_t index,
                     const uint32_t n,
                     baldr::GraphReader& graphreader,
                     const baldr::TimeInfo& time_info,
                     const bool invariant);

  /**
   * Check if the edge on the forward search connects to a reached edge
   * on the reverse search tree.
   * @param  source  Source index.
   * @param  pred    Edge label of the predecessor.
   * @param  n       Iteration counter.
   * @param  graphreader the graph reader instance
   * @param  options     the request options to check for the position along origin and destination
   *                     edges
   */
  void CheckForwardConnections(const uint32_t source,
                               const sif::BDEdgeLabel& pred,
                               const uint32_t n,
                               baldr::GraphReader& graphreader,
                               const valhalla::Options& options);

  template <const MatrixExpansionType expansion_direction,
            const bool FORWARD = expansion_direction == MatrixExpansionType::forward>
  bool Expand(const uint32_t index,
              const uint32_t n,
              baldr::GraphReader& graphreader,
              const valhalla::Options& options,
              const baldr::TimeInfo& time_info = baldr::TimeInfo::invalid(),
              const bool invariant = false);

  template <const MatrixExpansionType expansion_direction,
            const bool FORWARD = expansion_direction == MatrixExpansionType::forward>
  bool ExpandInner(baldr::GraphReader& graphreader,
                   const uint32_t index,
                   const sif::BDEdgeLabel& pred,
                   const baldr::DirectedEdge* opp_pred_edge,
                   const baldr::NodeInfo* nodeinfo,
                   const uint32_t pred_idx,
                   const EdgeMetadata& meta,
                   uint32_t& shortcuts,
                   const graph_tile_ptr& tile,
                   const baldr::TimeInfo& time_info);

  /**
   * Check if the edge on the backward search connects to a reached edge
   * on the reverse search tree.
   * @param  target      target index.
   * @param  pred        Edge label of the predecessor.
   * @param  n           Iteration counter.
   * @param  graphreader the graph reader instance
   * @param  options     the request options to check for the position along origin and destination
   *                     edges
   */
  void CheckReverseConnections(const uint32_t target,
                               const sif::BDEdgeLabel& pred,
                               const uint32_t n,
                               baldr::GraphReader& graphreader,
                               const valhalla::Options& options);

  /**
   * Update status when a connection is found.
   * @param  source  Source index
   * @param  target  Target index
   */
  void UpdateStatus(const uint32_t source, const uint32_t target);

  /**
   * Iterate the backward search from the target/destination location.
   * @param  index        Index of the target location.
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  n            Iteration counter.
   */
  void BackwardSearch(const uint32_t index, baldr::GraphReader& graphreader, const uint32_t n);

  /**
   * Sets the source/origin locations. Search expands forward from these
   * locations.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  sources       List of source/origin locations.
   */
  void SetSources(baldr::GraphReader& graphreader,
                  const google::protobuf::RepeatedPtrField<valhalla::Location>& sources,
                  const std::vector<baldr::TimeInfo>& time_infos);

  /**
   * Set the target/destination locations. Search expands backwards from
   * these locations.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  targets       List of target locations.
   */
  void SetTargets(baldr::GraphReader& graphreader,
                  const google::protobuf::RepeatedPtrField<valhalla::Location>& targets);

  /**
   * Update destinations along an edge that has been settled (lowest cost path
   * found to the end of edge).
   * @param   origin_index  Index of the origin location.
   * @param   locations     List of locations.
   * @param   destinations  Vector of destination indexes along this edge.
   * @param   edge          Directed edge
   * @param   pred          Predecessor information in shortest path.
   * @param   predindex     Predecessor index in EdgeLabels vector.
   * @return  Returns true if all destinations have been settled.
   */
  bool UpdateDestinations(const uint32_t origin_index,
                          const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                          std::vector<uint32_t>& destinations,
                          const baldr::DirectedEdge* edge,
                          const sif::BDEdgeLabel& pred,
                          const uint32_t predindex);

  /**
   * If time awareness was requested for the CostMatrix algorithm, we need
   * to form the paths the sources & targets generated, and recost them to
   * update the best connections, before returning the result. Optionally
   * returns the path's shape.
   * @param   graphreader  Graph tile reader
   * @param   origins      The source locations
   * @param   targets      The target locations
   * @param   time_infos   The time info objects for the sources
   * @param   invariant    Whether time is invariant
   * @return  optionally the path's shape or ""
   */
  std::string RecostFormPath(baldr::GraphReader& graphreader,
                             BestCandidate& connection,
                             const valhalla::Location& source,
                             const valhalla::Location& target,
                             const uint32_t source_idx,
                             const uint32_t target_idx,
                             const baldr::TimeInfo& time_info,
                             const bool invariant,
                             const ShapeFormat shape_format);

  /**
   * Sets the date_time on the origin locations.
   *
   * @param origins            the origins (sources or targets)
   * @param reader             the reader for looking up timezone information
   * @returns                  time info for each location
   */
  std::vector<baldr::TimeInfo>
  SetOriginTimes(google::protobuf::RepeatedPtrField<valhalla::Location>& origins,
                 baldr::GraphReader& reader) {
    // loop over all locations setting the date time with timezone
    std::vector<baldr::TimeInfo> infos;
    infos.reserve(origins.size());
    for (auto& origin : origins) {
      infos.emplace_back(baldr::TimeInfo::make(origin, reader, &tz_cache_));
    }

    return infos;
  };

  void ModifyHierarchyLimits() {
    // Distance threshold optimized for unidirectional search. For bidirectional case
    // they can be lowered.
    // Decrease distance thresholds only for arterial roads for now
    for (size_t source = 0; source < locs_count_[MATRIX_FORW]; source++) {
      if (hierarchy_limits_[MATRIX_FORW][source][1].max_up_transitions != kUnlimitedTransitions)
        hierarchy_limits_[MATRIX_FORW][source][1].expansion_within_dist /= 2.f;
    }
    for (size_t target = 0; target < locs_count_[MATRIX_REV]; target++) {
      if (hierarchy_limits_[MATRIX_REV][target][1].max_up_transitions != kUnlimitedTransitions)
        hierarchy_limits_[MATRIX_REV][target][1].expansion_within_dist /= 2.f;
    }
  };

  /**
   * Get the minimum AStar heuristic for a given source/target, i.e. for a source we get
   * the minimum heuristic of all targets for the forward expansion, so that we direct
   * the search towards the closest target/source.
   *
   * @param loc_idx  either the source or target index
   * @param node_ll  the current edge's end node's lat/lon
   * @returns The heuristic for the closest target/source of the passed node
   */
  template <const MatrixExpansionType expansion_direction,
            const bool FORWARD = expansion_direction == MatrixExpansionType::forward>
  float GetAstarHeuristic(const uint32_t loc_idx, const PointLL& node_ll) const;

private:
  class ReachedMap;

  // Mark each source/target edge with a list of source/target indexes that have reached it
  std::unique_ptr<ReachedMap> targets_;
  std::unique_ptr<ReachedMap> sources_;
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_COSTMATRIX_H_
