#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/time_info.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/hierarchylimits.h>
#include <valhalla/thor/astarheuristic.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathalgorithm.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

/**
 * Forward direction A* algorithm to create the shortest / least cost path.
 * This algorithm is used for "depart-at", time-dependent routes.
 * For driving routes it uses a highway hierarchy with shortcut edges to
 * improve performance.
 */
template <const ExpansionType expansion_direction,
          const bool FORWARD = expansion_direction == ExpansionType::forward>
class UnidirectionalAStar : public PathAlgorithm {
public:
  /**
   * Constructor.
   * @param config A config object of key, value pairs
   */
  explicit UnidirectionalAStar(const boost::property_tree::ptree& config = {});

  /**
   * Form path between and origin and destination location using the supplied
   * costing method.
   * @param  origin       Origin location
   * @param  dest         Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  mode_costing Costing methods for each mode.
   * @param  mode         Travel mode to use.
   * @return Returns the path edges (and elapsed time/modes at end of
   *          each edge).
   */
  std::vector<std::vector<PathInfo>>
  GetBestPath(valhalla::Location& origin,
              valhalla::Location& dest,
              baldr::GraphReader& graphreader,
              const sif::mode_costing_t& mode_costing,
              const sif::TravelMode mode,
              const Options& options = Options::default_instance()) override;

  /**
   * Clear the temporary information generated during path construction.
   */
  void Clear() override;

  /**
   * Returns the name of the algorithm
   * @return the name of the algorithm
   */
  virtual const char* name() const override {
    if (FORWARD) {
      return "time_dependent_forward_a*";
    } else {
      return "time_dependent_reverse_a*";
    }
  }

  /**
   * Set a maximum label count. The path algorithm terminates if this
   * is exceeded.
   * @param  max_count  Maximum number of labels to allow.
   */
  void set_max_label_count(const uint32_t max_count) {
    max_label_count_ = max_count;
  }

protected:
  /**
   * Initializes the hierarchy limits, A* heuristic, and adjacency list.
   * @param  origll  Lat,lng of the origin.
   * @param  destll  Lat,lng of the destination.
   */
  void Init(const midgard::PointLL& origll, const midgard::PointLL& destll);

  /**
   * Expand from the node along the forward search path. Immediately expands
   * from the end node of any transition edge (so no transition edges are added
   * to the adjacency list or EdgeLabel list). Does not expand transition
   * edges if from_transition is false.
   * @param  graphreader  Graph tile reader.
   * @param  node         Graph Id of the node being expanded.
   * @param  pred         Predecessor edge label (for costing).
   * @param  pred_idx     Predecessor index into the EdgeLabel list.
   * @param  time_info    Tracks time offset as the route progresses
   * @param  dest         Location information of the destination.
   * @param  best_path    Best path found so far. Includes the index into
   *                      EdgeLabels and the cost.
   */
  bool Expand(baldr::GraphReader& graphreader,
              const baldr::GraphId& node,
              sif::BDEdgeLabel& pred,
              const uint32_t pred_idx,
              const baldr::DirectedEdge* opp_pred_edge,
              const baldr::TimeInfo& time_info,
              const valhalla::Location& dest,
              std::pair<int32_t, float>& best_path);

  // Private helper function for `ExpandReverse`
  inline bool ExpandInner(baldr::GraphReader& graphreader,
                          const sif::BDEdgeLabel& pred,
                          const baldr::DirectedEdge* opp_pred_edge,
                          const baldr::NodeInfo* nodeinfo,
                          const uint32_t pred_idx,
                          const EdgeMetadata& meta,
                          const graph_tile_ptr& tile,
                          const baldr::TimeInfo& time_info,
                          const valhalla::Location& destination,
                          std::pair<int32_t, float>& best_path);

  /**
   * Modify hierarchy limits based on distance between origin and destination
   * and the relative road density at the destination. For shorter routes
   * we stay on arterial roads further from the destination. Also for lower
   * road densities near the destination the hierarchy transition distances
   * are increased.
   * @param   dist     Distance between origin and destination.
   * @param   density  Relative road density near the destination.
   */
  void ModifyHierarchyLimits(const float dist, const uint32_t density);

  /**
   * Add edges at the origin to the adjacency list.
   * @param  graphreader  Graph tile reader.
   * @param  origin       Location information of the origin.
   * @param  dest         Location information of the destination.
   */

  void SetOrigin(baldr::GraphReader& graphreader,
                 const valhalla::Location& origin,
                 const valhalla::Location& destination,
                 const baldr::TimeInfo& time_info);

  /**
   * Set the destination edge(s).
   * @param   graphreader  Graph tile reader.
   * @param   dest         Location information of the destination.
   * @return  Returns the relative density near the destination (0-15)
   */
  uint32_t SetDestination(baldr::GraphReader& graphreader, const valhalla::Location& dest);

  /**
   * Form the path from the adjacency list. Recovers the path from the
   * destination backwards towards the origin (using predecessor information)
   * @param   dest  Index in the edge labels of the destination edge.
   * @return  Returns the path info, a list of GraphIds representing the
   *          directed edges along the path - ordered from origin to
   *          destination - along with travel modes and elapsed time.
   */
  std::vector<PathInfo> FormPath(const uint32_t dest);

  uint32_t max_label_count_; // Max label count to allow
  sif::TravelMode mode_;     // Current travel mode
  uint8_t travel_type_;      // Current travel type

  // Hierarchy limits.
  std::vector<sif::HierarchyLimits> hierarchy_limits_;

  // A* heuristic
  AStarHeuristic astarheuristic_;

  // Current costing mode
  std::shared_ptr<sif::DynamicCost> costing_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::BDEdgeLabel> edgelabels_;

  // Edge status. Mark edges that are in adjacency list or settled.
  EdgeStatus edgestatus_;

  // Destinations, id and percent used along the edge
  std::unordered_map<uint64_t, float> destinations_percent_along_;

  // Access mode used by the costing method
  uint32_t access_mode_;

  // Adjacency list - approximate double bucket sort
  baldr::DoubleBucketQueue<sif::BDEdgeLabel> adjacencylist_;
};

/**
 * Reverse direction A* algorithm to create the shortest / least cost path.
 * This algorithm is used for "arrive-by", time-dependent routes. For driving
 * routes it uses a highway hierarchy with shortcut edges to improve
 * performance.
 */

/*
template <const ExpansionType expansion_direction,
          const bool FORWARD = expansion_direction == ExpansionType::forward>
class TimeDepReverse : public UnidirectionalAStar<expansion_direction> {
public:
  explicit TimeDepReverse(const boost::property_tree::ptree& config = {});

  virtual std::vector<std::vector<PathInfo>>
  GetBestPath(valhalla::Location& origin,
              valhalla::Location& dest,
              baldr::GraphReader& graphreader,
              const sif::mode_costing_t& mode_costing,
              const sif::TravelMode mode,
              const Options& options = Options::default_instance()) override;

};
*/

using TimeDepForward = UnidirectionalAStar<ExpansionType::forward>;
using TimeDepReverse = UnidirectionalAStar<ExpansionType::reverse>;

} // namespace thor
} // namespace valhalla
