#ifndef VALHALLA_THOR_TIMEDISTANCEMATRIX_H_
#define VALHALLA_THOR_TIMEDISTANCEMATRIX_H_

#include <cstdint>
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
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathalgorithm.h>
#include <valhalla/thor/costmatrix.h>
#include <valhalla/thor/astar.h>

namespace valhalla {
namespace thor {

// These cost thresholds are in addition to the distance
// thresholds for quick rejection
constexpr float kTimeDistCostThresholdAutoDivisor = 112.0f; // 400 km distance threshold will result in a cost threshold of ~2600 (1 hour)
constexpr float kTimeDistCostThresholdBicycleDivisor = 19.0f; // 200 km distance threshold will result in a cost threshold of ~10800 (3 hours)
constexpr float kTimeDistCostThresholdPedestrianDivisor = 9.0f; // 200 km distance threshold will result in a cost threshold of ~21600 (6 hours)

// Structure to hold information about each destination.
struct Destination {
  bool settled;         // Has the best time/distance to this destination
                        // been found?
  sif::Cost best_cost;  // Current best cost to this destination
  uint32_t distance;    // Path distance for the best cost path
  float threshold;      // Threshold above current best cost where no longer
                        // need to search for this destination.

  // Potential edges for this destination (and their partial distance)
  std::unordered_map<baldr::GraphId, float> dest_edges;

  // Constructor - set best_cost to an absurdly high value so any new cost
  // will be lower.
  Destination()
      : settled(false),
        best_cost{ kMaxCost, kMaxCost },
        distance(0),
        threshold(0.0f) {
  }
};

// Class to compute time + distance matrices among locations.
class TimeDistanceMatrix {
 public:

  /**
   * Default constructor. Most internal values are set when a query is made so
   * the constructor mainly just sets some internals to a default empty value.
   */
  TimeDistanceMatrix();

  /**
   * One to many time and distance cost matrix. Computes time and distance
   * matrix from one origin location to many other locations.
   * @param  origin                Location of the origin.
   * @param  locations             List of locations.
   * @param  graphreader           Graph reader for accessing routing graph.
   * @param  costing               Costing methods.
   * @param  mode                  Travel mode to use.
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   * @param  ignore_list           List of source + target pairs to ignore. (return empty)
   * @param  origin_ignore_id      The id of the origin inside the ignore_list
   * @return time/distance from origin index to all other locations
   */
  std::vector<TimeDistance> OneToMany(const baldr::PathLocation& origin,
          const std::vector<baldr::PathLocation>& locations,
          baldr::GraphReader& graphreader,
          const std::shared_ptr<sif::DynamicCost>* mode_costing,
          const sif::TravelMode mode, const float max_matrix_distance,
          const ignore_list_t& ignore_list = {},
          const uint32_t origin_ignore_id = 0);

  /**
   * Many to one time and distance cost matrix. Computes time and distance
   * matrix from many locations to one destination location.
   * @param  dest                  Location of the destination.
   * @param  locations             List of locations.
   * @param  graphreader           Graph reader for accessing routing graph.
   * @param  costing               Costing methods.
   * @param  mode                  Travel mode to use.
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   * @param  ignore_list           List of source + target pairs to ignore. (return empty)
   * @param  dest_ignore_id        The id of the destination inside the ignore_list
   * @return time/distance to the destination index from all other locations
   */
  std::vector<TimeDistance> ManyToOne(const baldr::PathLocation& dest,
          const std::vector<baldr::PathLocation>& locations,
          baldr::GraphReader& graphreader,
          const std::shared_ptr<sif::DynamicCost>* mode_costing,
          const sif::TravelMode mode, const float max_matrix_distance,
          const ignore_list_t& ignore_list = {},
          const uint32_t dest_ignore_id = 0);

  /**
   * Many to many time and distance cost matrix. Computes time and distance
   * matrix from many locations to many locations.
   * @param  locations             List of locations.
   * @param  graphreader           Graph reader for accessing routing graph.
   * @param  costing               Costing methods.
   * @param  mode                  Travel mode to use.
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   * @param  ignore_list           List of source + target pairs to ignore. (return empty)
   * @return time/distance between all pairs of locations
   */
  std::vector<TimeDistance> ManyToMany(
          const std::vector<baldr::PathLocation>& locations,
          baldr::GraphReader& graphreader,
          const std::shared_ptr<sif::DynamicCost>* mode_costing,
          const sif::TravelMode mode, const float max_matrix_distance,
          const ignore_list_t& ignore_list = {});

  /**
   * Forms a time distance matrix from the set of source locations
   * to the set of target locations.
   * @param  source_location_list  List of source/origin locations.
   * @param  target_location_list  List of target/destination locations.
   * @param  graphreader           Graph reader for accessing routing graph.
   * @param  costing               Costing methods.
   * @param  mode                  Travel mode to use.
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   * @param  ignore_list           List of source + target pairs to ignore. (return empty)
   * @return time/distance from origin index to all other locations
   */
  std::vector<TimeDistance> SourceToTarget(
          const std::vector<baldr::PathLocation>& source_location_list,
          const std::vector<baldr::PathLocation>& target_location_list,
          baldr::GraphReader& graphreader,
          const std::shared_ptr<sif::DynamicCost>* mode_costing,
          const sif::TravelMode mode, const float max_matrix_distance,
          const ignore_list_t& ignore_list = {});

  /**
   * Clear the temporary information generated during time+distance
   * matrix construction.
   */
  void Clear();

 protected:
  // Number of destinations that have been found and settled (least cost path
  // computed).
  uint32_t settled_count_;

  // The cost threshold being used for the currently executing query
  float current_cost_threshold_;

  // List of destinations
  std::vector<Destination> destinations_;

  // List of edges that have potential destinations. Each "marked" edge
  // has a vector of indexes into the destinations vector
  std::unordered_map<baldr::GraphId, std::vector<uint32_t>> dest_edges_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::EdgeLabel> edgelabels_;

  // Adjacency list - approximate double bucket sort
  std::shared_ptr<baldr::DoubleBucketQueue> adjacencylist_;

  // Edge status. Mark edges that are in adjacency list or settled.
  std::shared_ptr<EdgeStatus> edgestatus_;

  AStarHeuristic astarheuristic_;

  sif::TravelMode mode_;

  /**
   * Get the cost threshold based on the current mode and the max arc-length distance
   * for that mode.
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   */
  float GetCostThreshold (const float max_matrix_distance);

  /**
   * Sets the origin for a many to one time+distance matrix computation.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  origin        Origin location information.
   * @param  costing       Costing method.
   */
  void SetOriginOneToMany(baldr::GraphReader& graphreader,
                   const baldr::PathLocation& origin,
                   const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Sets the origin for a many to one time+distance matrix computation.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  dest          Destination
   * @param  costing       Costing method.
   */
  void SetOriginManyToOne(baldr::GraphReader& graphreader,
                        const baldr::PathLocation& dest,
                        const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Add destinations.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  locations     List of locations.
   * @param  costing       Costing method.
   * @param  ignore_ids    The ids of the destinations to ignore
   */
  void SetDestinations(baldr::GraphReader& graphreader,
                       const std::vector<baldr::PathLocation>& locations,
                       const std::shared_ptr<sif::DynamicCost>& costing,
                       const std::unordered_set<uint32_t>& ignore_ids);

  /**
   * Set destinations for the many to one time+distance matrix computation.
   * @param  graphreader           Graph reader for accessing routing graph.
   * @param  locations             List of locations.
   * @param  costing               Costing method.
   * @param  ignore_list           List of source + target pairs to ignore. (return empty)
   * @param  dest_ignore_id        The id of the destination inside the ignore_list
   */
  void SetDestinationsManyToOne(baldr::GraphReader& graphreader,
            const std::vector<baldr::PathLocation>& locations,
            const std::shared_ptr<sif::DynamicCost>& costing,
            const ignore_list_t& ignore_list,
            const uint32_t dest_ignore_id);

  /**
   * Update destinations along an edge that has been settled (lowest cost path
   * found to the end of edge).
   * @param   origin        Location of the origin.
   * @param   locations     List of locations.
   * @param   destinations  Vector of destination indexes along this edge.
   * @param   edge          Directed edge
   * @param   pred          Predecessor information in shortest path.
   * @param   predindex     Predecessor index in EdgeLabels vector.
   * @param   costing       Costing method.
   * @return  Returns true if all destinations have been settled.
   */
  bool UpdateDestinations(const baldr::PathLocation& origin,
                          const std::vector<baldr::PathLocation>& locations,
                          std::vector<uint32_t>& destinations,
                          const baldr::DirectedEdge* edge,
                          const sif::EdgeLabel& pred,
                          const uint32_t predindex,
                          const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Form a time/distance matrix from the results.
   * @return  Returns a time distance matrix among locations.
   */
  std::vector<TimeDistance> FormTimeDistanceMatrix();

  void AddToAdjacencyList(const baldr::GraphId& edgeid, const float sortcost);
};

}
}

#endif  // VALHALLA_THOR_TIMEDISTANCEMATRIX_H_
