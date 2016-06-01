#ifndef VALHALLA_THOR_COSTMATRIX_H_
#define VALHALLA_THOR_COSTMATRIX_H_

#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <memory>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/adjacencylist.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/timedistancematrix.h>

namespace valhalla {
namespace thor {

// Use a 4 hour cost threshold. This is in addition to the distance
// thresholds for quick rejection
constexpr float kCostThresholdDefault = 14400.0f;   // 4 hours

/**
 * Status of a location. Tracks remaining locations to be found
 * and a threshold or iterations. When threshold goes to 0 expansion
 * stops for this location.
 */
struct LocationStatus {
  int  threshold;
  std::set<uint32_t> remaining_locations;

  LocationStatus(const int t)
      : threshold(t) {
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
  uint32_t threshold;

  BestCandidate(const baldr::GraphId& e1, baldr::GraphId& e2,
                      const sif::Cost& c, const uint32_t d)
      : found(false),
        edgeid(e1),
        opp_edgeid(e2),
        cost(c),
        distance(d),
        threshold(0) {
  }

  void Update(const baldr::GraphId& e1, baldr::GraphId& e2,
              const sif::Cost& c, const uint32_t d) {
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
class CostMatrix {
 public:
  /**
   * Constructor with cost threshold.
   * @param initial_cost_threshold  Cost threshold for termination.
   */
  CostMatrix(float initial_cost_threshold = kCostThresholdDefault);

  /**
   * Forms a time distance matrix from the set of source locations
   * to the set of target locations.
   * @param  source_location_list  List of source/origin locations.
   * @param  target_location_list  List of target/destination locations.
   * @param  graphreader           Graph reader for accessing routing graph.
   * @param  costing               Costing methods.
   * @param  mode                  Travel mode to use.
   * @return time/distance from origin index to all other locations
   */
  std::vector<TimeDistance> SourceToTarget(
          const std::vector<baldr::PathLocation>& source_location_list,
          const std::vector<baldr::PathLocation>& target_location_list,
          baldr::GraphReader& graphreader,
          const std::shared_ptr<sif::DynamicCost>* mode_costing,
          const sif::TravelMode mode);

  /**
   * Clear the temporary information generated during time+distance
   * matrix construction.
   */
  void Clear();

 protected:
  // Allow transitions (set from the costing model)
  bool allow_transitions_;

  // Current travel mode
  sif::TravelMode mode_;

  // Number of source and target locations that can be expanded
  uint32_t source_count_;
  uint32_t remaining_sources_;
  uint32_t target_count_;
  uint32_t remaining_targets_;

  // Cost threshold - stop searches when this is reached.
  float cost_threshold_;

  // Status
  std::vector<LocationStatus> source_status_;
  std::vector<LocationStatus> target_status_;

  // Adjacency lists, EdgeLabels, EdgeStatus, and hierarchy limits for each
  // source location (forward traversal)
  std::vector<std::vector<sif::HierarchyLimits>> source_hierarchy_limits_;
  std::vector<std::shared_ptr<AdjacencyList>> source_adjacency_;
  std::vector<std::vector<sif::EdgeLabel>> source_edgelabel_;
  std::vector<EdgeStatus> source_edgestatus_;

  // Adjacency lists, EdgeLabels, EdgeStatus, and hierarchy limits for each
  // target location (reverse traversal)
  std::vector<std::vector<sif::HierarchyLimits>> target_hierarchy_limits_;
  std::vector<std::shared_ptr<AdjacencyList>> target_adjacency_;
  std::vector<std::vector<sif::EdgeLabel>> target_edgelabel_;
  std::vector<EdgeStatus> target_edgestatus_;

  // Mark each target edge with a list of target indexes that have reached it
  std::unordered_map<baldr::GraphId, std::vector<uint32_t>> targets_;

  // List of best connections found so far
  std::vector<BestCandidate> best_connection_;

  /**
   * Form the initial time distance matrix given the sources
   * and destinations.
   * @param  source_location_list   List of source/origin locations.
   * @param  target_location_list   List of target/destination locations.
   * @return Returns the initial time distance matrix.
   */
  void Initialize(
      const std::vector<baldr::PathLocation>& source_location_list,
      const std::vector<baldr::PathLocation>& target_location_list);

  /**
   * Iterate the forward search from the source/origin location.
   * @param  index        Index of the source location.
   * @param  n            Iteration counter.
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  costing      Costing methods.
   */
  void ForwardSearch(const uint32_t index, const uint32_t n,
                     baldr::GraphReader& graphreader,
                     const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Check if the edge on the forward search connects to a reached edge
   * on the reverse search tree.
   * @param  source  Source index.
   * @param  pred    Edge label of the predecessor.
   * @param  n            Iteration counter.
   */
  void CheckForwardConnections(const uint32_t source,
                               const sif::EdgeLabel& pred, const uint32_t n);

  /**
   * Update status when a connection is found.
   * @param  source  Source index
   * @param  target  Target index
   */
  void UpdateStatus(const uint32_t source_, const uint32_t target);

  /**
   * Iterate the backward search from the target/destination location.
   * @param  index        Index of the target location.
   * @param  graphreader  Graph reader for accessing routing graph.
   */
  void BackwardSearch(const uint32_t index,
                      baldr::GraphReader& graphreader,
                      const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Sets the source/origin locations. Search expands forward from these
   * locations.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  sources       List of source/origin locations.
   * @param  costing       Costing method.
   */
  void SetSources(baldr::GraphReader& graphreader,
                  const std::vector<baldr::PathLocation>& sources,
                  const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Set the target/destination locations. Search expands backwards from
   * these locations.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  locations     List of target locations.
   * @param  costing       Costing method.
   */
  void SetTargets(baldr::GraphReader& graphreader,
                  const std::vector<baldr::PathLocation>& targets,
                  const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Update destinations along an edge that has been settled (lowest cost path
   * found to the end of edge).
   * @param   origin_index  Index of the origin location.
   * @param   locations     List of locations.
   * @param   destinations  Vector of destination indexes along this edge.
   * @param   edge          Directed edge
   * @param   pred          Predecessor information in shortest path.
   * @param   predindex     Predecessor index in EdgeLabels vector.
   * @param   costing       Costing method.
   * @return  Returns true if all destinations have been settled.
   */
  bool UpdateDestinations(const uint32_t origin_index,
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
};

}
}

#endif  // VALHALLA_THOR_COSTMATRIX_H_
