#ifndef VALHALLA_THOR_TIMEDISTANCEBSSMATRIX_H_
#define VALHALLA_THOR_TIMEDISTANCEBSSMATRIX_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/astarheuristic.h>
#include <valhalla/thor/costmatrix.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/matrix_common.h>
#include <valhalla/thor/pathalgorithm.h>

namespace valhalla {
namespace thor {

// Class to compute time + distance matrices among locations.
class TimeDistanceBSSMatrix {
public:
  /**
   * Default constructor. Most internal values are set when a query is made so
   * the constructor mainly just sets some internals to a default empty value.
   */
  TimeDistanceBSSMatrix();

  /**
   * One to many time and distance cost matrix. Computes time and distance
   * matrix from one origin location to many other locations.
   * @param  origin        Location of the origin.
   * @param  locations     List of locations.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  mode_costing  Costing methods.
   * @param  mode          Travel mode to use. Actually It doesn't make sense in matrix_bss, because
   * the travel mode must be pedestrian and bicycle
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   * @return time/distance from origin index to all other locations
   */
  std::vector<TimeDistance>
  OneToMany(const valhalla::Location& origin,
            const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
            baldr::GraphReader& graphreader,
            const sif::mode_costing_t& mode_costing,
            const sif::TravelMode /*mode*/,
            const float max_matrix_distance);

  /**
   * Many to one time and distance cost matrix. Computes time and distance
   * matrix from many locations to one destination location.
   * @param  dest          Location of the destination.
   * @param  locations     List of locations.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  mode_costing  Costing methods.
   * @param  mode          Travel mode to use. Actually It doesn't make sense in matrix_bss, because
   * the travel mode must be pedestrian and bicycle
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   * @return time/distance to the destination index from all other locations
   */
  std::vector<TimeDistance>
  ManyToOne(const valhalla::Location& dest,
            const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
            baldr::GraphReader& graphreader,
            const sif::mode_costing_t& mode_costing,
            const sif::TravelMode /*mode*/,
            const float max_matrix_distance);

  /**
   * Many to many time and distance cost matrix. Computes time and distance
   * matrix from many locations to many locations.
   * @param  locations     List of locations.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  mode_costing  Costing methods.
   * @param  mode          Travel mode to use. Actually It doesn't make sense in matrix_bss, because
   * the travel mode must be pedestrian and bicycle
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   * @return time/distance between all pairs of locations
   */
  std::vector<TimeDistance>
  ManyToMany(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
             baldr::GraphReader& graphreader,
             const sif::mode_costing_t& mode_costing,
             const sif::TravelMode /*mode*/,
             const float max_matrix_distance);

  /**
   * Forms a time distance matrix from the set of source locations
   * to the set of target locations.
   * @param  source_location_list  List of source/origin locations.
   * @param  target_location_list  List of target/destination locations.
   * @param  graphreader           Graph reader for accessing routing graph.
   * @param  mode_costing          Costing methods.
   * @param  mode                  Travel mode to use. Actually It doesn't make sense in matrix_bss,
   * because the travel mode must be pedestrian and bicycle
   * @param  max_matrix_distance   Maximum arc-length distance for current mode.
   * @return time/distance from origin index to all other locations
   */
  std::vector<TimeDistance>
  SourceToTarget(const google::protobuf::RepeatedPtrField<valhalla::Location>& source_location_list,
                 const google::protobuf::RepeatedPtrField<valhalla::Location>& target_location_list,
                 baldr::GraphReader& graphreader,
                 const sif::mode_costing_t& mode_costing,
                 const sif::TravelMode /*mode*/,
                 const float max_matrix_distance);

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

  // A* heuristic
  AStarHeuristic pedestrian_astarheuristic_;
  AStarHeuristic bicycle_astarheuristic_;

  // Current costing mode
  std::shared_ptr<sif::DynamicCost> pedestrian_costing_;
  std::shared_ptr<sif::DynamicCost> bicycle_costing_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::EdgeLabel> edgelabels_;

  // Adjacency list - approximate double bucket sort
  baldr::DoubleBucketQueue<sif::EdgeLabel> adjacencylist_;

  // Edge status. Mark edges that are in adjacency list or settled.
  EdgeStatus pedestrian_edgestatus_;
  EdgeStatus bicycle_edgestatus_;

  // List of destinations
  std::vector<Destination> destinations_;

  // List of edges that have potential destinations. Each "marked" edge
  // has a vector of indexes into the destinations vector
  std::unordered_map<uint64_t, std::vector<uint32_t>> dest_edges_;

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
   * @param  from_bss     Is this Expansion done from a bike share station?
   * @param  mode         the current travel mode
   */
  void ExpandForward(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     const sif::EdgeLabel& pred,
                     const uint32_t pred_idx,
                     const bool from_transition,
                     const bool from_bss,
                     const sif::TravelMode mode);

  /**
   * Expand from the node along the reverse search path. Immediately expands
   * from the end node of any transition edge (so no transition edges are added
   * to the adjacency list or EdgeLabel list). Does not expand transition
   * edges if from_transition is false.
   * @param  graphreader  Graph tile reader.
   * @param  node         Graph Id of the node being expanded.
   * @param  pred         Predecessor edge label (for costing).
   * @param  pred_idx     Predecessor index into the EdgeLabel list.
   * @param  from_transition True if this method is called from a transition
   *                         edge.
   * @param  from_bss     Is this Expansion done from a bike share station?
   * @param  mode         the current travel mode
   */
  void ExpandReverse(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     const sif::EdgeLabel& pred,
                     const uint32_t pred_idx,
                     const bool from_transition,
                     const bool from_bss,
                     const sif::TravelMode mode);

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
  void SetOriginOneToMany(baldr::GraphReader& graphreader, const valhalla::Location& origin);

  /**
   * Sets the origin for a many to one time+distance matrix computation.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  dest          Destination
   */
  void SetOriginManyToOne(baldr::GraphReader& graphreader, const valhalla::Location& dest);

  /**
   * Add destinations.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  locations     List of locations.
   */
  void
  SetDestinationsOneToMany(baldr::GraphReader& graphreader,
                           const google::protobuf::RepeatedPtrField<valhalla::Location>& locations);

  /**
   * Set destinations for the many to one time+distance matrix computation.
   * @param  graphreader   Graph reader for accessing routing graph.
   * @param  locations     List of locations.
   */
  void
  SetDestinationsManyToOne(baldr::GraphReader& graphreader,
                           const google::protobuf::RepeatedPtrField<valhalla::Location>& locations);

  /**
   * Update destinations along an edge that has been settled (lowest cost path
   * found to the end of edge).
   * @param   origin        Location of the origin.
   * @param   locations     List of locations.
   * @param   destinations  Vector of destination indexes along this edge.
   * @param   edge          Directed edge
   * @param   pred          Predecessor information in shortest path.
   * @param   predindex     Predecessor index in EdgeLabels vector.
   * @return  Returns true if all destinations have been settled.
   */
  bool UpdateDestinations(const valhalla::Location& origin,
                          const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                          std::vector<uint32_t>& destinations,
                          const baldr::DirectedEdge* edge,
                          const graph_tile_ptr& tile,
                          const sif::EdgeLabel& pred);

  /**
   * Form a time/distance matrix from the results.
   * @return  Returns a time distance matrix among locations.
   */
  std::vector<TimeDistance> FormTimeDistanceMatrix();
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_TIMEDISTANCEMATRIX_H_
