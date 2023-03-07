#ifndef VALHALLA_THOR_MULTIMODAL_H_
#define VALHALLA_THOR_MULTIMODAL_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/time_info.h>
#include <valhalla/proto/common.pb.h>
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
 * Multi-modal pathfinding algorithm. Currently supports walking and
 * transit (bus, subway, light-rail, etc.).
 */
class MultiModalPathAlgorithm : public PathAlgorithm {
public:
  /**
   * Constructor.
   * @param config A config object of key, value pairs
   */
  explicit MultiModalPathAlgorithm(const boost::property_tree::ptree& config = {});

  /**
   * Destructor
   */
  virtual ~MultiModalPathAlgorithm();

  /**
   * Form multi-modal path between and origin and destination location using
   * the supplied costing method.
   * @param  origin  Origin location
   * @param  dest    Destination location
   * @param  graphreader  Graph reader for accessing routing graph.
   * @param  mode_costing  An array of costing methods, one per TravelMode.
   * @param  mode     Travel mode from the origin.
   * @return  Returns the path edges (and elapsed time/modes at end of
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
   * Returns the name of the algorithm
   * @return the name of the algorithm
   */
  virtual const char* name() const override {
    return "Multimodal";
  }

  /**
   * Clear the temporary information generated during path construction.
   */
  void Clear() override;

protected:
  uint32_t max_walking_dist_;
  uint32_t max_label_count_; // Max label count to allow
  sif::TravelMode mode_;     // Current travel mode
  uint8_t travel_type_;      // Current travel type

  bool date_set_;
  bool date_before_tile_;
  bool disable_transit_;
  uint32_t date_;
  uint32_t dow_;
  uint32_t day_;
  uint32_t start_time_;
  uint32_t max_seconds_;
  uint32_t max_transfer_distance_;
  std::string origin_date_time_;
  std::unordered_map<std::string, uint32_t> operators_;
  std::unordered_set<uint32_t> processed_tiles_;

  // Hierarchy limits.
  std::vector<sif::HierarchyLimits> hierarchy_limits_;

  // A* heuristic
  AStarHeuristic astarheuristic_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::MMEdgeLabel> edgelabels_;

  // Adjacency list - approximate double bucket sort
  baldr::DoubleBucketQueue<sif::MMEdgeLabel> adjacencylist_;

  // Edge status. Mark edges that are in adjacency list or settled.
  EdgeStatus edgestatus_;

  // Destinations, id and cost
  std::map<uint64_t, sif::Cost> destinations_;

  /**
   * Initializes the hierarchy limits, A* heuristic, and adjacency list.
   * @param  destll  Lat,lng of the destination.
   * @param  costing Dynamic costing method.
   */
  void Init(const midgard::PointLL& destll, const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Add edges at the origin to the adjacency list.
   * @param  graphreader  Graph tile reader.
   * @param  origin       Location information of the origin.
   * @param  dest         Location information of the destination.
   * @param  costing      Dynamic costing.
   */
  void SetOrigin(baldr::GraphReader& graphreader,
                 valhalla::Location& origin,
                 const valhalla::Location& dest,
                 const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Set the destination edge(s).
   * @param   graphreader  Graph tile reader.
   * @param   dest         Location information of the destination.
   * @param   costing      Dynamic costing.
   * @return  Returns the relative density near the destination (0-15)
   */
  uint32_t SetDestination(baldr::GraphReader& graphreader,
                          const valhalla::Location& dest,
                          const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Expand from the node along the forward search path. Immediately expands
   * from the end node of any transition edge (so no transition edges are added
   * to the adjacency list or EdgeLabel list). Does not expand transition edges if
   * from_transition is false. This method is only used in CanReachDestination.
   * @param  graphreader  Graph tile reader.
   * @param  node         Graph Id of the node being expanded.
   * @param  pred         Predecessor edge label (for costing).
   * @param  pred_idx     Predecessor index into the EdgeLabel list.
   * @param  costing      Current costing method.
   * @param  edgestatus   Local edge status information.
   * @param  edgelabels   Local edge label list.
   * @param  adjlist      Local adjacency list/priority queue.
   * @param  from_transition True if this method is called from a transition edge.
   * @return Returns true if a transit stop has been reached. False, otherwise.
   */
  bool ExpandFromNode(baldr::GraphReader& graphreader,
                      const baldr::GraphId& node,
                      const sif::EdgeLabel& pred,
                      const uint32_t pred_idx,
                      const std::shared_ptr<sif::DynamicCost>& costing,
                      EdgeStatus& edgestatus,
                      std::vector<sif::EdgeLabel>& edgelabels,
                      baldr::DoubleBucketQueue<sif::EdgeLabel>& adjlist,
                      const bool from_transition);

  /**
   * Expand from the node using multimodal algorithm.
   * @param graphreader  Graph reader.
   * @param node Graph Id of the node to expand.
   * @param pred Edge label of the predecessor edge leading to the node.
   * @param pred_idx Index in the edge label list of the predecessor edge.
   * @param from_transition Boolean indicating if this expansion is from a transition edge.
   * @param pc Pedestrian costing.
   * @param tc Transit costing.
   * @param mode_costing Array of all costing models.
   * @param  time_info    Information time offset as the route progresses
   * @return Returns false if the node could not be expanded from
   */
  bool ExpandForward(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     const sif::MMEdgeLabel& pred,
                     const uint32_t pred_idx,
                     const bool from_transition,
                     const std::shared_ptr<sif::DynamicCost>& pc,
                     const std::shared_ptr<sif::DynamicCost>& tc,
                     const sif::mode_costing_t& mode_costing,
                     const baldr::TimeInfo& time_info);

  /**
   * Check if destination can be reached if walking is the last mode. Checks
   * if there are any transit stops within maximum walking distance from
   * the destination. This is used to reject impossible routes given the
   * modes allowed.
   * TODO - once auto/bicycle are allowed modes we need to check if parking
   * or bikeshare locations are within walking distance.
   */
  bool CanReachDestination(const valhalla::Location& destination,
                           baldr::GraphReader& graphreader,
                           const sif::TravelMode dest_mode,
                           const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Form the path from the adjacency list. Recovers the path from the
   * destination backwards towards the origin (using predecessor information)
   * @param   dest  Index in the edge labels of the destination edge.
   * @return  Returns the path info, a list of GraphIds representing the
   *          directed edges along the path - ordered from origin to
   *          destination - along with travel modes and elapsed time.
   */
  std::vector<PathInfo> FormPath(const uint32_t dest);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_MULTIMODAL_H_
