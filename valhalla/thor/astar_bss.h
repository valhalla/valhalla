#ifndef VALHALLA_THOR_ASTAR_BSS_H
#define VALHALLA_THOR_ASTAR_BSS_H

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
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
 * Bla, bla, bla
 *
 */
class AStarBSSAlgorithm : public PathAlgorithm {
public:
  /**
   * Constructor.
   * @param config A config object of key, value pairs
   */
  explicit AStarBSSAlgorithm(const boost::property_tree::ptree& config = {});

  /**
   * Destructor
   */
  virtual ~AStarBSSAlgorithm();

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
  virtual std::vector<std::vector<PathInfo>>
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
    return "a*_bike_share_station";
  }

  /**
   * Clear the temporary information generated during path construction.
   */
  virtual void Clear() override;

  /**
   * Set a maximum label count. The path algorithm terminates if this
   * is exceeded.
   * @param  max_count  Maximum number of labels to allow.
   */
  void set_max_label_count(const uint32_t max_count) {
    max_label_count_ = max_count;
  }

protected:
  uint32_t max_label_count_; // Max label count to allow
  sif::TravelMode mode_;     // Current travel mode
  uint8_t travel_type_;      // Current travel type

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

  // Destinations, id and cost
  std::map<uint64_t, sif::Cost> destinations_;

  /**
   * Initializes the hierarchy limits, A* heuristic, and adjacency list.
   * @param  origll  Lat,lng of the origin.
   * @param  destll  Lat,lng of the destination.
   */
  virtual void Init(const midgard::PointLL& origll, const midgard::PointLL& destll);

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
   * @param   dest        Location information of the destination.
   */
  void ExpandForward(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     const sif::EdgeLabel& pred,
                     const uint32_t pred_idx,
                     const bool from_transition,
                     const bool from_bss,
                     const sif::TravelMode mode,
                     const valhalla::Location& dest,
                     std::pair<int32_t, float>& best_path);

  /**
   * Add edges at the origin to the adjacency list.
   * @param  graphreader  Graph tile reader.
   * @param  origin       Location information of the origin.
   * @param  dest         Location information of the destination.
   */
  void SetOrigin(baldr::GraphReader& graphreader,
                 valhalla::Location& origin,
                 const valhalla::Location& dest);

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
  std::vector<PathInfo> FormPath(baldr::GraphReader& graphreader, const uint32_t dest);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_ASTAR_BSS_H
