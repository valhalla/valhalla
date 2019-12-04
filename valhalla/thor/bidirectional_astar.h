#ifndef VALHALLA_THOR_BIDIRECTIONAL_ASTAR_H_
#define VALHALLA_THOR_BIDIRECTIONAL_ASTAR_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/hierarchylimits.h>
#include <valhalla/thor/astarheuristic.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathalgorithm.h>

namespace valhalla {
namespace thor {

/**
 * Candidate connections - a directed edge and its opposing directed edge
 * are both temporarily labeled. Store the edge Ids and its cost.
 */
struct CandidateConnection {
  baldr::GraphId edgeid;
  baldr::GraphId opp_edgeid;
  float cost;
  bool operator<(const CandidateConnection& o) const {
    return cost < o.cost;
  }
  bool operator<(float c) const {
    return cost < c;
  }
};

/**
 * Bidirectional A* algorithm. Method for finding least-cost path.
 */
class BidirectionalAStar : public PathAlgorithm {
public:
  /**
   * Constructor.
   */
  BidirectionalAStar();

  /**
   * Destructor
   */
  virtual ~BidirectionalAStar();

  /**
   * Form path between and origin and destination location using
   * the supplied mode and costing method.
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
              const std::shared_ptr<sif::DynamicCost>* mode_costing,
              const sif::TravelMode mode,
              const Options& options = Options::default_instance());

  /**
   * Clear the temporary information generated during path construction.
   */
  void Clear();

protected:
  // Access mode used by the costing method
  uint32_t access_mode_;

  // Current travel mode
  sif::TravelMode mode_;

  // Current travel type
  uint8_t travel_type_;

  // Current costing mode
  std::shared_ptr<sif::DynamicCost> costing_;

  // Hierarchy limits
  std::vector<sif::HierarchyLimits> hierarchy_limits_forward_;
  std::vector<sif::HierarchyLimits> hierarchy_limits_reverse_;

  // A* heuristic
  float cost_diff_;
  AStarHeuristic astarheuristic_forward_;
  AStarHeuristic astarheuristic_reverse_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::BDEdgeLabel> edgelabels_forward_;
  std::vector<sif::BDEdgeLabel> edgelabels_reverse_;

  // Adjacency list - approximate double bucket sort
  std::shared_ptr<baldr::DoubleBucketQueue> adjacencylist_forward_;
  std::shared_ptr<baldr::DoubleBucketQueue> adjacencylist_reverse_;

  // Edge status. Mark edges that are in adjacency list or settled.
  EdgeStatus edgestatus_forward_;
  EdgeStatus edgestatus_reverse_;

  // Best candidate connection and threshold to extend search.
  float threshold_;
  CandidateConnection best_connection_;

  /**
   * Initialize the A* heuristic and adjacency lists for both the forward
   * and reverse search.
   * @param  origll  Lat,lng of the origin.
   * @param  destll  Lat,lng of the destination.
   */
  void Init(const midgard::PointLL& origll, const midgard::PointLL& destll);

  /**
   * Expand from the node along the forward search path.
   */
  bool ExpandForward(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     sif::BDEdgeLabel& pred,
                     const uint32_t pred_idx,
                     const bool from_transition);
  // Private helper function for `ExpandForward`
  bool ExpandForwardInner(baldr::GraphReader& graphreader,
                          const sif::BDEdgeLabel& pred,
                          const baldr::NodeInfo* nodeinfo,
                          const uint32_t pred_idx,
                          const EdgeMetadata& meta,
                          uint32_t& shortcuts,
                          const baldr::GraphTile* tile);

  /**
   * Expand from the node along the reverse search path.
   */
  bool ExpandReverse(baldr::GraphReader& graphreader,
                     const baldr::GraphId& node,
                     sif::BDEdgeLabel& pred,
                     const uint32_t pred_idx,
                     const baldr::DirectedEdge* opp_pred_edge,
                     const bool from_transition);

  // Private helper function for `ExpandReverse`
  bool ExpandReverseInner(baldr::GraphReader& graphreader,
                          const sif::BDEdgeLabel& pred,
                          const baldr::DirectedEdge* opp_pred_edge,
                          const baldr::NodeInfo* nodeinfo,
                          const uint32_t pred_idx,
                          const EdgeMetadata& meta,
                          uint32_t& shortcuts,
                          const baldr::GraphTile* tile);
  /**
   * Add edges at the origin to the forward adjacency list.
   * @param  graphreader  Graph tile reader.
   * @param  origin       Location information of the destination
   */
  void SetOrigin(baldr::GraphReader& graphreader, valhalla::Location& origin);

  /**
   * Add destination edges to the reverse path adjacency list.
   * @param   graphreader  Graph tile reader.
   * @param   dest         Location information of the destination
   */
  void SetDestination(baldr::GraphReader& graphreader, const valhalla::Location& dest);

  /**
   * The edge on the forward search connects to a reached edge on the reverse
   * search tree. Check if this is the best connection so far and set the
   * search threshold.
   * @param  pred  Edge label of the predecessor.
   * @return Returns true if a connection was set, false if not (if on a complex restriction).
   */
  bool SetForwardConnection(baldr::GraphReader& graphreader, const sif::BDEdgeLabel& pred);

  /**
   * The edge on the reverse search connects to a reached edge on the forward
   * search tree. Check if this is the best connection so far and set the
   * search threshold.
   * @param  pred  Edge label of the predecessor.
   * @return Returns true if a connection was set, false if not (if on a complex restriction).
   */
  bool SetReverseConnection(baldr::GraphReader& graphreader, const sif::BDEdgeLabel& pred);

  /**
   * Form the path from the adjacency lists. Recovers the path from the
   * where the paths meet back towards the origin then reverses this path.
   * The path from where the paths meet to the destination is then appended
   * using the opposing edges (so the path is traversed forward).
   * @param   graphreader  Graph tile reader (for getting opposing edges).
   * @param   options      Controls whether or not we get alternatives
   * @return  Returns the path infos, a list of GraphIds representing the
   *          directed edges along the path - ordered from origin to
   *          destination - along with travel modes and elapsed time.
   */
  std::vector<std::vector<PathInfo>> FormPath(baldr::GraphReader& graphreader,
                                              const Options& options);
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_BIDIRECTIONAL_ASTAR_H_
