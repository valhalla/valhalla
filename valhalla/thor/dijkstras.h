#ifndef VALHALLA_THOR_Dijkstras_H_
#define VALHALLA_THOR_Dijkstras_H_

#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/time_info.h>
#include <valhalla/proto/common.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathalgorithm.h>

namespace valhalla {
namespace thor {

enum class ExpansionRecommendation {
  continue_expansion,
  stop_expansion,
  prune_expansion,
};

/**
 * Algorithms to do shortest first graph traversal
 */
class Dijkstras {
public:
  /**
   * Constructor.
   * @param config A config object of key, value pairs
   */
  explicit Dijkstras(const boost::property_tree::ptree& config = {});

  Dijkstras(const Dijkstras&) = delete;
  Dijkstras& operator=(const Dijkstras&) = delete;

  virtual ~Dijkstras() {
  }

  /**
   * Clear the temporary memory (adjacency list, edgestatus, edgelabels)
   */
  virtual void Clear();

  /**
   * Compute the best first graph traversal from a list locations
   * @param expansion_type  What type of expansion should be run
   * @param  locations      List of locations from which to expand.
   * @param  reader         provides access to underlying graph primitives
   * @param  costings       List of costing objects
   * @param  mode           Travel mode
   */
  void Expand(ExpansionType expansion_type,
              valhalla::Api& api,
              baldr::GraphReader& reader,
              const sif::mode_costing_t& costings,
              const sif::TravelMode mode);

  /**
   * Sets the functor which will track the Dijkstra expansion.
   *
   * @param  expansion_callback  the functor to call back when the Dijkstra makes progress
   *                             on a given edge
   */
  using expansion_callback_t = std::function<
      void(baldr::GraphReader&, baldr::GraphId, const char*, const char*, float, uint32_t, float)>;
  void set_track_expansion(const expansion_callback_t& expansion_callback) {
    expansion_callback_ = expansion_callback;
  }

protected:
  /**
   * Compute the best first graph traversal from a list of origin locations
   * @param  origin_locs  List of origin locations.
   * @param  graphreader  Graphreader
   * @param  mode_costing List of costing objects
   * @param  mode         Travel mode
   */
  // Note: ExpansionType::multimodal not yet implemented
  template <const ExpansionType expansion_direction>
  void Compute(google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
               baldr::GraphReader& graphreader,
               const sif::mode_costing_t& mode_costing,
               const sif::TravelMode mode);

  /**
   * Compute the best first graph traversal from a list of origin locations using multimodal
   * @param  origin_locs  List of origin locations.
   * @param  graphreader  Graphreader
   * @param  mode_costing List of costing objects
   * @param  mode         Travel mode
   */
  virtual void
  ComputeMultiModal(google::protobuf::RepeatedPtrField<valhalla::Location>& origin_locations,
                    baldr::GraphReader& graphreader,
                    const sif::mode_costing_t& mode_costing,
                    const sif::TravelMode mode,
                    const valhalla::Options& options);

  // A child-class must implement this to learn about what nodes were expanded
  virtual void ExpandingNode(baldr::GraphReader&,
                             graph_tile_ptr,
                             const baldr::NodeInfo*,
                             const sif::EdgeLabel&,
                             const sif::EdgeLabel*) = 0;

  // A child-class must implement this to decide when to stop the expansion
  virtual ExpansionRecommendation ShouldExpand(baldr::GraphReader& graphreader,
                                               const sif::EdgeLabel& pred,
                                               const ExpansionType route_type) = 0;

  // A child-class must implement this to tell the algorithm how much expansion to expect to do
  virtual void GetExpansionHints(uint32_t& bucket_count, uint32_t& edge_label_reservation) const = 0;

  sif::TravelMode mode_; // Current travel mode
  uint32_t access_mode_; // Access mode used by the costing method

  // For multimodal
  bool date_set_;
  bool date_before_tile_;
  uint32_t date_;
  uint32_t dow_;
  uint32_t day_;
  uint32_t max_transfer_distance_;
  uint32_t max_walking_dist_;
  std::string origin_date_time_;
  std::unordered_map<std::string, uint32_t> operators_;
  std::unordered_set<uint32_t> processed_tiles_;

  // Current costing mode
  std::shared_ptr<sif::DynamicCost> costing_;

  // Vector of edge labels (requires access by index).
  std::vector<sif::BDEdgeLabel> bdedgelabels_;
  std::vector<sif::MMEdgeLabel> mmedgelabels_;
  uint32_t max_reserved_labels_count_;

  // if `true` clean reserved memory for edge labels
  bool clear_reserved_memory_;

  // Adjacency list - approximate double bucket sort
  baldr::DoubleBucketQueue<sif::BDEdgeLabel> adjacencylist_;
  baldr::DoubleBucketQueue<sif::MMEdgeLabel> mmadjacencylist_;

  // Edge status. Mark edges that are in adjacency list or settled.
  EdgeStatus edgestatus_;

  // when doing timezone differencing a timezone cache speeds up the computation
  baldr::DateTime::tz_sys_info_cache_t tz_cache_;

  // for tracking the expansion of the Dijkstra
  expansion_callback_t expansion_callback_;

  // when expanding should we treat each location as its own individual path to track concurrently but
  // separately from the other paths
  bool multipath_;

  // TODO: add an interrupt here so that the caller can abort the main loop externally

  /**
   * Initialization prior to computing the graph expansion
   *
   * Creates adjacency list, edgestatus support, and reserves edgelabels.
   * @param bucketsize  Adjacency list bucket size.
   */
  template <typename label_container_t>
  void Initialize(label_container_t& labels,
                  baldr::DoubleBucketQueue<typename label_container_t::value_type>& queue,
                  const uint32_t bucketsize);

  /**
   * Sets the start time for forward expansion or end time for reverse expansion based on the
   * locations date time string and the edge candidates timezone
   *
   * @param location           which location to use for the date time information
   * @param reader             the reader for looking up timezone information
   * @returns                  time info for each location
   */
  std::vector<baldr::TimeInfo>
  SetTime(google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
          baldr::GraphReader& reader);

  /**
   * Expand from the node along the search path for non-multimodal expansion
   * Handles both forward and reverse traversals
   * @param graphreader  Graph reader.
   * @param node Graph Id of the node to expand.
   * @param pred Edge label of the predecessor edge leading to the node.
   * @param pred_idx Index in the edge label list of the predecessor edge.
   * @param from_transition Boolean indicating if this expansion is from a transition edge.
   * @param time_info Tracks time offset as the expansion progresses
   */
  template <const ExpansionType expansion_direction>
  void ExpandInner(baldr::GraphReader& graphreader,
                   const baldr::GraphId& node,
                   const typename decltype(Dijkstras::bdedgelabels_)::value_type& pred,
                   const uint32_t pred_idx,
                   const baldr::DirectedEdge* opp_pred_edge,
                   const bool from_transition,
                   const baldr::TimeInfo& time_info);

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
   * @param time_info Tracks time offset as the expansion progresses
   */
  void ExpandForwardMultiModal(baldr::GraphReader& graphreader,
                               const baldr::GraphId& node,
                               const sif::MMEdgeLabel& pred,
                               const uint32_t pred_idx,
                               const bool from_transition,
                               const std::shared_ptr<sif::DynamicCost>& pc,
                               const std::shared_ptr<sif::DynamicCost>& tc,
                               const sif::mode_costing_t& mode_costing,
                               const baldr::TimeInfo& time_info);

  /**
   * Add edge(s) at each origin location to the adjacency list.
   * @param  graphreader       Graph tile reader.
   * @param  locations  Location information for origins.
   * @param  costing           Dynamic costing.
   */
  void SetOriginLocations(baldr::GraphReader& graphreader,
                          google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                          const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Add edge(s) at each origin location to the adjacency list.
   * @param  graphreader       Graph tile reader.
   * @param  origin_locations  Location information for origins.
   * @param  costing           Dynamic costing.
   */
  void SetOriginLocationsMultiModal(
      baldr::GraphReader& graphreader,
      google::protobuf::RepeatedPtrField<valhalla::Location>& origin_locations,
      const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Add edge(s) at each destination location to the adjacency list.
   * @param  graphreader       Graph tile reader.
   * @param  locations    Location information for destinations.
   * @param  costing           Dynamic costing.
   */
  void SetDestinationLocations(baldr::GraphReader& graphreader,
                               google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                               const std::shared_ptr<sif::DynamicCost>& costing);

  /**
   * Convenience method to get the timezone index at a node.
   * @param graphreader Graph reader.
   * @param node GraphId of the node to get the timezone index.
   * @return Returns the timezone index. A value of 0 indicates an invalid timezone.
   */
  int GetTimezone(baldr::GraphReader& graphreader, const baldr::GraphId& node) {
    graph_tile_ptr tile = graphreader.GetGraphTile(node);
    return (tile == nullptr) ? 0 : tile->node(node)->timezone();
  }
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_Dijkstras_H_
