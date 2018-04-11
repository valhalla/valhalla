#ifndef VALHALLA_SIF_DYNAMICCOST_H_
#define VALHALLA_SIF_DYNAMICCOST_H_

#include <cstdint>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/transitdeparture.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/double_bucket_queue.h> // For kInvalidLabel

#include <memory>
#include <unordered_set>

#include <valhalla/sif/hierarchylimits.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/costconstants.h>

namespace valhalla {
namespace sif {

/**
 * A callable element which returns a value between 0 and 1 indicating how
 * desirable the edge is for use as a location. A value of 0 indicates the
 * edge is not usable (no access for the travel mode used by this costing)
 * while 1 indicates the edge is highly preferred. Values in between can be
 * used to rank edges such that desirable edges that might be slightly
 * farther from the location than a less desirable edge can be chosen.
 */
using EdgeFilter = std::function<float (const baldr::DirectedEdge*)>;

/**
 * A callable element which returns true if a node should be
 * filtered out/ not used and false if the node is usable
 */
using NodeFilter = std::function<bool (const baldr::NodeInfo*)>;

// Default unit size (seconds) for cost sorting.
constexpr uint32_t kDefaultUnitSize = 1;

/**
 * Base class for dynamic edge costing. This class defines the interface for
 * costing methods and includes a few base methods that define default behavior
 * for cases where a derived class does not need to override the method.
 * Derived classes must implement methods to consider if access is allowed,
 * compute a cost to traverse an edge, and a couple other methods required to
 * setup A* heuristics and sorting methods. Derived classes can also define
 * edge transition (intersection/turn) costing. EdgeCost and TransitionCost
 * methods return a Cost structure includes a cost for Dijkstra/A* as well as
 * the elapsed time (seconds) so that time along the path can be estimated
 * (for transit schedule lookups. timed restrictions, and other time dependent
 * logic).
 */
class DynamicCost {
 public:
  /**
   * Constructor.
   * @param  pt   Property tree with (optional) costing configuration.
   * @param  mode Travel mode
   */
  DynamicCost(const boost::property_tree::ptree& pt,
              const TravelMode mode);

  virtual ~DynamicCost();

  /**
   * Does the costing method allow multiple passes (with relaxed
   * hierarchy limits).
   * @return  Returns true if the costing model allows multiple passes.
   */
  virtual bool AllowMultiPass() const;

  /**
   * Get the pass number.
   * @return  Returns the pass through the algorithm.
   */
  uint32_t pass() const {
    return pass_;
  }

  /**
   * Set the pass number.
   * @param  pass  Pass number (incremental).
   */
  void set_pass(const uint32_t pass) {
    pass_ = pass;
  }

  /**
   * Returns the maximum transfer distance between stops that you are willing
   * to travel for this mode.  It is the max distance you are willing to
   * travel between transfers.
   * @return  max transfer distance multimodal
   */
  virtual uint32_t GetMaxTransferDistanceMM();

  /**
   * This method overrides the factor for this mode.  The lower the value
   * the more the mode is favored.
   * @return  mode factor
   */
  virtual float GetModeFactor();

  /**
   * This method overrides the max_distance with the max_distance_mm per segment
   * distance. An example is a pure walking route may have a max distance of
   * 10000 meters (10km) but for a multi-modal route a lower limit of 5000
   * meters per segment (e.g. from origin to a transit stop or from the last
   * transit stop to the destination).
   */
  virtual void UseMaxMultiModalDistance();

  /**
   * Get the access mode used by this costing method.
   * @return  Returns access mode.
   */
  virtual uint32_t access_mode() const = 0;

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters.
   * @param  edge     Pointer to a directed edge.
   * @param  pred     Predecessor edge information.
   * @param  tile     current tile
   * @param  edgeid   edgeid that we care about
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid) const = 0;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges are
   * provided.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           current tile
   * @param  edgeid         edgeid that we care about
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                 const EdgeLabel& pred,
                 const baldr::DirectedEdge* opp_edge,
                 const baldr::GraphTile*& tile,
                 const baldr::GraphId& edgeid) const = 0;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards are present.
   * @param   node  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const = 0;

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge  Pointer to a directed edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge) const = 0;

  /**
   * Get the cost to traverse the specified directed edge using a transit
   * departure (schedule based edge traversal). Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge      Pointer to a directed edge.
   * @param   departure Transit departure record.
   * @param   curr_time Current local time (seconds from midnight).
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::TransitDeparture* departure,
                        const uint32_t curr_time) const;

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param   edge  Directed edge (the to edge)
   * @param   node  Node (intersection) where transition occurs.
   * @param   pred  Predecessor edge information.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred) const;

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  idx   Directed edge local index
   * @param  node  Node (intersection) where transition occurs.
   * @param  opp_edge  Pointer to the opposing directed edge - this is the
   *                   "from" or predecessor edge in the transition.
   * @param  opp_pred_edge  Pointer to the opposing directed edge to the
   *                        predecessor. This is the "to" edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                              const baldr::NodeInfo* node,
                              const baldr::DirectedEdge* opp_edge,
                              const baldr::DirectedEdge* opp_pred_edge) const;

  /**
   * Test if an edge should be restricted due to a complex restriction.
   * @param  edge  Directed edge.
   * @param  pred        Predecessor information.
   * @param  edge_labels List of edge labels.
   * @param  tile        Graph tile (to read restriction if needed).
   * @param  edgeid      Edge Id for the directed edge.
   * @param  forward     Forward search or reverse search.
   * @return Returns true it there is a complex restriction onto this edge
   *         that matches the mode and the predecessor list for the current
   *         path matches a complex restriction.
   */
  template <typename edge_labels_container_t>
  bool Restricted(const baldr::DirectedEdge* edge,
                  const EdgeLabel& pred,
                  const edge_labels_container_t& edge_labels,
                  const baldr::GraphTile*& tile,
                  const baldr::GraphId& edgeid,
                  const bool forward) const {
    // Lambda to get the next predecessor EdgeLabel (that is not a transition)
    auto next_predecessor = [&edge_labels](const EdgeLabel* label) {
      // Get the next predecessor - make sure it is valid. Continue to get
      // the next predecessor if the edge is a transition edge.
      const EdgeLabel* next_pred = (label->predecessor() == baldr::kInvalidLabel) ?
                      label : &edge_labels[label->predecessor()];
      while (next_pred->use() == baldr::Use::kTransitionUp &&
             next_pred->predecessor() != baldr::kInvalidLabel) {
        next_pred = &edge_labels[next_pred->predecessor()];
      }
      return next_pred;
    };

    // If forward, check if the edge marks the end of a restriction, else check
    // if the edge marks the start of a complex restriction.
    if (( forward && (edge->end_restriction()   & access_mode())) ||
        (!forward && (edge->start_restriction() & access_mode()))) {
      // Get complex restrictions. Return false if no restrictions are found
      auto restrictions = tile->GetRestrictions(forward, edgeid, access_mode());
      if (restrictions.size() == 0) {
        return false;
      }

      // Get the first predecessor edge (that is not a transition)
      // TODO - do not need this if no transition edges are added to EdgeLabels
      const EdgeLabel* first_pred = &pred;
      if (first_pred->use() == baldr::Use::kTransitionUp) {
        first_pred = next_predecessor(first_pred);
      }

      // Iterate through the restrictions
      for (const auto& cr : restrictions) {
        // Walk the via list, move to the next restriction if the via edge
        // Ids do not match the path for this restriction.
        bool match = true;
        const EdgeLabel* next_pred = first_pred;
        for (const auto& via_id : cr.GetVias()) {
          if (via_id != next_pred->edgeid()) {
            match = false;
            break;
          }
          next_pred = next_predecessor(next_pred);
        }

        // Check against the start/end of the complex restriction
        if (match && (( forward && next_pred->edgeid() == cr.from_id()) ||
                      (!forward && next_pred->edgeid() == cr.to_id()))) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * Returns the transfer cost between 2 transit stops.
   * @return  Returns the transfer cost and time (seconds).
   */
  virtual Cost TransferCost() const;

  /**
   * Returns the default transfer cost between 2 transit lines.
   * @return  Returns the transfer cost and time (seconds).
   */
  virtual Cost DefaultTransferCost() const;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   * @return  Returns the cost factor used in the A* heuristic.
   */
  virtual float AStarCostFactor() const = 0;

  /**
   * Get the general unit size that can be considered as equal for sorting
   * purposes. The A* method uses an approximate bucket sort, and this value
   * is used to size the buckets used for sorting. For example, for time
   * based costs one might compute costs in seconds and consider any time
   * within 2 seconds of each other as being equal (for sorting purposes).
   * @return  Returns the unit size for sorting (must be an integer value).
   * Defaults to 1 (second).
   * @return  Returns unit size.
   */
  virtual uint32_t UnitSize() const;

  /**
   * Sets the flag indicating whether destination only edges are allowed.
   * Bidirectional path algorithms can (usually) disable access.
   */
  virtual void set_allow_destination_only(const bool allow);

  /**
   * Set to allow use of transit connections.
   * @param  allow  Flag indicating whether transit connections are allowed.
   */
  virtual void SetAllowTransitConnections(const bool allow);

  /**
   * Set the current travel mode.
   * @param  mode  Travel mode
   */
  void set_travel_mode(const TravelMode mode);

  /**
   * Get the current travel mode.
   * @return  Returns the current travel mode.
   */
  TravelMode travel_mode() const;

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const;

  /**
   * Get the wheelchair required flag.
   * @return  Returns true if wheelchair is required.
   */
  virtual bool wheelchair() const;

  /**
   * Get the bicycle required flag.
   * @return  Returns true if bicycle is required.
   */
  virtual bool bicycle() const;

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method.
   */
  virtual const EdgeFilter GetEdgeFilter() const = 0;

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each node's attribution
   */
  virtual const NodeFilter GetNodeFilter() const = 0;

  /**
   * Gets the hierarchy limits.
   * @return  Returns the hierarchy limits.
   */
  std::vector<HierarchyLimits>& GetHierarchyLimits();

  /**
   * Relax hierarchy limits.
   */
  void RelaxHierarchyLimits(const float factor, const float expansion_within_factor);

  /**
   * Checks if we should exclude or not.
   */
  virtual void AddToExcludeList(const baldr::GraphTile*& tile);

  /**
   * Checks if we should exclude or not.
   * @return  Returns true if we should exclude, false if not.
   */
  virtual bool IsExcluded(const baldr::GraphTile*& tile,
                          const baldr::DirectedEdge* edge);

  /**
   * Checks if we should exclude or not.
   * @return  Returns true if we should exclude, false if not.
   */
  virtual bool IsExcluded(const baldr::GraphTile*& tile,
                          const baldr::NodeInfo* node);

  /**
   * Adds a list of edges (GraphIds) to the user specified avoid list.
   * This can be used by test programs - alternatively a list of avoid
   * edges will be passed in the property tree for the costing options
   * of a specified type.
   * @param  avoid_edges  Set of edge Ids to avoid.
   */
  void AddUserAvoidEdges(const std::vector<baldr::GraphId>& avoid_edges);

  /**
   * Check if the edge is in the user-specified avoid list.
   * @param  edgeid  Directed edge Id.
   * @return Returns true if the edge Id is in the user avoid edges set,
   *         false otherwise.
   */
  bool IsUserAvoidEdge(const baldr::GraphId& edgeid) const {
    return (user_avoid_edges_.size() != 0 &&
            user_avoid_edges_.find(edgeid) != user_avoid_edges_.end());
  }

 protected:
  // Algorithm pass
  uint32_t pass_;

  // Flag indicating whether transit connections are allowed.
  bool allow_transit_connections_;

  // Allow entrance onto destination only edges. Bidirectional A* can (usually)
  // disable access onto destination only edges for driving routes. Pedestrian
  // and bicycle generally allow access (with small penalties).
  bool allow_destination_only_;

  // Travel mode
  TravelMode travel_mode_;

  // Hierarchy limits.
  std::vector<HierarchyLimits> hierarchy_limits_;

  // User specified edges to avoid
  std::unordered_set<baldr::GraphId> user_avoid_edges_;
};

typedef std::shared_ptr<DynamicCost> cost_ptr_t;

}
}

#endif  // VALHALLA_SIF_DYNAMICCOST_H_
