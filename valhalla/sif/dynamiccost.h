#ifndef VALHALLA_THOR_DYNAMICCOST_H_
#define VALHALLA_THOR_DYNAMICCOST_H_

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <memory>

#include <valhalla/sif/hierarchylimits.h>
#include <valhalla/sif/edgelabel.h>

namespace valhalla {
namespace thor {

/**
 * A callable element which returns true if an edge should be
 * filtered out of the correlated set and false if the edge is usable
 */
using EdgeFilter = std::function<bool (const baldr::DirectedEdge*)>;

// Default unit size (seconds) for cost sorting.
constexpr uint32_t kDefaultUnitSize = 1;

// Simple structure for returning costs
struct Cost {
  float cost;
  float seconds;

  // Default constructor
  Cost()
      : cost(0.0f),
        seconds(0.0f) {
  }

  // Constructor with args
  Cost(const float c, const float s)
       : cost(c),
         seconds(s) {
  }

  // Add 2 costs
  Cost operator + (const Cost& other) const {
    return Cost(cost + other.cost, seconds + other.seconds);
  }
};

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
   * @param  pt  Property tree with (optional) costing configuration.
   */
  DynamicCost(const boost::property_tree::ptree& pt);

  virtual ~DynamicCost();

  /**
   * Does the costing allow hierarchy transitions?
   * @return  Returns true if the costing model allows hierarchy transitions).
   */
  virtual bool AllowTransitions() const;

  /**
   * Does the costing method allow multiple passes (with relaxed
   * hierarchy limits).
   * @return  Returns true if the costing model allows multiple passes.
   */
  virtual bool AllowMultiPass() const;

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters.
   * @param  edge  Pointer to a directed edge.
   * @param  pred  Predecessor edge information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred) const = 0;

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
   * @param   edge     Pointer to a directed edge.
   * @param   density  Relative road density.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const uint32_t density) const = 0;

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param   edge  Directed edge (the to edge)
   * @param   node  Node (intersection) where transition occurs.
   * @param   pred  Predecessor edge information.
   * @param   to_idx Index of the "to" directed edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred,
                              const uint32_t to_idx) const;

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
   * Set the distance from the destination where "not_thru" edges are allowed.
   * @param  d  Distance in meters.
   */
  void set_not_thru_distance(const float d);

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each edges attribution
   */
  virtual const EdgeFilter GetFilter() const = 0;

  /**
   * Gets the hierarchy limits.
   * @return  Returns the hierarchy limits.
   */
  std::vector<HierarchyLimits>& GetHierarchyLimits();

  /**
   * Relax hierarchy limits.
   */
  void RelaxHierarchyLimits(const float factor);

  /**
   * Do not transition up to highway level - remain on arterial. Used as last
   * resort.
   */
  void DisableHighwayTransitions();

  /**
   * Reset hierarchy limits.
   */
  void ResetHierarchyLimits();

 protected:
  // Hierarchy limits.
  std::vector<HierarchyLimits> hierarchy_limits_;

  // Distance from the destination within which "not_thru" roads are
  // considered. All costing methods exclude such roads except when close
  // to the destination
  float not_thru_distance_;
};

typedef std::shared_ptr<DynamicCost> cost_ptr_t;

}
}

#endif  // VALHALLA_THOR_DYNAMICCOST_H_
