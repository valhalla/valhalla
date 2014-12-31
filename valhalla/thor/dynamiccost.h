#ifndef VALHALLA_THOR_DYNAMICCOST_H_
#define VALHALLA_THOR_DYNAMICCOST_H_

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>

namespace valhalla {
namespace thor {

/**
 * Base class for dynamic edge costing. This class is an abstract class
 * defining the interface for costing methods. Derived classes must implement
 * methods to consider if access is allowed, compute a cost to traverse an
 * edge, and a couple other methods required to setup A* heuristics and
 * sorting methods.
 */
class DynamicCost {
 public:
  DynamicCost();

  virtual ~DynamicCost() = 0;

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters.
   * @param  edge  Pointer to a directed edge.
   * @param uturn Is this a Uturn?
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge, const bool uturn) = 0;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present. (TODO - others?)
   * @param  edge  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) = 0;

  /**
   * Get the cost given a directed edge.
   * @param edge  Pointer to a directed edge.
   * @return  Returns the cost to traverse the edge.
   */
  virtual float Get(const baldr::DirectedEdge* edge) = 0;

  /**
   * Returns the time (in seconds) to traverse the edge.
   * @param edge  Pointer to a directed edge.
   * @return  Returns the time in seconds to traverse the edge.
   */
  virtual float Seconds(const baldr::DirectedEdge* edge) = 0;


  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() = 0;

  /**
   * Get the general unit size that can be considered as equal for sorting
   * purposes. The A* method uses an approximate bucket sort, and this value
   * is used to size the buckets used for sorting. For example, for time
   * based costs one might compute costs in seconds and consider any time
   * within 1.5 seconds of each other as being equal (for sorting purposes).
   * @return  Returns the unit size for sorting.
   */
  virtual float UnitSize() const = 0;
};

}
}

#endif  // VALHALLA_THOR_DYNAMICCOST_H_
