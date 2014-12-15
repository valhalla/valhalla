#ifndef VALHALLA_THOR_EDGECOST_H_
#define VALHALLA_THOR_EDGECOST_H_

#include "baldr/directededge.h"

namespace valhalla{
namespace thor{

/**
 * Base class for dynamic edge costing.
 */
class EdgeCost {
 public:
  EdgeCost();

  virtual ~EdgeCost() {
  }

  // TODO - use int or float?
  virtual float Get(const baldr::DirectedEdge* edge);

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor();
};

}
}

#endif  // VALHALLA_THOR_EDGECOST_H_
