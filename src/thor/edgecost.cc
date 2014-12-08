#include "thor/edgecost.h"

using namespace valhalla::baldr;

namespace valhalla{
namespace thor{

	EdgeCost::EdgeCost() { }

  // TODO - use int or float?
	float EdgeCost::Get(const DirectedEdge* edge) {
    return edge->length();
  }

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  float EdgeCost::AStarCostFactor() {
    return 1.0f;
  }

}
}
