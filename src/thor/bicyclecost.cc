#include "thor/bicyclecost.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// TODO - should bicycle routes be distance based or time based? Could alter
// time based on hilliness?

// Constructor
BicycleCost::BicycleCost()
    : DynamicCost() {
}

// Destructor
BicycleCost::~BicycleCost() {
}

// Check if access is allowed on the specified edge.
bool BicycleCost::Allowed(const baldr::DirectedEdge* edge) {
  return (edge->forwardaccess() & kBicycleAccess);
}

// Check if access is allowed at the specified node.
bool BicycleCost::Allowed(const baldr::NodeInfo* node) {
  // TODO
  return true;
}

// Get the cost to traverse the edge in seconds.
float BicycleCost::Get(const DirectedEdge* edge) {
  // TODO - cost based on desirability of cycling.
  return edge->length() / edge->speed();
}

float BicycleCost::Seconds(const DirectedEdge* edge) {
  // TODO - what to use for speed?
  return edge->length() / edge->speed();
}

/**
 * Get the cost factor for A* heuristics. This factor is multiplied
 * with the distance to the destination to produce an estimate of the
 * minimum cost to the destination. The A* heuristic must underestimate the
 * cost to the destination. So a time based estimate based on speed should
 * assume the maximum speed is used to the destination such that the time
 * estimate is less than the least possible time along roads.
 */
float BicycleCost::AStarCostFactor() {
  // This should be multiplied by the maximum speed expected.
  return 1.0f / 70.0f;
}

float BicycleCost::UnitSize() const {
  // Consider anything within 2 sec to be same cost
  return 2.0f;
}

}
}
