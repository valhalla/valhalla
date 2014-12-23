#include "thor/autocost.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// Constructor
AutoCost::AutoCost()
    : DynamicCost() {
}

// Destructor
AutoCost::~AutoCost() {
}

// Check if access is allowed on the specified edge.
bool AutoCost::Allowed(const baldr::DirectedEdge* edge) {
  return (edge->forwardaccess() & kAutoAccess);
}

// Check if access is allowed at the specified node.
bool AutoCost::Allowed(const baldr::NodeInfo* node) {
  // TODO
  return true;
}

// Get the cost to traverse the edge in seconds.
float AutoCost::Get(const DirectedEdge* edge) {
  // TODO - to avoid division should we set speed as uint32 and
  // index into a speed cost array?
  return edge->length() / edge->speed();
}

float AutoCost::Seconds(const DirectedEdge* edge) {
  // TODO - to avoid division should we set speed as uint32 and
  // index into a speed cost array?
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
float AutoCost::AStarCostFactor() {
  // This should be multiplied by the maximum speed expected.
  return 1.0f / 200.0f;
}

float AutoCost::UnitSize() const {
  // Consider anything within 2 sec to be same cost
  return 2.0f;
}

}
}
