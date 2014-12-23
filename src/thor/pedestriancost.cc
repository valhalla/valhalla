#include "thor/pedestriancost.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// Constructor
PedestrianCost::PedestrianCost()
    : DynamicCost(),
      walkingspeed_(5.1f) {
}

// Destructor
PedestrianCost::~PedestrianCost() {
}

// Check if access is allowed on the specified edge.
bool PedestrianCost::Allowed(const baldr::DirectedEdge* edge) {
  return (edge->forwardaccess() & kPedestrianAccess);
}

// Check if access is allowed at the specified node.
bool PedestrianCost::Allowed(const baldr::NodeInfo* node) {
  // TODO
  return true;
}

// Get the cost to traverse the edge
float PedestrianCost::Get(const DirectedEdge* edge) {
  // TODO - slightly favor walkways/paths.
  // TODO - slightly avoid steps?
  return edge->length();
}

// Returns the time (in seconds) to traverse the edge.
float PedestrianCost::Seconds(const baldr::DirectedEdge* edge) {
  return edge->length() / walkingspeed_;
}

/**
 * Get the cost factor for A* heuristics. This factor is multiplied
 * with the distance to the destination to produce an estimate of the
 * minimum cost to the destination. The A* heuristic must underestimate the
 * cost to the destination. So a time based estimate based on speed should
 * assume the maximum speed is used to the destination such that the time
 * estimate is less than the least possible time along roads.
 */
float PedestrianCost::AStarCostFactor() {
  // This should be multiplied by the factor used to favor walkways/paths
  return 1.0f;
}

float PedestrianCost::UnitSize() const {
  // Consider anything within 5 m to be same cost
  return 0.005f;
}

// Set the walking speed
void PedestrianCost::set_walkingspeed(const float speed) {
  walkingspeed_ = speed;
}

}
}
