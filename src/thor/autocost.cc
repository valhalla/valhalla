#include "thor/autocost.h"

#include <iostream>
#include <valhalla/midgard/constants.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// Constructor
AutoCost::AutoCost()
    : DynamicCost() {
  // Create speed cost table
  speedfactor_[0] = kSecPerHour;  // TODO - what to make speed=0?
  for (uint32_t s = 1; s < 255; s++) {
    speedfactor_[s] = kSecPerHour / static_cast<float>(s);
  }
}

// Destructor
AutoCost::~AutoCost() {
}

// Check if access is allowed on the specified edge.
bool AutoCost::Allowed(const baldr::DirectedEdge* edge,  const bool uturn,
                       const float dist2dest) {
  // Do not allow Uturns or entering no-thru edges. TODO - evaluate later!
  if (uturn || (edge->not_thru() && dist2dest > 5.0)) {
    return false;
  }
  return (edge->forwardaccess() & kAutoAccess);
}

// Check if access is allowed at the specified node.
bool AutoCost::Allowed(const baldr::NodeInfo* node) {
  // TODO
  return true;
}

// Get the cost to traverse the edge in seconds.
float AutoCost::Get(const DirectedEdge* edge) {
  if (edge->speed() > 150) {
    std::cout << "Speed = " << static_cast<uint32_t>(edge->speed()) << std::endl;
  }
  return edge->length() * speedfactor_[edge->speed()];
}

float AutoCost::Seconds(const DirectedEdge* edge) {
  return edge->length() * speedfactor_[edge->speed()];
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
  return speedfactor_[120];
}

float AutoCost::UnitSize() const {
  // Consider anything within 1 sec to be same cost
  return 1.0f;
}

}
}
