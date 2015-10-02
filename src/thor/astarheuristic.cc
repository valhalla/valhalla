#include "thor/astarheuristic.h"

using namespace valhalla::midgard;

namespace valhalla {
namespace thor {

// Default constructor
AStarHeuristic::AStarHeuristic()
  : costfactor_(1.0f), distapprox_({}) {
}

// Initializes the AStar heuristic
void AStarHeuristic::Init(const PointLL& ll, const float factor) {
  distapprox_.SetTestPoint(ll);
  costfactor_ = factor;
}

// Get the distance to the destination
float AStarHeuristic::GetDistance(const midgard::PointLL& ll) const {
  return sqrtf(distapprox_.DistanceSquared(ll));
}

// Get the A* heuristic given the lat,lng.
float AStarHeuristic::Get(const midgard::PointLL& ll) const {
  return sqrtf(distapprox_.DistanceSquared(ll)) * costfactor_;
}

// Get the A* heuristic given the lat,lng. Returns distance via and argument.
float AStarHeuristic::Get(const midgard::PointLL& ll, float& dist) const {
  dist = sqrtf(distapprox_.DistanceSquared(ll));
  return  dist * costfactor_;
}

// Get the A* heuristic given the distance to the destination.
float AStarHeuristic::Get(const float dist) const {
  return dist * costfactor_;
}

}
}
