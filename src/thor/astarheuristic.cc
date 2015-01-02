#include "thor/astarheuristic.h"

using namespace valhalla::midgard;

namespace valhalla{
namespace thor{

  // Default constructor
  AStarHeuristic::AStarHeuristic()
    : costfactor_(1.0f) {
  }

  // Initializes the AStar heuristic
  void AStarHeuristic::Init(const PointLL& ll, const float factor) {
    distapprox_.SetTestPoint(ll);
    costfactor_ = factor;
  }

  // Ge tthe distance to the destination
  float AStarHeuristic::GetDistance(const midgard::PointLL& ll) {
    return sqrtf(distapprox_.DistanceSquared(ll));
  }

  // Get the A* heuristic given the lat,lng.
  float AStarHeuristic::Get(const midgard::PointLL& ll) {
    return sqrtf(distapprox_.DistanceSquared(ll)) * costfactor_;
  }

  // Get the A* heuristic given the distance to the destination.
  float AStarHeuristic::Get(const float dist) {
    return dist * costfactor_;
  }

}
}
