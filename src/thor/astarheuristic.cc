#include "thor/astarheuristic.h"

using namespace valhalla::midgard;

namespace valhalla{
namespace thor{

  /**
   * Constructor.
   */
  AStarHeuristic::AStarHeuristic()
    : costfactor_(1.0f) {
  }

  /**
   * Sets the destination latitude and longitude positions in the
   * distance approximator.
   * @param  ll   Latitude, longitude (in degrees) of the destination
   * @param  factor  Costing factor to multiply distance by. This factor
   *                 needs to be tied to the costing model to multiply
   *                 distance that will underestimate the cost to the
   *                 destination, but keep close to a reasonable true
   *                 cost so that performance is kept high.
   */
  void AStarHeuristic::Init(const PointLL& ll, const float factor) {
    distapprox_.SetTestPoint(ll);
    costfactor_ = factor;
  }

  /**
   * Get the A* heuristic given the current lat,lng.
   * @param  ll  Current latitude, longitude.
   * @return  Returns an estimate of the cost to the destination.
   *          For A* shortest path this MUST UNDERESTIMATE the true cost.
   */
  float AStarHeuristic::Get(const PointLL& ll) {
    return sqrtf(distapprox_.DistanceSquared(ll)) * costfactor_;
  }

}
}
