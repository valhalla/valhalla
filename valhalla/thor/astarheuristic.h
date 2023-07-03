#ifndef VALHALLA_THOR_ASTARHEURISTIC_H_
#define VALHALLA_THOR_ASTARHEURISTIC_H_

#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace thor {

/**
 * Class to calculate A* cost heuristics based on distances of nodes from
 * a destination within the shortest path computation.
 */
class AStarHeuristic {
public:
  /**
   * Constructor.
   */
  AStarHeuristic() : distapprox_({}), costfactor_(1.0f) {
  }

  /**
   * Sets the destination latitude and longitude positions in the
   * distance approximator.
   * @param  ll      Latitude, longitude (in degrees) of the destination.
   * @param  factor  Costing factor to multiply distance by. This factor
   *                 needs to be tied to the costing model to multiply
   *                 distance that will underestimate the cost to the
   *                 destination, but keep close to a reasonable true
   *                 cost so that performance is kept high.
   */
  void Init(const midgard::PointLL& ll, const float factor) {
    distapprox_.SetTestPoint(ll);
    costfactor_ = factor;
  }

  /**
   * Get the distance to the destination given the lat,lng.
   * @param   ll  Current latitude, longitude.
   * @return  Returns the distance to the destination.
   */
  float GetDistance(const midgard::PointLL& ll) const {
    return sqrtf(distapprox_.DistanceSquared(ll));
  }

  /**
   * Get the A* heuristic given the distance to the destination.
   * @param   distance  Distance (meters) to the destination.
   * @return  Returns an estimate of the cost to the destination.
   *          For A* shortest path this MUST UNDERESTIMATE the true cost.
   */
  float Get(const float distance) const {
    return distance * costfactor_;
  }

  /**
   * Get the A* heuristic given the current lat,lng.
   * @param   ll  Lat,lng
   * @return  Returns an estimate of the cost to the destination.
   *          For A* shortest path this MUST UNDERESTIMATE the true cost.
   */
  float Get(const midgard::PointLL& ll) const {
    return sqrtf(distapprox_.DistanceSquared(ll)) * costfactor_;
  }

  /**
   * Get the A* heuristic given the lat,lng. Also return distance via
   * an argument.
   * @param   ll  Lat,lng
   * @param   dist  Distance (meters) to the destination.
   * @return  Returns an estimate of the cost to the destination.
   *          For A* shortest path this MUST UNDERESTIMATE the true cost.
   */
  float Get(const midgard::PointLL& ll, float& dist) const {
    dist = sqrtf(distapprox_.DistanceSquared(ll));
    return dist * costfactor_;
  }

private:
  midgard::DistanceApproximator<midgard::PointLL> distapprox_; // Distance approximation
  float costfactor_; // Cost factor - ensures the cost estimate
                     // underestimates the true cost.
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_ASTARHEURISTIC_H_
