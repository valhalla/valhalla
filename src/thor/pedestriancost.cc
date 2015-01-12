#include "thor/pedestriancost.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

/**
 * Derived class providing dynamic edge costing for pedestrian routes.
 */
class PedestrianCost : public DynamicCost {
 public:
  PedestrianCost();

  virtual ~PedestrianCost();

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters.
   * @param edge      Pointer to a directed edge.
   * @param uturn     Is this a Uturn?
   * @param dist2dest Distance to the destination.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge, const bool uturn,
                       const float dist2dest) const;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present. (TODO - others?)
   * @param  edge  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const;

  /**
   * Get the cost given a directed edge.
   * @param edge  Pointer to a directed edge.
   * @return  Returns the cost to traverse the edge.
   */
  virtual float Get(const baldr::DirectedEdge* edge) const;

  /**
   * Returns the time (in seconds) to traverse the edge.
   * @param edge  Pointer to a directed edge.
   * @return  Returns the time in seconds to traverse the edge.
   */
  virtual float Seconds(const baldr::DirectedEdge* edge) const;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const;

  /**
   * Get the general unit size that can be considered as equal for sorting
   * purposes. The A* method uses an approximate bucket sort, and this value
   * is used to size the buckets used for sorting. For example, for time
   * based costs one might compute costs in seconds and consider any time
   * within 1.5 seconds of each other as being equal (for sorting purposes).
   * @return  Returns the unit size for sorting.
   */
  virtual float UnitSize() const;

 private:
  // Walking speed (default to 5.1 km / hour)
  float walkingspeed_;
  // Favor walkways and paths? (default to 0.95f)
  float favorwalkways_;
};

// Constructor
PedestrianCost::PedestrianCost()
    : DynamicCost(),
      walkingspeed_(5.1f),
      favorwalkways_(0.90f) {

  //TODO: load up stuff from ptree config
  //We want to use walkways
  favorwalkways_ = .5f;
}

// Destructor
PedestrianCost::~PedestrianCost() {
}

// Check if access is allowed on the specified edge.
bool PedestrianCost::Allowed(const baldr::DirectedEdge* edge,
                             const bool uturn, const float dist2dest) const {
  // Do not allow upward transitions (always stay at local level)
  // Also do not allow Uturns or entering no-thru edges
  if (edge->trans_up() || uturn || (edge->not_thru() && dist2dest > 5.0)) {
    return false;
  }
  return (edge->forwardaccess() & kPedestrianAccess);
}

// Check if access is allowed at the specified node.
bool PedestrianCost::Allowed(const baldr::NodeInfo* node) const {
  // TODO
  return true;
}

// Get the cost to traverse the edge
float PedestrianCost::Get(const DirectedEdge* edge) const {
  // TODO - slightly avoid steps?

  // Slightly favor walkways/paths.
  if (edge->use() == Use::kFootway)
    return edge->length() * favorwalkways_;

  return edge->length();

}

// Returns the time (in seconds) to traverse the edge.
float PedestrianCost::Seconds(const baldr::DirectedEdge* edge) const {
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
float PedestrianCost::AStarCostFactor() const {
  // Multiplied by the factor used to favor walkways/paths if < 1.0f
  if (favorwalkways_ < 1.0f)
    return 1.0f * favorwalkways_;

  return 1.0f;
}

float PedestrianCost::UnitSize() const {
  // Consider anything within 5 m to be same cost
  return 0.005f;
}

cost_ptr_t CreatePedestrianCost(/*pt::ptree const& config*/){
  return std::make_shared<PedestrianCost>();
}

}
}
