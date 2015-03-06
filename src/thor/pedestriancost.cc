#include "thor/pedestriancost.h"

#include <valhalla/midgard/constants.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

/**
 * Derived class providing dynamic edge costing for pedestrian routes.
 */
class PedestrianCost : public DynamicCost {
 public:
  /**
   * Constructor. Configuration / options for pedestrian costing are provided
   * via a property tree.
   * @param  config  Property tree with configuration/options.
   */
  PedestrianCost(const boost::property_tree::ptree& config);

  virtual ~PedestrianCost();

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters.
   * @param  edge  Pointer to a directed edge.
   * @param  pred  Predecessor edge information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const bool uturn) const;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present. (TODO - others?)
   * @param  edge  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const;

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge  Pointer to a directed edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge) const;

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  edge  Directed edge (the to edge)
   * @param  node  Node (intersection) where transition occurs.
   * @param  pred  Predecessor edge information.
   * @return  Returns the cost and time (seconds)
   */
// TODO - add logic for transition costs!
//  virtual Cost TransitionCost(const baldr::DirectedEdge* edge,
//                               const baldr::NodeInfo* node,
//                               const EdgeLabel& pred) const;

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
   * Override unit size since walking costs are higher range of vales
   */
  virtual uint32_t UnitSize() const;

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each edges attribution
   * @return Function/functor to be used in filtering out edges
   */
  virtual const loki::EdgeFilter GetFilter() const {
    //throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge){
      return edge->trans_up() || edge->trans_down() || !(edge->forwardaccess() & kPedestrianAccess);
    };
  }

 private:
  // Walking speed (default to 5.1 km / hour)
  float walkingspeed_;

  float speedfactor_;

  // Favor walkways and paths? (default to 0.9f)
  float favorwalkways_;
};

// Constructor
// TODO: parse pedestrian configuration from ptree config
PedestrianCost::PedestrianCost(const boost::property_tree::ptree& config)
    : DynamicCost(config),
      walkingspeed_(5.1f),
      favorwalkways_(0.9f) {
  speedfactor_ = (kSecPerHour * 0.001f) / walkingspeed_;
}

// Destructor
PedestrianCost::~PedestrianCost() {
}

// Check if access is allowed on the specified edge.
bool PedestrianCost::Allowed(const baldr::DirectedEdge* edge,
                             const EdgeLabel& pred,
                             const bool uturn) const {
  // Return false if no pedestrian access. Disallow uturns or transition
  // onto not-thru edges (except near the destination)
  return ((edge->forwardaccess() & kPedestrianAccess) &&
          !uturn &&
          !(edge->not_thru() && pred.distance() > not_thru_distance_));
}

// Check if access is allowed at the specified node.
bool PedestrianCost::Allowed(const baldr::NodeInfo* node) const {
  return (node->access() & kPedestrianAccess);
}

// Returns the cost to traverse the edge and an estimate of the actual time
// (in seconds) to traverse the edge.
Cost PedestrianCost::EdgeCost(const baldr::DirectedEdge* edge) const {
  // Slightly favor walkways/paths. TODO - penalize stairs
  float sec = edge->length() * speedfactor_;
  if (edge->use() == Use::kFootway) {
    return Cost(sec * favorwalkways_, sec);
  } else {
    return Cost(sec, sec);
  }
};

/**
 * Get the cost factor for A* heuristics. This factor is multiplied
 * with the distance to the destination to produce an estimate of the
 * minimum cost to the destination. The A* heuristic must underestimate the
 * cost to the destination. So a time based estimate based on speed should
 * assume the maximum speed is used to the destination such that the time
 * estimate is less than the least possible time along roads.
 */
float PedestrianCost::AStarCostFactor() const {
  // Use the factor to favor walkways/paths if < 1.0f
  return (favorwalkways_ < 1.0f) ? favorwalkways_ * speedfactor_ : speedfactor_;
}

//  Override unit size since walking costs are higher range of values
uint32_t PedestrianCost::UnitSize() const {
  return 5;
}
cost_ptr_t CreatePedestrianCost(const boost::property_tree::ptree& config) {
  return std::make_shared<PedestrianCost>(config);
}

}
}
