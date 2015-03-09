#include "thor/autocost.h"

#include <iostream>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// Maximum speed expected - this is used for the A* heuristic
constexpr uint32_t kMaxSpeedKph = 140;

/**
 * Derived class providing dynamic edge costing for "direct" auto routes. This
 * is a route that is generally shortest time but uses route hierarchies that
 * can result in slightly longer routes that avoid shortcuts on residential
 * roads.
 */
class AutoCost : public DynamicCost {
 public:
  /**
   * Construct auto costing. Pass in configuration using property tree.
   * @param  config  Property tree with configuration/options.
   */
  AutoCost(const boost::property_tree::ptree& config);

  virtual ~AutoCost();

  /**
   * Does the costing allow hierarchy transitions. Auto costing will allow
   * transitions by default.
   * @return  Returns true if the costing model allows hierarchy transitions).
   */
   virtual bool AllowTransitions() const;

  /**
   * Does the costing method allow multiple passes (with relaxed hierarchy
   * limits).
   * @return  Returns true if the costing model allows multiple passes.
   */
  virtual bool AllowMultiPass() const;

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
                       const EdgeLabel& pred) const;

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
  virtual Cost TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred) const;

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
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each edges attribution
   * @return Function/functor to be used in filtering out edges
   */
  virtual const loki::EdgeFilter GetFilter() const {
    //throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge){
      return edge->trans_up() || edge->trans_down() || !(edge->forwardaccess() & kAutoAccess);
    };
  }

 protected:
  float speedfactor_[256];
};


// Constructor
AutoCost::AutoCost(const boost::property_tree::ptree& config)
    : DynamicCost(config) {
  // Create speed cost table
  speedfactor_[0] = kSecPerHour;  // TODO - what to make speed=0?
  for (uint32_t s = 1; s < 255; s++) {
    speedfactor_[s] = (kSecPerHour * 0.001f) / static_cast<float>(s);
  }
}

// Destructor
AutoCost::~AutoCost() {
}

// Auto costing will allow hierarchy transitions by default.
bool AutoCost::AllowTransitions() const {
  return true;
}

// Does the costing method allow multiple passes (with relaxed hierarchy
// limits).
bool AutoCost::AllowMultiPass() const {
  return true;
}

// Check if access is allowed on the specified edge.
bool AutoCost::Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred) const {
  // Check access, simple turn restrictions, and Uturns.
  // TODO - perhaps allow Uturns at dead-end nodes?
  if (!(edge->forwardaccess() & kAutoAccess) ||
      pred.opp_local_idx() == edge->localedgeidx() ||
     (pred.restrictions() & (1 << edge->localedgeidx()))) {
    return false;
  }

  // Do not allow entering not-thru edges except near the destination.
  if (edge->not_thru() && pred.distance() > not_thru_distance_) {
    return false;
  }
  return true;
}

// Check if access is allowed at the specified node.
bool AutoCost::Allowed(const baldr::NodeInfo* node) const  {
  return (node->access() & kAutoAccess);
}

// Get the cost to traverse the edge in seconds
Cost AutoCost::EdgeCost(const DirectedEdge* edge) const {
#ifdef LOGGING_LEVEL_WARN
  if (edge->speed() > kMaxSpeedKph) {
    LOG_WARN("Speed = " + std::to_string(edge->speed()));
  }
#endif
  float sec = (edge->length() * speedfactor_[edge->speed()]);
  return Cost(sec, sec);
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost AutoCost::TransitionCost(const baldr::DirectedEdge* edge,
                               const baldr::NodeInfo* node,
                               const EdgeLabel& pred) const {
  return Cost(0.0f, 0.0f);
}

// Get the cost factor for A* heuristics. This factor is multiplied
// with the distance to the destination to produce an estimate of the
// minimum cost to the destination. The A* heuristic must underestimate the
// cost to the destination. So a time based estimate based on speed should
// assume the maximum speed is used to the destination such that the time
// estimate is less than the least possible time along roads.
float AutoCost::AStarCostFactor() const {
  return speedfactor_[kMaxSpeedKph];
}

cost_ptr_t CreateAutoCost(const boost::property_tree::ptree& config) {
  return std::make_shared<AutoCost>(config);
}

/**
 * Derived class providing dynamic edge costing for pedestrian routes.
 */
class AutoShorterCost : public AutoCost {
 public:
  /**
   * Construct auto costing for shorter (not absolute shortest) path.
   * Pass in configuration using property tree.
   * @param  config  Property tree with configuration/options.
   */
  AutoShorterCost(const boost::property_tree::ptree& config);

  virtual ~AutoShorterCost();

  /**
    * Returns the cost to traverse the edge and an estimate of the actual time
    * (in seconds) to traverse the edge.
    * @param  edge     Pointer to a directed edge.
    * @param  seconds  (OUT) - elapsed time in seconds to traverse the edge
    * @return  Returns the cost to traverse the edge.
    */
   virtual Cost EdgeCost(const baldr::DirectedEdge* edge) const;

   /**
    * Get the cost factor for A* heuristics. This factor is multiplied
    * with the distance to the destination to produce an estimate of the
    * minimum cost to the destination. The A* heuristic must underestimate the
    * cost to the destination. So a time based estimate based on speed should
    * assume the maximum speed is used to the destination such that the time
    * estimate is less than the least possible time along roads.
    */
   virtual float AStarCostFactor() const;

 protected:
  float adjspeedfactor_[256];
};


// Constructor
AutoShorterCost::AutoShorterCost(const boost::property_tree::ptree& config)
    : AutoCost(config) {
  // Create speed cost table that reduces the impact of speed
  adjspeedfactor_[0] = kSecPerHour;  // TODO - what to make speed=0?
  for (uint32_t s = 1; s < 255; s++) {
    adjspeedfactor_[s] = (kSecPerHour * 0.001f) / sqrtf(static_cast<float>(s));
  }
}

// Destructor
AutoShorterCost::~AutoShorterCost() {
}

/**
 * Returns the cost to traverse the edge and an estimate of the actual time
 * (in seconds) to traverse the edge.
 * @param  edge     Pointer to a directed edge.
 * @param  seconds  (OUT) - elapsed time in seconds to traverse the edge
 * @return  Returns the cost to traverse the edge.
 */
Cost AutoShorterCost::EdgeCost(const baldr::DirectedEdge* edge) const {
  return Cost(edge->length() * adjspeedfactor_[edge->speed()],
              edge->length() * speedfactor_[edge->speed()]);
}

float AutoShorterCost::AStarCostFactor() const {
  return adjspeedfactor_[120];
}

cost_ptr_t CreateAutoShorterCost(const boost::property_tree::ptree& config) {
  return std::make_shared<AutoShorterCost>(config);
}

}
}
