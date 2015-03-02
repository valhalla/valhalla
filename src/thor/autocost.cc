#include "thor/autocost.h"

#include <iostream>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

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
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters.
   * @param edge      Pointer to a directed edge.
   * @param restriction Restriction mask. Identifies the edges at the end
   *                  node onto which turns are restricted at all times.
   *                  This mask is compared to the next edge's localedgeidx.
   * @param uturn     Is this a Uturn?
   * @param dist2dest Distance to the destination.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const uint32_t restriction, const bool uturn,
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

// Check if access is allowed on the specified edge.
bool AutoCost::Allowed(const baldr::DirectedEdge* edge,
                       const uint32_t restriction, const bool uturn,
                       const float dist2dest) const {
  // Check access and simple turn restrictions
  if (!(edge->forwardaccess() & kAutoAccess) ||
     (restriction & (1 << edge->localedgeidx()))) {
    return false;
  }

  // Do not allow Uturns. TODO - evaluate later! - Uturns on different
  // hierarchy levels present problems. May want to set implied turn
  // restrictions
  if (uturn)
    return false;

  // Do not allow entering not-thru edges except near the destination.
  if (edge->not_thru() && dist2dest > not_thru_distance_) {
    return false;
  }
  return true;
}

// Check if access is allowed at the specified node.
bool AutoCost::Allowed(const baldr::NodeInfo* node) const  {
  return (node->access() & kAutoAccess);
}

// Get the cost to traverse the edge in seconds.
float AutoCost::Get(const DirectedEdge* edge) const {
#ifdef LOGGING_LEVEL_WARN
  if (edge->speed() > 150) {
    LOG_WARN("Speed = " + std::to_string(edge->speed()));
  }
#endif
  return edge->length() * speedfactor_[edge->speed()];
}

float AutoCost::Seconds(const DirectedEdge* edge) const {
  return edge->length() * speedfactor_[edge->speed()];
}

// Get the cost factor for A* heuristics. This factor is multiplied
// with the distance to the destination to produce an estimate of the
// minimum cost to the destination. The A* heuristic must underestimate the
// cost to the destination. So a time based estimate based on speed should
// assume the maximum speed is used to the destination such that the time
// estimate is less than the least possible time along roads.
float AutoCost::AStarCostFactor() const {
  return speedfactor_[120];
}

float AutoCost::UnitSize() const {
  // Consider anything within 1 sec to be same cost
  return 1.0f;
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
   * Get the cost given a directed edge.
   * @param edge  Pointer to a directed edge.
   * @return  Returns the cost to traverse the edge.
   */
   virtual float Get(const baldr::DirectedEdge* edge) const;

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

// Get the cost to traverse the edge in seconds.
float AutoShorterCost::Get(const DirectedEdge* edge) const {
  return edge->length() * adjspeedfactor_[edge->speed()];
}

float AutoShorterCost::AStarCostFactor() const {
  return adjspeedfactor_[120];
}

cost_ptr_t CreateAutoShorterCost(const boost::property_tree::ptree& config) {
  return std::make_shared<AutoShorterCost>(config);
}

}
}
