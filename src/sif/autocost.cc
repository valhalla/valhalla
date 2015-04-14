#include "sif/autocost.h"

#include <iostream>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {
constexpr float kDefaultManeuverPenalty         = 5.0f;   // Seconds
constexpr float kDefaultGateCost                = 30.0f;  // Seconds
constexpr float kDefaultTollBoothCost           = 15.0f;  // Seconds
constexpr float kDefaultTollBoothPenalty        = 0.0f;   // Seconds
constexpr float kDefaultCountryCrossingCost     = 600.0f; // Seconds
constexpr float kDefaultCountryCrossingPenalty  = 0.0f;   // Seconds
// Maximum speed expected - this is used for the A* heuristic
constexpr uint32_t kMaxSpeedKph = 140;
}

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
   * be restricted if bollards or gates are present.
   * @param  edge  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const;

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge  Pointer to a directed edge.
   * @param   density  Relative road density.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const uint32_t density) const;

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  edge  Directed edge (the to edge)
   * @param  node  Node (intersection) where transition occurs.
   * @param  pred  Predecessor edge information.
   * @param   to_idx Index of the "to" directed edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred,
                              const uint32_t to_idx) const;

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
  virtual const EdgeFilter GetFilter() const {
    //throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge){
      return edge->trans_up() || edge->trans_down() || !(edge->forwardaccess() & kAutoAccess);
    };
  }

 protected:
  float speedfactor_[256];
  float density_factor_[16]; // Density factor
  float maneuver_penalty_;   // Penalty (seconds) when inconsistent names
  float gate_cost_;          // Cost (seconds) to go through gate
  float tollbooth_cost_;     // Cost (seconds) to go through toll booth
  float tollbooth_penalty_;  // Penalty (seconds) to go through a toll booth
  float country_crossing_cost_;     // Cost (seconds) to go through toll booth
  float country_crossing_penalty_;  // Penalty (seconds) to go across a country border
  /**
   * Compute a turn cost based on the turn type, crossing flag,
   * and whether right or left side of road driving.
   * @param  turn_type  Turn type (see baldr/turn.h)
   * @param  crossing   Crossing another road if true.
   * @param  drive_on_right  Right hand side of road driving if true.
   */
  float TurnCost(const baldr::Turn::Type turn_type, const bool crossing,
                 const bool drive_on_right) const;
};


// Constructor
AutoCost::AutoCost(const boost::property_tree::ptree& pt)
    : DynamicCost(pt) {

  maneuver_penalty_ = pt.get<float>("maneuver_penalty",
                                    kDefaultManeuverPenalty);
  gate_cost_ = pt.get<float>("gate_cost", kDefaultGateCost);
  tollbooth_cost_ = pt.get<float>("toll_booth_cost", kDefaultTollBoothCost);
  tollbooth_penalty_ = pt.get<float>("toll_booth_penalty",
                                     kDefaultTollBoothPenalty);

  country_crossing_cost_ = pt.get<float>("country_crossing_cost", kDefaultCountryCrossingCost);
  country_crossing_penalty_ = pt.get<float>("country_crossing_penalty",
                                           kDefaultCountryCrossingPenalty);

  // Create speed cost table
  speedfactor_[0] = kSecPerHour;  // TODO - what to make speed=0?
  for (uint32_t s = 1; s < 255; s++) {
    speedfactor_[s] = (kSecPerHour * 0.001f) / static_cast<float>(s);
  }
  for (uint32_t d = 0; d < 16; d++) {
    density_factor_[d] = 0.85f + (d * 0.025f);
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

// Check if access is allowed on the specified edge. Not worh checking
// not_thru due to hierarchy transitions
bool AutoCost::Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred) const {
  // Check access, Uturn, and simple turn restriction.
  // TODO - perhaps allow Uturns at dead-end nodes?
  if (!(edge->forwardaccess() & kAutoAccess) ||
      (pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
       edge->surface() == Surface::kImpassable) {
    return false;
  }

  return true;
}

// Check if access is allowed at the specified node.
bool AutoCost::Allowed(const baldr::NodeInfo* node) const  {
  return (node->access() & kAutoAccess);
}

// Get the cost to traverse the edge in seconds
Cost AutoCost::EdgeCost(const DirectedEdge* edge,
                        const uint32_t density) const {
#ifdef LOGGING_LEVEL_WARN
  if (edge->speed() > kMaxSpeedKph) {
    LOG_WARN("Speed = " + std::to_string(edge->speed()));
  }
#endif
  float sec = (edge->length() * speedfactor_[edge->speed()]);
  return Cost(sec * density_factor_[density], sec);
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost AutoCost::TransitionCost(const baldr::DirectedEdge* edge,
                               const baldr::NodeInfo* node,
                               const EdgeLabel& pred,
                               const uint32_t to_idx) const {
  // Special cases: gate, toll booth, false intersections
  // TODO - do we want toll booth penalties?
  if (node->intersection() == IntersectionType::kFalse) {
    return { 0.0f, 0.0f };
  } else if (edge->ctry_crossing()) {
    return { country_crossing_cost_ + country_crossing_penalty_, country_crossing_cost_ };
  } else if (node->type() == NodeType::kGate) {
    return { gate_cost_, gate_cost_ };
  } else if (node->type() == NodeType::kTollBooth) {
    return { tollbooth_cost_ + tollbooth_penalty_, tollbooth_cost_ };
  } else {
    // Transition cost = stopimpact * turncost + maneuverpenalty
    uint32_t idx = pred.opp_local_idx();
    float seconds = edge->stopimpact(idx) * TurnCost(edge->turntype(idx),
                         edge->edge_to_right(idx) && edge->edge_to_left(idx),
                         edge->drive_on_right());
    return (node->name_consistency(idx, to_idx)) ?
              Cost(seconds, seconds) :
              Cost(seconds + maneuver_penalty_, seconds);
  }
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

// Compute a turn cost based on the turn type, crossing flag,
//  and whether right or left side of road driving.
float AutoCost::TurnCost(const baldr::Turn::Type turn_type,
                         const bool crossing,
                         const bool drive_on_right) const {
  if (crossing) {
    return 2.0f;
  }
  if (drive_on_right) {
    if (turn_type <= Turn::Type::kRight)
      return 0.5f;
    else if (turn_type == Turn::Type::kSharpRight)
      return 1.0f;
    else if (turn_type <= Turn::Type::kSharpLeft) // Reverse and sharp left
      return 5.0f;
    else if (turn_type <= Turn::Type::kLeft)
      return 2.5f;
    else // Slight left
      return 0.75f;
  } else {
    // TODO - reverse logic
    return 1.0f;
  }
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
   * @param   density  Relative road density.
   * @return  Returns the cost to traverse the edge.
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const uint32_t density) const;

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
AutoShorterCost::AutoShorterCost(const boost::property_tree::ptree& pt)
    : AutoCost(pt) {
  // Create speed cost table that reduces the impact of speed
  adjspeedfactor_[0] = kSecPerHour;  // TODO - what to make speed=0?
  for (uint32_t s = 1; s < 255; s++) {
    adjspeedfactor_[s] = (kSecPerHour * 0.001f) / sqrtf(static_cast<float>(s));
  }
}

// Destructor
AutoShorterCost::~AutoShorterCost() {
}

// Returns the cost to traverse the edge and an estimate of the actual time
// (in seconds) to traverse the edge.
Cost AutoShorterCost::EdgeCost(const baldr::DirectedEdge* edge,
                               const uint32_t density) const {
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
