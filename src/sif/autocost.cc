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
constexpr float kDefaultDestinationOnlyPenalty  = 600.0f; // Seconds
constexpr float kDefaultAlleyPenalty            = 30.0f;   // Seconds
constexpr float kDefaultGateCost                = 30.0f;  // Seconds
constexpr float kDefaultTollBoothCost           = 15.0f;  // Seconds
constexpr float kDefaultTollBoothPenalty        = 0.0f;   // Seconds
constexpr float kDefaultCountryCrossingCost     = 600.0f; // Seconds
constexpr float kDefaultCountryCrossingPenalty  = 0.0f;   // Seconds

// Default turn costs
constexpr float kTCStraight         = 0.5f;
constexpr float kTCSlight           = 0.75f;
constexpr float kTCFavorable        = 1.0f;
constexpr float kTCFavorableSharp   = 1.5f;
constexpr float kTCCrossing         = 2.0f;
constexpr float kTCUnfavorable      = 2.5f;
constexpr float kTCUnfavorableSharp = 3.5f;
constexpr float kTCReverse          = 5.0f;
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
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges are
   * provided.
   * @param  edge  Pointer to a directed edge.
   * @param  opp_edge  Pointer to the opposing directed edge.
   * @param  opp_pred_edge  Pointer to the opposing directed edge to the
   *                        predecessor.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                 const baldr::DirectedEdge* opp_edge,
                 const baldr::DirectedEdge* opp_pred_edge) const;

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
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred) const;

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * @param  idx   Directed edge local index
   * @param  node  Node (intersection) where transition occurs.
   * @param  opp_edge  Pointer to the opposing directed edge - this is the
   *                   "from" or predecessor edge in the transition.
   * @param  opp_pred_edge  Pointer to the opposing directed edge to the
   *                        predecessor. This is the "to" edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                              const baldr::NodeInfo* node,
                              const baldr::DirectedEdge* opp_edge,
                              const baldr::DirectedEdge* opp_pred_edge) const;

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
  float density_factor_[16];        // Density factor
  float maneuver_penalty_;          // Penalty (seconds) when inconsistent names
  float destination_only_penalty_;  // Penalty (seconds) using a driveway or parking aisle
  float gate_cost_;                 // Cost (seconds) to go through gate
  float tollbooth_cost_;            // Cost (seconds) to go through toll booth
  float tollbooth_penalty_;         // Penalty (seconds) to go through a toll booth
  float alley_penalty_;             // Penalty (seconds) to use a alley
  float country_crossing_cost_;     // Cost (seconds) to go through toll booth
  float country_crossing_penalty_;  // Penalty (seconds) to go across a country border

  // Density factor used in edge transition costing
  std::vector<float> trans_density_factor_;

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
    : DynamicCost(pt, TravelMode::kDrive),
      trans_density_factor_{ 1.0f, 1.0f, 1.0f, 1.0f,
                             1.0f, 1.1f, 1.2f, 1.3f,
                             1.4f, 1.6f, 1.9f, 2.2f,
                             2.5f, 2.8f, 3.1f, 3.5f } {
  maneuver_penalty_ = pt.get<float>("maneuver_penalty",
                                    kDefaultManeuverPenalty);
  destination_only_penalty_ = pt.get<float>("destination_only_penalty",
                                            kDefaultDestinationOnlyPenalty);
  gate_cost_ = pt.get<float>("gate_cost", kDefaultGateCost);
  tollbooth_cost_ = pt.get<float>("toll_booth_cost", kDefaultTollBoothCost);
  tollbooth_penalty_ = pt.get<float>("toll_booth_penalty",
                                     kDefaultTollBoothPenalty);
  alley_penalty_ = pt.get<float>("alley_penalty",
                                 kDefaultAlleyPenalty);
  country_crossing_cost_ = pt.get<float>("country_crossing_cost",
                                           kDefaultCountryCrossingCost);
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

// Check if access is allowed on the specified edge. Not worth checking
// not_thru due to hierarchy transitions
bool AutoCost::Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred) const {
  // Check access, U-turn, and simple turn restriction.
  // TODO - perhaps allow U-turns at dead-end nodes?
  if (!(edge->forwardaccess() & kAutoAccess) ||
      (pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
       edge->surface() == Surface::kImpassable) {
    return false;
  }
  return true;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool AutoCost::AllowedReverse(const baldr::DirectedEdge* edge,
               const baldr::DirectedEdge* opp_edge,
               const baldr::DirectedEdge* opp_pred_edge) const {
  // Check access, U-turn, and simple turn restriction.
  // TODO - perhaps allow U-turns at dead-end nodes?
  if (!(opp_edge->forwardaccess() & kAutoAccess) ||
       (opp_pred_edge->localedgeidx() == edge->localedgeidx()) ||
       (opp_edge->restrictions() & (1 << opp_pred_edge->localedgeidx())) ||
       opp_edge->surface() == Surface::kImpassable) {
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
  float sec = (edge->length() * speedfactor_[edge->speed()]);
  return Cost(sec * density_factor_[density], sec);
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost AutoCost::TransitionCost(const baldr::DirectedEdge* edge,
                               const baldr::NodeInfo* node,
                               const EdgeLabel& pred) const {
  // Accumulate cost and penalty
  float seconds = 0.0f;
  float penalty = 0.0f;

  // Special cases with both time and penalty: country crossing,
  // gate, toll booth
  if (edge->ctry_crossing()) {
    seconds += country_crossing_cost_;
    penalty += country_crossing_penalty_;
  }
  if (node->type() == NodeType::kGate) {
    seconds += gate_cost_;
  } else if (node->type() == NodeType::kTollBooth) {
    seconds += tollbooth_cost_;
    penalty += tollbooth_penalty_;
  }

  // Additional penalties without any time cost
  uint32_t idx = pred.opp_local_idx();
  if (!pred.destonly() && edge->destonly()) {
    penalty += destination_only_penalty_;
  }
  if (pred.use() != Use::kAlley && edge->use() == Use::kAlley) {
    penalty += alley_penalty_;
  }
  if (!node->name_consistency(idx, edge->localedgeidx())) {
    penalty += maneuver_penalty_;
  }

  // Transition time = densityfactor * stopimpact * turncost
  if (edge->stopimpact(idx) > 0) {
    seconds += trans_density_factor_[node->density()] * edge->stopimpact(idx) *
               TurnCost(edge->turntype(idx),
                       edge->edge_to_right(idx) && edge->edge_to_left(idx),
                       edge->drive_on_right());
  }

  // Return cost (time and penalty)
  return { seconds + penalty, seconds };
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// pred is the opposing current edge in the reverse tree
// edge is the opposing predecessor in the reverse tree
Cost AutoCost::TransitionCostReverse(const uint32_t idx,
                            const baldr::NodeInfo* node,
                            const baldr::DirectedEdge* pred,
                            const baldr::DirectedEdge* edge) const {
  // Accumulate cost and penalty
  float seconds = 0.0f;
  float penalty = 0.0f;

  // Special cases with both time and penalty: country crossing,
  // gate, toll booth
  if (edge->ctry_crossing()) {
    seconds += country_crossing_cost_;
    penalty += country_crossing_penalty_;
  }
  if (node->type() == NodeType::kGate) {
    seconds += gate_cost_;
  } else if (node->type() == NodeType::kTollBooth) {
    seconds += tollbooth_cost_;
    penalty += tollbooth_penalty_;
  }

  // Additional penalties without any time cost
  if (!pred->destonly() && edge->destonly()) {
    penalty += destination_only_penalty_;
  }
  if (pred->use() != Use::kAlley && edge->use() == Use::kAlley) {
    penalty += alley_penalty_;
  }
  if (!node->name_consistency(idx, edge->localedgeidx())) {
    penalty += maneuver_penalty_;
  }


  // Transition time = densityfactor * stopimpact * turncost
  if (edge->stopimpact(idx) > 0) {
    seconds += trans_density_factor_[node->density()] * edge->stopimpact(idx) *
               TurnCost(edge->turntype(idx),
                        edge->edge_to_right(idx) && edge->edge_to_left(idx),
                        edge->drive_on_right());
  }

  // Return cost (time and penalty)
  return { seconds + penalty, seconds };
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
// and whether right or left side of road driving.
float AutoCost::TurnCost(const baldr::Turn::Type turn_type,
                         const bool crossing,
                         const bool drive_on_right) const {
  if (crossing) {
    return kTCCrossing;
  }

  switch (turn_type) {
  case Turn::Type::kStraight:
    return kTCStraight;

  case Turn::Type::kSlightLeft:
  case Turn::Type::kSlightRight:
    return kTCSlight;

  case Turn::Type::kRight:
    return (drive_on_right) ? kTCFavorable : kTCUnfavorable;

  case Turn::Type::kLeft:
    return (drive_on_right) ? kTCUnfavorable : kTCFavorable;

  case Turn::Type::kSharpRight:
    return (drive_on_right) ? kTCFavorableSharp : kTCUnfavorableSharp;

  case Turn::Type::kSharpLeft:
    return (drive_on_right) ? kTCUnfavorableSharp : kTCFavorableSharp;

  case Turn::Type::kReverse:
    return kTCReverse;
  }
}

cost_ptr_t CreateAutoCost(const boost::property_tree::ptree& config) {
  return std::make_shared<AutoCost>(config);
}

/**
 * Derived class providing an alternate costing for driving that is intended
 * to provide a short path.
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
  return adjspeedfactor_[kMaxSpeedKph];
}

cost_ptr_t CreateAutoShorterCost(const boost::property_tree::ptree& config) {
  return std::make_shared<AutoShorterCost>(config);
}

/**
 * Derived class providing bus costing for driving.
 */
class BusCost : public AutoCost {
 public:
  /**
   * Construct auto costing for shorter (not absolute shortest) path.
   * Pass in configuration using property tree.
   * @param  config  Property tree with configuration/options.
   */
  BusCost(const boost::property_tree::ptree& config);

  virtual ~BusCost();

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
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges are
   * provided.
   * @param  edge  Pointer to a directed edge.
   * @param  opp_edge  Pointer to the opposing directed edge.
   * @param  opp_pred_edge  Pointer to the opposing directed edge to the
   *                        predecessor.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                 const baldr::DirectedEdge* opp_edge,
                 const baldr::DirectedEdge* opp_pred_edge) const;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present.
   * @param  edge  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const;

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each edges attribution
   * @return Function/functor to be used in filtering out edges
   */
  virtual const EdgeFilter GetFilter() const;

};


// Constructor
BusCost::BusCost(const boost::property_tree::ptree& pt)
    : AutoCost(pt) {
}

// Destructor
BusCost::~BusCost() {
}

// Check if access is allowed on the specified edge. Not worth checking
// not_thru due to hierarchy transitions
bool BusCost::Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred) const {
  // Check access, U-turn, and simple turn restriction.
  // TODO - perhaps allow U-turns at dead-end nodes?
  if (!(edge->forwardaccess() & kBusAccess) ||
      (pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
       edge->surface() == Surface::kImpassable) {
    return false;
  }
  return true;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool BusCost::AllowedReverse(const baldr::DirectedEdge* edge,
               const baldr::DirectedEdge* opp_edge,
               const baldr::DirectedEdge* opp_pred_edge) const {
  // Check access, U-turn, and simple turn restriction.
  // TODO - perhaps allow U-turns at dead-end nodes?
  if (!(opp_edge->forwardaccess() & kBusAccess) ||
      (opp_pred_edge->opp_local_idx() == opp_edge->opp_local_idx()) ||
      (opp_edge->restrictions() & (1 << opp_pred_edge->localedgeidx())) ||
       opp_edge->surface() == Surface::kImpassable) {
    return false;
  }
  return true;
}

// Check if access is allowed at the specified node.
bool BusCost::Allowed(const baldr::NodeInfo* node) const  {
  return (node->access() & kBusAccess);
}

// Function/functor to be used in filtering out edges
const EdgeFilter BusCost::GetFilter() const {
  //throw back a lambda that checks the access for this type of costing
  return [](const baldr::DirectedEdge* edge){
    return edge->trans_up() || edge->trans_down() || !(edge->forwardaccess() & kBusAccess);
  };
}

cost_ptr_t CreateBusCost(const boost::property_tree::ptree& config) {
  return std::make_shared<BusCost>(config);
}


}
}
