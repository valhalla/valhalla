#include "sif/autocost.h"

#include <iostream>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/accessrestriction.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {
constexpr float kDefaultManeuverPenalty         = 5.0f;   // Seconds
constexpr float kDefaultDestinationOnlyPenalty  = 600.0f; // Seconds
constexpr float kDefaultAlleyPenalty            = 30.0f;  // Seconds
constexpr float kDefaultGateCost                = 30.0f;  // Seconds
constexpr float kDefaultGatePenalty             = 300.0f; // Seconds
constexpr float kDefaultTollBoothCost           = 15.0f;  // Seconds
constexpr float kDefaultTollBoothPenalty        = 0.0f;   // Seconds
constexpr float kDefaultFerryCost               = 300.0f; // Seconds
constexpr float kDefaultCountryCrossingCost     = 600.0f; // Seconds
constexpr float kDefaultCountryCrossingPenalty  = 0.0f;   // Seconds

// Maximum ferry penalty (when use_ferry == 0). Can't make this too large
// since a ferry is sometimes required to complete a route.
constexpr float kMaxFerryPenalty = 6.0f * 3600.0f; // 6 hours

// Default turn costs
constexpr float kTCStraight         = 0.5f;
constexpr float kTCSlight           = 0.75f;
constexpr float kTCFavorable        = 1.0f;
constexpr float kTCFavorableSharp   = 1.5f;
constexpr float kTCCrossing         = 2.0f;
constexpr float kTCUnfavorable      = 2.5f;
constexpr float kTCUnfavorableSharp = 3.5f;
constexpr float kTCReverse          = 5.0f;

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = { kTCStraight, kTCSlight,
      kTCFavorable, kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
      kTCUnfavorable, kTCSlight };
constexpr float kLeftSideTurnCosts[]  = { kTCStraight, kTCSlight,
      kTCUnfavorable, kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
      kTCFavorable, kTCSlight };
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
   * @param  edge     Pointer to a directed edge.
   * @param  pred     Predecessor edge information.
   * @param  tile     current tile
   * @param  edgeid   edgeid that we care about
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid) const;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges are
   * provided.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           current tile
   * @param  edgeid         edgeid that we care about
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                 const EdgeLabel& pred,
                 const baldr::DirectedEdge* opp_edge,
                 const baldr::GraphTile*& tile,
                 const baldr::GraphId& edgeid) const;

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
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. Function/functor is also used to filter
   * edges not usable / inaccessible by automobile.
   */
  virtual const EdgeFilter GetEdgeFilter() const {
    // Throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge) {
      if (edge->trans_up() || edge->trans_down() ||
         !(edge->forwardaccess() & kAutoAccess))
        return 0.0f;
      else {
        // TODO - use classification/use to alter the factor
        return 1.0f;
      }
    };
  }

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each node's attribution
   * @return Function/functor to be used in filtering out nodes
   */
  virtual const NodeFilter GetNodeFilter() const {
    //throw back a lambda that checks the access for this type of costing
    return [](const baldr::NodeInfo* node){
      return !(node->access() & kAutoAccess);
    };
  }

 protected:
  float speedfactor_[256];
  float density_factor_[16];        // Density factor
  float maneuver_penalty_;          // Penalty (seconds) when inconsistent names
  float destination_only_penalty_;  // Penalty (seconds) using a driveway or parking aisle
  float gate_cost_;                 // Cost (seconds) to go through gate
  float gate_penalty_;              // Penalty (seconds) to go through gate
  float tollbooth_cost_;            // Cost (seconds) to go through toll booth
  float tollbooth_penalty_;         // Penalty (seconds) to go through a toll booth
  float ferry_cost_;                // Cost (seconds) to enter a ferry
  float ferry_penalty_;             // Penalty (seconds) to enter a ferry
  float ferry_weight_;              // Weighting to apply to ferry edges
  float alley_penalty_;             // Penalty (seconds) to use a alley
  float country_crossing_cost_;     // Cost (seconds) to go through toll booth
  float country_crossing_penalty_;  // Penalty (seconds) to go across a country border

  // Density factor used in edge transition costing
  std::vector<float> trans_density_factor_;
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
  gate_penalty_ = pt.get<float>("gate_penalty", kDefaultGatePenalty);
  tollbooth_cost_ = pt.get<float>("toll_booth_cost", kDefaultTollBoothCost);
  tollbooth_penalty_ = pt.get<float>("toll_booth_penalty",
                                     kDefaultTollBoothPenalty);
  alley_penalty_ = pt.get<float>("alley_penalty", kDefaultAlleyPenalty);
  country_crossing_cost_ = pt.get<float>("country_crossing_cost",
                                           kDefaultCountryCrossingCost);
  country_crossing_penalty_ = pt.get<float>("country_crossing_penalty",
                                           kDefaultCountryCrossingPenalty);

  // Set the cost (seconds) to enter a ferry (only apply entering since
  // a route must exit a ferry (except artificial test routes ending on
  // a ferry!)
  ferry_cost_ = pt.get<float>("ferry_cost", kDefaultFerryCost);

  // Modify ferry penalty and edge weighting based on use_ferry factor
  float use_ferry = pt.get<float>("use_ferry", 0.65f);
  if (use_ferry < 0.5f) {
    // Penalty goes from max at use_ferry = 0 to 0 at use_ferry = 0.5
    float w = 1.0f - ((0.5f - use_ferry) * 2.0f);
    ferry_penalty_ = static_cast<uint32_t>(kMaxFerryPenalty * (1.0f - w));

    // Double the cost at use_ferry == 0, progress to 1.0 at use_ferry = 0.5
    ferry_weight_ = 1.0f + w;
  } else {
    // Add a ferry weighting factor to influence cost along ferries to make
    // them more favorable if desired rather than driving. No ferry penalty.
    // Half the cost at use_ferry == 1, progress to 1.0 at use_ferry = 0.5
    ferry_penalty_ = 0.0f;
    ferry_weight_  = 1.0f - (use_ferry - 0.5f);
  }

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

// Check if access is allowed on the specified edge.
bool AutoCost::Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid) const {
  // TODO - obtain and check the access restrictions.

  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes in case the origin is inside
  // a not thru region and a heading selected an edge entering the
  // region.
  if (!(edge->forwardaccess() & kAutoAccess) ||
      (pred.opp_local_idx() == edge->localedgeidx() && !pred.deadend()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
       edge->surface() == Surface::kImpassable) {
    return false;
  }
  return true;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool AutoCost::AllowedReverse(const baldr::DirectedEdge* edge,
               const EdgeLabel& pred,
               const baldr::DirectedEdge* opp_edge,
               const baldr::GraphTile*& tile,
               const baldr::GraphId& edgeid) const {
  // TODO - obtain and check the access restrictions.

  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(opp_edge->forwardaccess() & kAutoAccess) ||
       (pred.opp_local_idx() == edge->localedgeidx() && !pred.deadend()) ||
       (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
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
  float factor = (edge->use() == Use::kFerry) ?
        ferry_weight_ : density_factor_[density];
  float sec = (edge->length() * speedfactor_[edge->speed()]);
  return Cost(sec * factor, sec);
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
  if (node->type() == NodeType::kBorderControl) {
    seconds += country_crossing_cost_;
    penalty += country_crossing_penalty_;
  } else if (node->type() == NodeType::kGate) {
    seconds += gate_cost_;
    penalty += gate_penalty_;
  }
  if (node->type() == NodeType::kTollBooth ||
     (!pred.toll() && edge->toll())) {
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
  if (pred.use() != Use::kFerry && edge->use() == Use::kFerry) {
    seconds += ferry_cost_;
    penalty += ferry_penalty_;
  }
  if (!node->name_consistency(idx, edge->localedgeidx())) {
    penalty += maneuver_penalty_;
  }

  // Transition time = densityfactor * stopimpact * turncost
  if (edge->stopimpact(idx) > 0) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (edge->drive_on_right()) ?
          kRightSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))] :
          kLeftSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))];
    }
    seconds += trans_density_factor_[node->density()] *
               edge->stopimpact(idx) * turn_cost;
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
  if (node->type() == NodeType::kBorderControl) {
    seconds += country_crossing_cost_;
    penalty += country_crossing_penalty_;
  } else if (node->type() == NodeType::kGate) {
    seconds += gate_cost_;
    penalty += gate_penalty_;
  }
  if (node->type() == NodeType::kTollBooth ||
     (!pred->toll() && edge->toll())) {
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
  if (pred->use() != Use::kFerry && edge->use() == Use::kFerry) {
    seconds += ferry_cost_;
    penalty += ferry_penalty_;
  }
  if (!node->name_consistency(idx, edge->localedgeidx())) {
    penalty += maneuver_penalty_;
  }

  // Transition time = densityfactor * stopimpact * turncost
  if (edge->stopimpact(idx) > 0) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (edge->drive_on_right()) ?
          kRightSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))] :
          kLeftSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))];
    }
    seconds += trans_density_factor_[node->density()] *
               edge->stopimpact(idx) * turn_cost;
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
  float factor = (edge->use() == Use::kFerry) ? ferry_weight_ : 1.0f;
  return Cost(edge->length() * adjspeedfactor_[edge->speed()] * factor,
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
   * @param  edge     Pointer to a directed edge.
   * @param  pred     Predecessor edge information.
   * @param  tile     current tile
   * @param  edgeid   edgeid that we care about
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid) const;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges are
   * provided.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           current tile
   * @param  edgeid         edgeid that we care about
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                 const EdgeLabel& pred,
                 const baldr::DirectedEdge* opp_edge,
                 const baldr::GraphTile*& tile,
                 const baldr::GraphId& edgeid) const;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present.
   * @param  edge  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const;

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. Function/functor is also used to filter
   * edges not usable / inaccessible by bus.
   */
  virtual const EdgeFilter GetEdgeFilter() const {
    // Throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge) {
      if (edge->trans_up() || edge->trans_down() ||
         !(edge->forwardaccess() & kBusAccess))
        return 0.0f;
      else {
        // TODO - use classification/use to alter the factor
        return 1.0f;
      }
    };
  }

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each node's attribution
   * @return Function/functor to be used in filtering out nodes
   */
  virtual const NodeFilter GetNodeFilter() const {
    //throw back a lambda that checks the access for this type of costing
    return [](const baldr::NodeInfo* node){
      return !(node->access() & kBusAccess);
    };
  }
};


// Constructor
BusCost::BusCost(const boost::property_tree::ptree& pt)
    : AutoCost(pt) {
}

// Destructor
BusCost::~BusCost() {
}

// Check if access is allowed on the specified edge.
bool BusCost::Allowed(const baldr::DirectedEdge* edge,
                      const EdgeLabel& pred,
                      const baldr::GraphTile*& tile,
                      const baldr::GraphId& edgeid) const {
  // TODO - obtain and check the access restrictions.

  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(edge->forwardaccess() & kBusAccess) ||
      (pred.opp_local_idx() == edge->localedgeidx() && !pred.deadend()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
       edge->surface() == Surface::kImpassable) {
    return false;
  }
  return true;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool BusCost::AllowedReverse(const baldr::DirectedEdge* edge,
                             const EdgeLabel& pred,
                             const baldr::DirectedEdge* opp_edge,
                             const baldr::GraphTile*& tile,
                             const baldr::GraphId& edgeid) const {
  // TODO - obtain and check the access restrictions.

  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(opp_edge->forwardaccess() & kBusAccess) ||
       (pred.opp_local_idx() == edge->localedgeidx() && !pred.deadend()) ||
       (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
        opp_edge->surface() == Surface::kImpassable) {
    return false;
  }
  return true;
}

// Check if access is allowed at the specified node.
bool BusCost::Allowed(const baldr::NodeInfo* node) const  {
  return (node->access() & kBusAccess);
}

cost_ptr_t CreateBusCost(const boost::property_tree::ptree& config) {
  return std::make_shared<BusCost>(config);
}


}
}
