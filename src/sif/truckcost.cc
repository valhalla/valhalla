#include "sif/truckcost.h"

#include <iostream>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
// TODO - alter any of these for trucks??
namespace {
constexpr float kDefaultManeuverPenalty         = 5.0f;   // Seconds
constexpr float kDefaultDestinationOnlyPenalty  = 600.0f; // Seconds
constexpr float kDefaultAlleyPenalty            = 30.0f;  // Seconds
constexpr float kDefaultGateCost                = 30.0f;  // Seconds
constexpr float kDefaultTollBoothCost           = 15.0f;  // Seconds
constexpr float kDefaultTollBoothPenalty        = 0.0f;   // Seconds
constexpr float kDefaultCountryCrossingCost     = 600.0f; // Seconds
constexpr float kDefaultCountryCrossingPenalty  = 0.0f;   // Seconds

// Default turn costs
// TODO - alter any of these for trucks?
constexpr float kTCStraight         = 0.5f;
constexpr float kTCSlight           = 0.75f;
constexpr float kTCFavorable        = 1.0f;
constexpr float kTCFavorableSharp   = 1.5f;
constexpr float kTCCrossing         = 2.0f;
constexpr float kTCUnfavorable      = 2.5f;
constexpr float kTCUnfavorableSharp = 3.5f;
constexpr float kTCReverse          = 5.0f;

// Default truck attributes
constexpr float kDefaultTruckWeight = 5.0f;  // TODO - units and default value
constexpr float kDefaultTruckHeight = 3.65f; // Meters (12 feet)
constexpr float kDefaultTruckWidth  = 3.0f;  // Meters (10 feet)
constexpr float kDefaultTruckLength = 21.5f; // Meters (70 feet)

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = { kTCStraight, kTCSlight,
      kTCFavorable, kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
      kTCUnfavorable, kTCSlight };
constexpr float kLeftSideTurnCosts[]  = { kTCStraight, kTCSlight,
      kTCUnfavorable, kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
      kTCFavorable, kTCSlight };
}

/**
 * Derived class providing dynamic edge costing for truck routes.
 */
class TruckCost : public DynamicCost {
 public:
  /**
   * Construct truck costing. Pass in configuration using property tree.
   * @param  config  Property tree with configuration/options.
   */
  TruckCost(const boost::property_tree::ptree& config);

  virtual ~TruckCost();

  /**
   * Does the costing allow hierarchy transitions. Truck costing will allow
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
   * @return Returns true if access is allowed, false if not.
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
      return edge->trans_up() || edge->trans_down() ||
             !(edge->forwardaccess() & kTruckAccess);
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

  // Vehicle attributes (used for special restrictions and costing)
  bool hazmat_;         // Carrying hazardous materials
  float weight_;        // Vehicle weight (units = TODO)
  float axleload_;      // Axle load - how does this differ from weight?
  float height_;        // Vehicle height in meters
  float width_;         // Vehicle width in meters
  float length_;        // Vehicle length in meters

  // Density factor used in edge transition costing
  std::vector<float> trans_density_factor_;
};


// Constructor
TruckCost::TruckCost(const boost::property_tree::ptree& pt)
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
  alley_penalty_ = pt.get<float>("alley_penalty", kDefaultAlleyPenalty);
  country_crossing_cost_ = pt.get<float>("country_crossing_cost",
                                           kDefaultCountryCrossingCost);
  country_crossing_penalty_ = pt.get<float>("country_crossing_penalty",
                                           kDefaultCountryCrossingPenalty);

  // Get the vehicle attributes
  hazmat_   = pt.get<bool>("hazmat", false);
  weight_   = pt.get<float>("weight", kDefaultTruckWeight);
  axleload_ = pt.get<float>("axleload", weight_);
  height_   = pt.get<float>("height", kDefaultTruckHeight);
  width_    = pt.get<float>("width", kDefaultTruckWidth);
  length_   = pt.get<float>("length", kDefaultTruckLength);

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
TruckCost::~TruckCost() {
}

// Auto costing will allow hierarchy transitions by default.
bool TruckCost::AllowTransitions() const {
  return true;
}

// Does the costing method allow multiple passes (with relaxed hierarchy
// limits).
bool TruckCost::AllowMultiPass() const {
  return true;
}

// Check if access is allowed on the specified edge. Not worth checking
// not_thru due to hierarchy transitions
bool TruckCost::Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred) const {
  // Check access, U-turn, and simple turn restriction.
  // TODO - perhaps allow U-turns at dead-end nodes?
  if (!(edge->forwardaccess() & kTruckAccess) ||
      (pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
       edge->surface() == Surface::kImpassable) {
    return false;
  }
  return true;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool TruckCost::AllowedReverse(const baldr::DirectedEdge* edge,
               const baldr::DirectedEdge* opp_edge,
               const baldr::DirectedEdge* opp_pred_edge) const {
  // Check access, U-turn, and simple turn restriction.
  // TODO - perhaps allow U-turns at dead-end nodes?
  if (!(opp_edge->forwardaccess() & kTruckAccess) ||
       (opp_pred_edge->localedgeidx() == edge->localedgeidx()) ||
       (opp_edge->restrictions() & (1 << opp_pred_edge->localedgeidx())) ||
       opp_edge->surface() == Surface::kImpassable) {
    return false;
  }
  return true;
}

// Check if access is allowed at the specified node.
bool TruckCost::Allowed(const baldr::NodeInfo* node) const  {
  return (node->access() & kTruckAccess);
}

// Get the cost to traverse the edge in seconds
Cost TruckCost::EdgeCost(const DirectedEdge* edge,
                        const uint32_t density) const {
  float sec = (edge->length() * speedfactor_[edge->speed()]);
  return Cost(sec * density_factor_[density], sec);
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost TruckCost::TransitionCost(const baldr::DirectedEdge* edge,
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
Cost TruckCost::TransitionCostReverse(const uint32_t idx,
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
float TruckCost::AStarCostFactor() const {
  return speedfactor_[kMaxSpeedKph];
}

cost_ptr_t CreateTruckCost(const boost::property_tree::ptree& config) {
  return std::make_shared<TruckCost>(config);
}

}
}
