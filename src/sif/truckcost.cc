#include "sif/truckcost.h"

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
constexpr float kDefaultAlleyPenalty            = 5.0f;   // Seconds
constexpr float kDefaultLowClassPenalty         = 30.0f;  // Seconds
constexpr float kDefaultGateCost                = 30.0f;  // Seconds
constexpr float kDefaultGatePenalty             = 300.0f; // Seconds
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

// Default truck attributes
constexpr float kDefaultTruckWeight   = 21.77f; // Metric Tons (48,000 lbs)
constexpr float kDefaultTruckAxleLoad = 9.07f;  // Metric Tons (20,000 lbs)
constexpr float kDefaultTruckHeight   = 4.11f;  // Meters (13 feet 6 inches)
constexpr float kDefaultTruckWidth    = 2.6f;   // Meters (102.36 inches)
constexpr float kDefaultTruckLength   = 21.64f; // Meters (71 feet)

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = { kTCStraight, kTCSlight,
      kTCFavorable, kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
      kTCUnfavorable, kTCSlight };
constexpr float kLeftSideTurnCosts[]  = { kTCStraight, kTCSlight,
      kTCUnfavorable, kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
      kTCFavorable, kTCSlight };

// How much to favor truck routes.
constexpr float kTruckRouteFactor = 0.85f;

// Weighting factor based on road class. These apply penalties to lower class
// roads.
constexpr float kRoadClassFactor[] = {
    0.0f,  // Motorway
    0.05f, // Trunk
    0.1f,  // Primary
    0.25f, // Secondary
    0.35f, // Tertiary
    0.5f,  // Unclassified
    0.75f, // Residential
    0.1f   // Service, other
};
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
   * Get the access mode used by this costing method.
   * @return  Returns access mode.
   */
  uint32_t access_mode() const;

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters.
   * @param  edge     Pointer to a directed edge.
   * @param  pred     Predecessor edge information.
   * @param  tile     current tile
   * @param  edgeid   edgeid that we care about
   * @return Returns true if access is allowed, false if not.
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
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const;

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. Function/functor is also used to filter
   * edges not usable / inaccessible by truck.
   */
  virtual const EdgeFilter GetEdgeFilter() const {
    // Throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge) {
      if (edge->trans_up() || edge->trans_down() || edge->is_shortcut() ||
         !(edge->forwardaccess() & kTruckAccess))
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
      return !(node->access() & kTruckAccess);
    };
  }

 protected:
  VehicleType type_;                // Vehicle type: tractor trailer
  float speedfactor_[256];
  float density_factor_[16];        // Density factor
  float maneuver_penalty_;          // Penalty (seconds) when inconsistent names
  float destination_only_penalty_;  // Penalty (seconds) using a driveway or parking aisle
  float gate_cost_;                 // Cost (seconds) to go through gate
  float gate_penalty_;              // Penalty (seconds) to go through gate
  float tollbooth_cost_;            // Cost (seconds) to go through toll booth
  float tollbooth_penalty_;         // Penalty (seconds) to go through a toll booth
  float alley_penalty_;             // Penalty (seconds) to use a alley
  float country_crossing_cost_;     // Cost (seconds) to go through toll booth
  float country_crossing_penalty_;  // Penalty (seconds) to go across a country border
  float low_class_penalty_;         // Penalty (seconds) to go to residential or service road

  // Vehicle attributes (used for special restrictions and costing)
  bool  hazmat_;        // Carrying hazardous materials
  float weight_;        // Vehicle weight in metric tons
  float axle_load_;     // Axle load weight in metric tons
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
  type_ = VehicleType::kTractorTrailer;
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

  low_class_penalty_ = pt.get<float>("low_class_penalty",
                                     kDefaultLowClassPenalty);

  // Get the vehicle attributes
  hazmat_     = pt.get<bool>("hazmat", false);
  weight_     = pt.get<float>("weight", kDefaultTruckWeight);
  axle_load_  = pt.get<float>("axle_load", kDefaultTruckAxleLoad);
  height_     = pt.get<float>("height", kDefaultTruckHeight);
  width_      = pt.get<float>("width", kDefaultTruckWidth);
  length_     = pt.get<float>("length", kDefaultTruckLength);

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

// Get the access mode used by this costing method.
uint32_t TruckCost::access_mode() const {
  return kTruckAccess;
}

// Check if access is allowed on the specified edge.
bool TruckCost::Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid) const {
  // Check access, U-turn, and simple turn restriction.
  // TODO - perhaps allow U-turns at dead-end nodes?
  if (!(edge->forwardaccess() & kTruckAccess) ||
      (pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
       edge->surface() == Surface::kImpassable) {
    return false;
  }

  if (edge->access_restriction()) {
    const std::vector<baldr::AccessRestriction>& restrictions =
        tile->GetAccessRestrictions(edgeid.id(), kTruckAccess);

    for (const auto& restriction : restrictions ) {
      // TODO:  Need to handle restictions that take place only at certain
      // times.  Currently, we only support kAllDaysOfWeek;
      switch (restriction.type()) {
        case AccessType::kHazmat:
          if (hazmat_ != restriction.value())
            return false;
          break;
        case AccessType::kMaxAxleLoad:
          if (axle_load_ > static_cast<float>(restriction.value()*0.01))
            return false;
          break;
        case AccessType::kMaxHeight:
          if (height_ > static_cast<float>(restriction.value()*0.01))
            return false;
          break;
        case AccessType::kMaxLength:
          if (length_ > static_cast<float>(restriction.value()*0.01))
            return false;
          break;
        case AccessType::kMaxWeight:
          if (weight_ > static_cast<float>(restriction.value()*0.01))
            return false;
          break;
        case AccessType::kMaxWidth:
          if (width_ > static_cast<float>(restriction.value()*0.01))
            return false;
          break;
        default:
          break;
      }
    }
  }

  return true;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool TruckCost::AllowedReverse(const baldr::DirectedEdge* edge,
               const EdgeLabel& pred,
               const baldr::DirectedEdge* opp_edge,
               const baldr::GraphTile*& tile,
               const baldr::GraphId& edgeid) const {
  // Check access, U-turn, and simple turn restriction.
  // TODO - perhaps allow U-turns at dead-end nodes?
  if (!(opp_edge->forwardaccess() & kTruckAccess) ||
       (pred.opp_local_idx() == edge->localedgeidx()) ||
       (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
       opp_edge->surface() == Surface::kImpassable) {
    return false;
  }

  if (edge->access_restriction()) {
    const std::vector<baldr::AccessRestriction>& restrictions =
          tile->GetAccessRestrictions(edgeid.id(), kTruckAccess);

    for (const auto& restriction : restrictions ) {
      // TODO:  Need to handle restictions that take place only at certain
      // times.  Currently, we only support kAllDaysOfWeek;
      if (restriction.modes() & kTruckAccess) {

        switch (restriction.type()) {
          case AccessType::kHazmat:
            if (hazmat_ != restriction.value())
              return false;
            break;
          case AccessType::kMaxAxleLoad:
            if (axle_load_ > static_cast<float>(restriction.value()*0.01))
              return false;
            break;
          case AccessType::kMaxHeight:
            if (height_ > static_cast<float>(restriction.value()*0.01))
              return false;
            break;
          case AccessType::kMaxLength:
            if (length_ > static_cast<float>(restriction.value()*0.01))
              return false;
            break;
          case AccessType::kMaxWeight:
            if (weight_ > static_cast<float>(restriction.value()*0.01))
              return false;
            break;
          case AccessType::kMaxWidth:
            if (width_ > static_cast<float>(restriction.value()*0.01))
              return false;
            break;
          default:
            break;
        }
      }
    }
  }
  return true;
}

// Check if access is allowed at the specified node.
bool TruckCost::Allowed(const baldr::NodeInfo* node) const  {
  return (node->access() & kTruckAccess);
}

// Get the cost to traverse the edge in seconds
Cost TruckCost::EdgeCost(const DirectedEdge* edge) const {

  float factor = density_factor_[edge->density()];

  if (edge->truck_route() > 0) {
    factor *= kTruckRouteFactor;
  }

  float sec = 0.0f;
  if (edge->truck_speed() > 0)
    sec = (edge->length() * speedfactor_[edge->truck_speed()]);
  else
    sec = (edge->length() * speedfactor_[edge->speed()]);

  return { sec * factor, sec };
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
  if (!node->name_consistency(idx, edge->localedgeidx())) {
    penalty += maneuver_penalty_;
  }

  if (edge->classification() == RoadClass::kResidential ||
      edge->classification() == RoadClass::kServiceOther)
    penalty += low_class_penalty_;

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
  if (!node->name_consistency(idx, edge->localedgeidx())) {
    penalty += maneuver_penalty_;
  }

  if (edge->classification() == RoadClass::kResidential ||
      edge->classification() == RoadClass::kServiceOther)
    penalty += low_class_penalty_;

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

// Returns the current travel type.
uint8_t TruckCost::travel_type() const {
  return static_cast<uint8_t>(type_);
}

cost_ptr_t CreateTruckCost(const boost::property_tree::ptree& config) {
  return std::make_shared<TruckCost>(config);
}

}
}
