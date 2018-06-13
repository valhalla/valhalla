#include "sif/autocost.h"
#include "sif/costconstants.h"

#include "baldr/accessrestriction.h"
#include "baldr/directededge.h"
#include "baldr/nodeinfo.h"
#include "midgard/constants.h"
#include "midgard/util.h"
#include <iostream>

#ifdef INLINE_TEST
#include "test/test.h"
#include <boost/property_tree/json_parser.hpp>
#include <random>
#endif

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {

constexpr float kDefaultManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultDestinationOnlyPenalty = 600.0f; // Seconds
constexpr float kDefaultAlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultGateCost = 30.0f;                // Seconds
constexpr float kDefaultGatePenalty = 300.0f;            // Seconds
constexpr float kDefaultTollBoothCost = 15.0f;           // Seconds
constexpr float kDefaultTollBoothPenalty = 0.0f;         // Seconds
constexpr float kDefaultFerryCost = 300.0f;              // Seconds
constexpr float kDefaultCountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultCountryCrossingPenalty = 0.0f;   // Seconds
constexpr float kDefaultUseFerry = 0.5f;                 // Factor between 0 and 1
constexpr float kDefaultUseHighways = 1.0f;              // Factor between 0 and 1
constexpr float kDefaultUseTolls = 0.5f;                 // Factor between 0 and 1

// Maximum ferry penalty (when use_ferry == 0). Can't make this too large
// since a ferry is sometimes required to complete a route.
constexpr float kMaxFerryPenalty = 6.0f * kSecPerHour; // 6 hours

// Default turn costs
constexpr float kTCStraight = 0.5f;
constexpr float kTCSlight = 0.75f;
constexpr float kTCFavorable = 1.0f;
constexpr float kTCFavorableSharp = 1.5f;
constexpr float kTCCrossing = 2.0f;
constexpr float kTCUnfavorable = 2.5f;
constexpr float kTCUnfavorableSharp = 3.5f;
constexpr float kTCReverse = 5.0f;

// How much to favor hov roads.
constexpr float kHOVFactor = 0.85f;

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = {kTCStraight,       kTCSlight,  kTCFavorable,
                                         kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
                                         kTCUnfavorable,    kTCSlight};
constexpr float kLeftSideTurnCosts[] = {kTCStraight,         kTCSlight,  kTCUnfavorable,
                                        kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
                                        kTCFavorable,        kTCSlight};

// Maximum amount of seconds that will be allowed to be passed in to influence paths
// This can't be too high because sometimes a certain kind of path is required to be taken
constexpr float kMaxSeconds = 12.0f * kSecPerHour; // 12 hours

// Valid ranges and defaults
constexpr ranged_default_t<float> kManeuverPenaltyRange{0, kDefaultManeuverPenalty, kMaxSeconds};
constexpr ranged_default_t<float> kDestinationOnlyPenaltyRange{0, kDefaultDestinationOnlyPenalty,
                                                               kMaxSeconds};
constexpr ranged_default_t<float> kAlleyPenaltyRange{0, kDefaultAlleyPenalty, kMaxSeconds};
constexpr ranged_default_t<float> kGateCostRange{0, kDefaultGateCost, kMaxSeconds};
constexpr ranged_default_t<float> kGatePenaltyRange{0, kDefaultGatePenalty, kMaxSeconds};
constexpr ranged_default_t<float> kTollBoothCostRange{0, kDefaultTollBoothCost, kMaxSeconds};
constexpr ranged_default_t<float> kTollBoothPenaltyRange{0, kDefaultTollBoothPenalty, kMaxSeconds};
constexpr ranged_default_t<float> kFerryCostRange{0, kDefaultFerryCost, kMaxSeconds};
constexpr ranged_default_t<float> kCountryCrossingCostRange{0, kDefaultCountryCrossingCost,
                                                            kMaxSeconds};
constexpr ranged_default_t<float> kCountryCrossingPenaltyRange{0, kDefaultCountryCrossingPenalty,
                                                               kMaxSeconds};
constexpr ranged_default_t<float> kUseFerryRange{0, kDefaultUseFerry, 1.0f};
constexpr ranged_default_t<float> kUseHighwaysRange{0, kDefaultUseHighways, 1.0f};
constexpr ranged_default_t<float> kUseTollsRange{0, kDefaultUseTolls, 1.0f};

constexpr float kHighwayFactor[] = {
    1.0f, // Motorway
    0.5f, // Trunk
    0.0f, // Primary
    0.0f, // Secondary
    0.0f, // Tertiary
    0.0f, // Unclassified
    0.0f, // Residential
    0.0f  // Service, other
};

constexpr float kSurfaceFactor[] = {
    0.0f, // kPavedSmooth
    0.0f, // kPaved
    0.0f, // kPaveRough
    0.1f, // kCompacted
    0.2f, // kDirt
    0.5f, // kGravel
    1.0f  // kPath
};

} // namespace

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

  virtual ~AutoCost() {
  }

  /**
   * Does the costing method allow multiple passes (with relaxed hierarchy
   * limits).
   * @return  Returns true if the costing model allows multiple passes.
   */
  virtual bool AllowMultiPass() const {
    return true;
  }

  /**
   * Get the access mode used by this costing method.
   * @return  Returns access mode.
   */
  uint32_t access_mode() const {
    return kAutoAccess;
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the directed edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index) const;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges (current and
   * predecessor) are provided. The access check is generally based on mode
   * of travel and the access modes allowed on the edge. However, it can be
   * extended to exclude access based on other parameters such as conditional
   * restrictions and conditional access that can depend on time and travel
   * mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the opposing edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const baldr::GraphTile*& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index) const;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present.
   * @param  node  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const {
    return (node->access() & kAutoAccess);
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge  Pointer to a directed edge.
   * @param   speed A speed for a road segment/edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge, const uint32_t speed) const;

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
   * @param  pred  the opposing current edge in the reverse tree.
   * @param  edge  the opposing predecessor in the reverse tree
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* pred,
                                     const baldr::DirectedEdge* edge) const;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const {
    return speedfactor_[kMaxSpeedKph];
  }

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const {
    return static_cast<uint8_t>(type_);
  }

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
      if (edge->IsTransition() || edge->is_shortcut() || !(edge->forwardaccess() & kAutoAccess)) {
        return 0.0f;
      } else {
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
    // throw back a lambda that checks the access for this type of costing
    return [](const baldr::NodeInfo* node) { return !(node->access() & kAutoAccess); };
  }

  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes
public:
  VehicleType type_; // Vehicle type: car (default), motorcycle, etc
  float speedfactor_[kMaxSpeedKph + 1];
  float density_factor_[16];       // Density factor
  float maneuver_penalty_;         // Penalty (seconds) when inconsistent names
  float destination_only_penalty_; // Penalty (seconds) using private road, driveway, or parking aisle
  float gate_cost_;                // Cost (seconds) to go through gate
  float gate_penalty_;             // Penalty (seconds) to go through gate
  float tollbooth_cost_;           // Cost (seconds) to go through toll booth
  float tollbooth_penalty_;        // Penalty (seconds) to go through a toll booth
  float ferry_cost_;               // Cost (seconds) to enter a ferry
  float ferry_penalty_;            // Penalty (seconds) to enter a ferry
  float ferry_factor_;             // Weighting to apply to ferry edges
  float alley_penalty_;            // Penalty (seconds) to use a alley
  float country_crossing_cost_;    // Cost (seconds) to go across a country border
  float country_crossing_penalty_; // Penalty (seconds) to go across a country border
  float use_ferry_;                // Preference to use ferries. Is a value from 0 to 1
  float use_highways_;             // Preference to use highways. Is a value from 0 to 1
  float highway_factor_;           // Factor applied when road is a motorway or trunk
  float use_tolls_;                // Preference to use tolls. Is a value from 0 to 1
  float toll_factor_;              // Factor applied when road has a toll
  float surface_factor_;           // How much the surface factors are applied.

  // Density factor used in edge transition costing
  std::vector<float> trans_density_factor_;
};

// Constructor
AutoCost::AutoCost(const boost::property_tree::ptree& pt)
    : DynamicCost(pt, TravelMode::kDrive), trans_density_factor_{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.1f,
                                                                 1.2f, 1.3f, 1.4f, 1.6f, 1.9f, 2.2f,
                                                                 2.5f, 2.8f, 3.1f, 3.5f} {
  surface_factor_ = 0.5f;
  // Get the vehicle type - enter as string and convert to enum
  std::string type = pt.get<std::string>("type", "car");
  if (type == "motorcycle") {
    type_ = VehicleType::kMotorcycle;
    surface_factor_ = 1.0f;
  } else if (type == "bus") {
    type_ = VehicleType::kBus;
  } else if (type == "tractor_trailer") {
    type_ = VehicleType::kTractorTrailer;
  } else if (type == "four_wheel_drive") {
    type_ = VehicleType::kFourWheelDrive;
    surface_factor_ = 0.0f;
  } else {
    type_ = VehicleType::kCar;
  }

  maneuver_penalty_ =
      kManeuverPenaltyRange(pt.get<float>("maneuver_penalty", kDefaultManeuverPenalty));
  destination_only_penalty_ = kDestinationOnlyPenaltyRange(
      pt.get<float>("destination_only_penalty", kDefaultDestinationOnlyPenalty));
  gate_cost_ = kGateCostRange(pt.get<float>("gate_cost", kDefaultGateCost));
  gate_penalty_ = kGatePenaltyRange(pt.get<float>("gate_penalty", kDefaultGatePenalty));
  tollbooth_cost_ = kTollBoothCostRange(pt.get<float>("toll_booth_cost", kDefaultTollBoothCost));
  tollbooth_penalty_ =
      kTollBoothPenaltyRange(pt.get<float>("toll_booth_penalty", kDefaultTollBoothPenalty));
  alley_penalty_ = kAlleyPenaltyRange(pt.get<float>("alley_penalty", kDefaultAlleyPenalty));
  country_crossing_cost_ =
      kCountryCrossingCostRange(pt.get<float>("country_crossing_cost", kDefaultCountryCrossingCost));
  country_crossing_penalty_ = kCountryCrossingPenaltyRange(
      pt.get<float>("country_crossing_penalty", kDefaultCountryCrossingPenalty));

  // Set the cost (seconds) to enter a ferry (only apply entering since
  // a route must exit a ferry (except artificial test routes ending on
  // a ferry!)
  ferry_cost_ = kFerryCostRange(pt.get<float>("ferry_cost", kDefaultFerryCost));

  // Modify ferry penalty and edge weighting based on use_ferry factor
  use_ferry_ = kUseFerryRange(pt.get<float>("use_ferry", kDefaultUseFerry));
  if (use_ferry_ < 0.5f) {
    // Penalty goes from max at use_ferry_ = 0 to 0 at use_ferry_ = 0.5
    ferry_penalty_ = static_cast<uint32_t>(kMaxFerryPenalty * (1.0f - use_ferry_ * 2.0f));

    // Cost X10 at use_ferry_ == 0, slopes downwards towards 1.0 at use_ferry_ = 0.5
    ferry_factor_ = 10.0f - use_ferry_ * 18.0f;
  } else {
    // Add a ferry weighting factor to influence cost along ferries to make
    // them more favorable if desired rather than driving. No ferry penalty.
    // Half the cost at use_ferry_ == 1, progress to 1.0 at use_ferry_ = 0.5
    ferry_penalty_ = 0.0f;
    ferry_factor_ = 1.5f - use_ferry_;
  }

  use_highways_ = kUseHighwaysRange(pt.get<float>("use_highways", kDefaultUseHighways));

  highway_factor_ = 1.0f - use_highways_;

  use_tolls_ = kUseTollsRange(pt.get<float>("use_tolls", kDefaultUseTolls));

  // Toll factor of 0 would indicate no adjustment to weighting for toll roads.
  // use_tolls = 1 would reduce weighting slightly (a negative delta) while
  // use_tolls = 0 would penalize (positive delta to weighting factor).
  toll_factor_ = use_tolls_ < 0.5f ? (2.0f - 4 * use_tolls_) : // ranges from 2 to 0
                     (0.5f - use_tolls_) * 0.03f;              // ranges from 0 to -0.15

  // Create speed cost table
  speedfactor_[0] = kSecPerHour; // TODO - what to make speed=0?
  for (uint32_t s = 1; s <= kMaxSpeedKph; s++) {
    speedfactor_[s] = (kSecPerHour * 0.001f) / static_cast<float>(s);
  }

  // Set density factors - used to penalize edges in dense, urban areas
  for (uint32_t d = 0; d < 16; d++) {
    density_factor_[d] = 0.85f + (d * 0.025f);
  }
}

// Check if access is allowed on the specified edge.
bool AutoCost::Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes in case the origin is inside
  // a not thru region and a heading selected an edge entering the
  // region.
  if (!(edge->forwardaccess() & kAutoAccess) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
      edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly())) {
    return false;
  }

  if (edge->access_restriction()) {
    const std::vector<baldr::AccessRestriction>& restrictions =
        tile->GetAccessRestrictions(edgeid.id(), kAutoAccess);
    for (const auto& restriction : restrictions) {
      if (restriction.type() == AccessType::kTimedAllowed) {
        // allowed at this range or allowed all the time
        return (current_time && restriction.value())
                   ? IsRestricted(restriction.value(), current_time, tz_index)
                   : true;
      } else if (restriction.type() == AccessType::kTimedDenied) {
        // not allowed at this range or restricted all the time
        return (current_time && restriction.value())
                   ? !IsRestricted(restriction.value(), current_time, tz_index)
                   : false;
      }
    }
  }
  return true;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool AutoCost::AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const baldr::GraphTile*& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(opp_edge->forwardaccess() & kAutoAccess) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
      opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly())) {
    return false;
  }

  if (edge->access_restriction()) {
    const std::vector<baldr::AccessRestriction>& restrictions =
        tile->GetAccessRestrictions(opp_edgeid.id(), kAutoAccess);
    for (const auto& restriction : restrictions) {
      if (restriction.type() == AccessType::kTimedAllowed) {
        // allowed at this range or allowed all the time
        return (current_time && restriction.value())
                   ? IsRestricted(restriction.value(), current_time, tz_index)
                   : true;
      } else if (restriction.type() == AccessType::kTimedDenied) {
        // not allowed at this range or restricted all the time
        return (current_time && restriction.value())
                   ? !IsRestricted(restriction.value(), current_time, tz_index)
                   : false;
      }
    }
  }
  return true;
}

// Get the cost to traverse the edge in seconds
Cost AutoCost::EdgeCost(const DirectedEdge* edge, const uint32_t speed) const {
  float factor = (edge->use() == Use::kFerry) ? ferry_factor_ : density_factor_[edge->density()];

  factor += highway_factor_ * kHighwayFactor[static_cast<uint32_t>(edge->classification())] +
            surface_factor_ * kSurfaceFactor[static_cast<uint32_t>(edge->surface())];
  if (edge->toll()) {
    factor += toll_factor_;
  }

  float sec = (edge->length() * speedfactor_[speed]);
  return Cost(sec * factor, sec);
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost AutoCost::TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred) const {
  // Accumulate cost and penalty
  float seconds = 0.0f;
  float penalty = 0.0f;

  // Special cases with both time and penalty: country crossing, ferry,
  // gate, toll booth
  if (node->type() == NodeType::kBorderControl) {
    seconds += country_crossing_cost_;
    penalty += country_crossing_penalty_;
  } else if (node->type() == NodeType::kGate) {
    seconds += gate_cost_;
    penalty += gate_penalty_;
  }
  if (node->type() == NodeType::kTollBooth || (!pred.toll() && edge->toll())) {
    seconds += tollbooth_cost_;
    penalty += tollbooth_penalty_;
  }
  if (edge->use() == Use::kFerry && pred.use() != Use::kFerry) {
    seconds += ferry_cost_;
    penalty += ferry_penalty_;
  }

  // Additional penalties without any time cost
  uint32_t idx = pred.opp_local_idx();
  if (edge->destonly() && !pred.destonly()) {
    penalty += destination_only_penalty_;
  }
  if (edge->use() == Use::kAlley && pred.use() != Use::kAlley) {
    penalty += alley_penalty_;
  }

  // Maneuver penalty, ignore when entering a link to avoid double penalizing
  if (!edge->link() && !node->name_consistency(idx, edge->localedgeidx())) {
    penalty += maneuver_penalty_;
  }

  // Transition time = densityfactor * stopimpact * turncost
  if (edge->stopimpact(idx) > 0) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (edge->drive_on_right())
                      ? kRightSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))]
                      : kLeftSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))];
    }
    seconds += trans_density_factor_[node->density()] * edge->stopimpact(idx) * turn_cost;
  }

  // Return cost (time and penalty)
  return {seconds + penalty, seconds};
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

  // Special cases with both time and penalty: country crossing, ferry,
  // gate, toll booth
  if (node->type() == NodeType::kBorderControl) {
    seconds += country_crossing_cost_;
    penalty += country_crossing_penalty_;
  } else if (node->type() == NodeType::kGate) {
    seconds += gate_cost_;
    penalty += gate_penalty_;
  }
  if (node->type() == NodeType::kTollBooth || (!pred->toll() && edge->toll())) {
    seconds += tollbooth_cost_;
    penalty += tollbooth_penalty_;
  }
  if (edge->use() == Use::kFerry && pred->use() != Use::kFerry) {
    seconds += ferry_cost_;
    penalty += ferry_penalty_;
  }

  // Additional penalties without any time cost
  if (edge->destonly() && !pred->destonly()) {
    penalty += destination_only_penalty_;
  }
  if (edge->use() == Use::kAlley && pred->use() != Use::kAlley) {
    penalty += alley_penalty_;
  }

  // Maneuver penalty, ignore when entering a link to avoid double penalizing
  if (!edge->link() && !node->name_consistency(idx, edge->localedgeidx())) {
    penalty += maneuver_penalty_;
  }

  // Transition time = densityfactor * stopimpact * turncost
  if (edge->stopimpact(idx) > 0) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (edge->drive_on_right())
                      ? kRightSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))]
                      : kLeftSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))];
    }
    seconds += trans_density_factor_[node->density()] * edge->stopimpact(idx) * turn_cost;
  }

  // Return cost (time and penalty)
  return {seconds + penalty, seconds};
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
   * @param  pt  Property tree with configuration/options.
   */
  AutoShorterCost(const boost::property_tree::ptree& pt) : AutoCost(pt) {
    // Create speed cost table that reduces the impact of speed
    adjspeedfactor_[0] = kSecPerHour; // TODO - what to make speed=0?
    for (uint32_t s = 1; s <= kMaxSpeedKph; s++) {
      adjspeedfactor_[s] = (kSecPerHour * 0.001f) / sqrtf(static_cast<float>(s));
    }
  }

  // virtual destructor
  virtual ~AutoShorterCost() {
  }

  /**
   * Returns the cost to traverse the edge and an estimate of the actual time
   * (in seconds) to traverse the edge.
   * @param  edge     Pointer to a directed edge.
   * @param  speed    A speed for a road segment/edge.
   * @return  Returns the cost to traverse the edge.
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge, const uint32_t speed) const {
    float factor = (edge->use() == Use::kFerry) ? ferry_factor_ : 1.0f;
    return Cost(edge->length() * adjspeedfactor_[speed] * factor,
                edge->length() * speedfactor_[speed]);
  }

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const {
    return adjspeedfactor_[kMaxSpeedKph];
  }

protected:
  float adjspeedfactor_[kMaxSpeedKph + 1];
};

cost_ptr_t CreateAutoShorterCost(const boost::property_tree::ptree& config) {
  return std::make_shared<AutoShorterCost>(config);
}

/**
 * Derived class providing bus costing for driving.
 */
class BusCost : public AutoCost {
public:
  /**
   * Construct bus costing.
   * Pass in configuration using property tree.
   * @param  pt  Property tree with configuration/options.
   */
  BusCost(const boost::property_tree::ptree& pt) : AutoCost(pt) {
    type_ = VehicleType::kBus;
  }

  /// virtual destructor
  virtual ~BusCost() {
  }

  /**
   * Get the access mode used by this costing method.
   * @return  Returns access mode.
   */
  uint32_t access_mode() const {
    return kBusAccess;
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the directed edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index) const;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges (current and
   * predecessor) are provided. The access check is generally based on mode
   * of travel and the access modes allowed on the edge. However, it can be
   * extended to exclude access based on other parameters such as conditional
   * restrictions and conditional access that can depend on time and travel
   * mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the opposing edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const baldr::GraphTile*& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index) const;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present.
   * @param  node  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const {
    return (node->access() & kBusAccess);
  }

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
      if (edge->IsTransition() || !(edge->forwardaccess() & kBusAccess)) {
        return 0.0f;
      } else {
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
    // throw back a lambda that checks the access for this type of costing
    return [](const baldr::NodeInfo* node) { return !(node->access() & kBusAccess); };
  }
};

// Check if access is allowed on the specified edge.
bool BusCost::Allowed(const baldr::DirectedEdge* edge,
                      const EdgeLabel& pred,
                      const baldr::GraphTile*& tile,
                      const baldr::GraphId& edgeid,
                      const uint64_t current_time,
                      const uint32_t tz_index) const {
  // TODO - obtain and check the access restrictions.

  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(edge->forwardaccess() & kBusAccess) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
      edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly())) {
    return false;
  }

  if (edge->access_restriction()) {
    const std::vector<baldr::AccessRestriction>& restrictions =
        tile->GetAccessRestrictions(edgeid.id(), kBusAccess);
    for (const auto& restriction : restrictions) {
      if (restriction.type() == AccessType::kTimedAllowed) {
        // allowed at this range or allowed all the time
        return (current_time && restriction.value())
                   ? IsRestricted(restriction.value(), current_time, tz_index)
                   : true;
      } else if (restriction.type() == AccessType::kTimedDenied) {
        // not allowed at this range or restricted all the time
        return (current_time && restriction.value())
                   ? !IsRestricted(restriction.value(), current_time, tz_index)
                   : false;
      }
    }
  }
  return true;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool BusCost::AllowedReverse(const baldr::DirectedEdge* edge,
                             const EdgeLabel& pred,
                             const baldr::DirectedEdge* opp_edge,
                             const baldr::GraphTile*& tile,
                             const baldr::GraphId& opp_edgeid,
                             const uint64_t current_time,
                             const uint32_t tz_index) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(opp_edge->forwardaccess() & kBusAccess) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
      opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly())) {
    return false;
  }

  if (edge->access_restriction()) {
    const std::vector<baldr::AccessRestriction>& restrictions =
        tile->GetAccessRestrictions(opp_edgeid.id(), kBusAccess);
    for (const auto& restriction : restrictions) {
      if (restriction.type() == AccessType::kTimedAllowed) {
        // allowed at this range or allowed all the time
        return (current_time && restriction.value())
                   ? IsRestricted(restriction.value(), current_time, tz_index)
                   : true;
      } else if (restriction.type() == AccessType::kTimedDenied) {
        // not allowed at this range or restricted all the time
        return (current_time && restriction.value())
                   ? !IsRestricted(restriction.value(), current_time, tz_index)
                   : false;
      }
    }
  }

  return true;
}

cost_ptr_t CreateBusCost(const boost::property_tree::ptree& config) {
  return std::make_shared<BusCost>(config);
}

/**
 * Derived class providing an alternate costing for driving that is intended
 * to favor HOV roads.
 */
class HOVCost : public AutoCost {
public:
  /**
   * Construct hov costing.
   * Pass in configuration using property tree.
   * @param  pt  Property tree with configuration/options.
   */
  HOVCost(const boost::property_tree::ptree& pt) : AutoCost(pt) {
  }

  virtual ~HOVCost() {
  }

  /**
   * Get the access mode used by this costing method.
   * @return  Returns access mode.
   */
  uint32_t access_mode() const {
    return kHOVAccess;
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the directed edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index) const;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges (current and
   * predecessor) are provided. The access check is generally based on mode
   * of travel and the access modes allowed on the edge. However, it can be
   * extended to exclude access based on other parameters such as conditional
   * restrictions and conditional access that can depend on time and travel
   * mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the opposing edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const baldr::GraphTile*& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index) const;

  /**
   * Returns the cost to traverse the edge and an estimate of the actual time
   * (in seconds) to traverse the edge.
   * @param  edge     Pointer to a directed edge.
   * @param  speed    A speed for a road segment/edge.
   * @return  Returns the cost to traverse the edge.
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge, const uint32_t speed) const {
    float factor = (edge->use() == Use::kFerry) ? ferry_factor_ : density_factor_[edge->density()];
    if ((edge->forwardaccess() & kHOVAccess) && !(edge->forwardaccess() & kAutoAccess)) {
      factor *= kHOVFactor;
    }
    float sec = (edge->length() * speedfactor_[speed]);
    return Cost(sec * factor, sec);
  }

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present.
   * @param  node  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const {
    return (node->access() & kHOVAccess);
  }

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. Function/functor is also used to filter
   * edges not usable / inaccessible by hov.
   */
  virtual const EdgeFilter GetEdgeFilter() const {
    // Throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge) {
      if (edge->IsTransition() || !(edge->forwardaccess() & kHOVAccess)) {
        return 0.0f;
      } else {
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
    // throw back a lambda that checks the access for this type of costing
    return [](const baldr::NodeInfo* node) { return !(node->access() & kHOVAccess); };
  }
};

// Check if access is allowed on the specified edge.
bool HOVCost::Allowed(const baldr::DirectedEdge* edge,
                      const EdgeLabel& pred,
                      const baldr::GraphTile*& tile,
                      const baldr::GraphId& edgeid,
                      const uint64_t current_time,
                      const uint32_t tz_index) const {
  // TODO - obtain and check the access restrictions.

  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes in case the origin is inside
  // a not thru region and a heading selected an edge entering the
  // region.
  if (!(edge->forwardaccess() & kHOVAccess) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
      edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly())) {
    return false;
  }
  if (edge->access_restriction()) {
    const std::vector<baldr::AccessRestriction>& restrictions =
        tile->GetAccessRestrictions(edgeid.id(), kHOVAccess);
    for (const auto& restriction : restrictions) {
      if (restriction.type() == AccessType::kTimedAllowed) {
        // allowed at this range or allowed all the time
        return (current_time && restriction.value())
                   ? IsRestricted(restriction.value(), current_time, tz_index)
                   : true;
      } else if (restriction.type() == AccessType::kTimedDenied) {
        // not allowed at this range or restricted all the time
        return (current_time && restriction.value())
                   ? !IsRestricted(restriction.value(), current_time, tz_index)
                   : false;
      }
    }
  }
  return true;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool HOVCost::AllowedReverse(const baldr::DirectedEdge* edge,
                             const EdgeLabel& pred,
                             const baldr::DirectedEdge* opp_edge,
                             const baldr::GraphTile*& tile,
                             const baldr::GraphId& opp_edgeid,
                             const uint64_t current_time,
                             const uint32_t tz_index) const {
  // TODO - obtain and check the access restrictions.

  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(opp_edge->forwardaccess() & kHOVAccess) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
      opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly())) {
    return false;
  }

  if (edge->access_restriction()) {
    const std::vector<baldr::AccessRestriction>& restrictions =
        tile->GetAccessRestrictions(opp_edgeid.id(), kHOVAccess);
    for (const auto& restriction : restrictions) {
      if (restriction.type() == AccessType::kTimedAllowed) {
        // allowed at this range or allowed all the time
        return (current_time && restriction.value())
                   ? IsRestricted(restriction.value(), current_time, tz_index)
                   : true;
      } else if (restriction.type() == AccessType::kTimedDenied) {
        // not allowed at this range or restricted all the time
        return (current_time && restriction.value())
                   ? !IsRestricted(restriction.value(), current_time, tz_index)
                   : false;
      }
    }
  }
  return true;
}

cost_ptr_t CreateHOVCost(const boost::property_tree::ptree& config) {
  return std::make_shared<HOVCost>(config);
}

/**
 * Derived class providing an alternate costing for driving that is ignores
 * oneways and turn restrictions. This can be useful for map-matching traces
 * when trying data that may have incorrect restrictions or oneway information.
 */
class AutoDataFix : public AutoCost {
public:
  /**
   * Construct auto data fix costing.
   * Pass in configuration using property tree.
   * @param  pt  Property tree with configuration/options.
   */
  AutoDataFix(const boost::property_tree::ptree& pt) : AutoCost(pt) {
  }

  virtual ~AutoDataFix() {
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the directed edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index) const {
    // Check access and return false (not allowed if no auto access is allowed in either
    // direction. Also disallow simple U-turns except at dead-end nodes.
    if (!((edge->forwardaccess() & kAutoAccess) || (edge->reverseaccess() & kAutoAccess)) ||
        (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
        edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
        (!allow_destination_only_ && !pred.destonly() && edge->destonly())) {
      return false;
    }
    return true;
  }

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. Function/functor is also used to filter
   * edges not usable / inaccessible by auto in either direction.
   */
  virtual const EdgeFilter GetEdgeFilter() const {
    // Throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge) {
      // Do not allow transition edges and edges with no auto access in either direction
      if (edge->IsTransition() ||
          !((edge->forwardaccess() & kAutoAccess) || (edge->reverseaccess() & kAutoAccess))) {
        return 0.0f;
      } else {
        return 1.0f;
      }
    };
  }
};

cost_ptr_t CreateAutoDataFixCost(const boost::property_tree::ptree& config) {
  return std::make_shared<AutoDataFix>(config);
}

} // namespace sif
} // namespace valhalla

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

AutoCost* make_autocost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({")" << property << R"(":)" << testVal << "}";
  boost::property_tree::ptree costing_ptree;
  boost::property_tree::read_json(ss, costing_ptree);
  return new AutoCost(costing_ptree);
}

std::uniform_real_distribution<float>*
make_distributor_from_range(const ranged_default_t<float>& range) {
  float rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<float>(range.min - rangeLength, range.max + rangeLength);
}

void testAutoCostParams() {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::default_random_engine generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> distributor;
  std::shared_ptr<AutoCost> ctorTester;

  // maneuver_penalty_
  distributor.reset(make_distributor_from_range(kManeuverPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_autocost_from_json("maneuver_penalty", (*distributor)(generator)));
    if (ctorTester->maneuver_penalty_ < kManeuverPenaltyRange.min ||
        ctorTester->maneuver_penalty_ > kManeuverPenaltyRange.max) {
      throw std::runtime_error("maneuver_penalty_ is not within it's range");
    }
  }

  // destination_only_penalty_
  distributor.reset(make_distributor_from_range(kDestinationOnlyPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_autocost_from_json("destination_only_penalty", (*distributor)(generator)));
    if (ctorTester->destination_only_penalty_ < kDestinationOnlyPenaltyRange.min ||
        ctorTester->destination_only_penalty_ > kDestinationOnlyPenaltyRange.max) {
      throw std::runtime_error("destination_only_penalty_ is not within it's range");
    }
  }

  // alley_penalty_
  distributor.reset(make_distributor_from_range(kAlleyPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_autocost_from_json("alley_penalty", (*distributor)(generator)));
    if (ctorTester->alley_penalty_ < kAlleyPenaltyRange.min ||
        ctorTester->alley_penalty_ > kAlleyPenaltyRange.max) {
      throw std::runtime_error("alley_penalty_ is not within it's range");
    }
  }

  // gate_cost_
  distributor.reset(make_distributor_from_range(kGateCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_autocost_from_json("gate_cost", (*distributor)(generator)));
    if (ctorTester->gate_cost_ < kGateCostRange.min || ctorTester->gate_cost_ > kGateCostRange.max) {
      throw std::runtime_error("gate_cost_ is not within it's range");
    }
  }

  // gate_penalty_
  distributor.reset(make_distributor_from_range(kGatePenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_autocost_from_json("gate_penalty", (*distributor)(generator)));
    if (ctorTester->gate_penalty_ < kGatePenaltyRange.min ||
        ctorTester->gate_penalty_ > kGatePenaltyRange.max) {
      throw std::runtime_error("gate_penalty_ is not within it's range");
    }
  }

  // tollbooth_cost_
  distributor.reset(make_distributor_from_range(kTollBoothCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_autocost_from_json("toll_booth_cost", (*distributor)(generator)));
    if (ctorTester->tollbooth_cost_ < kTollBoothCostRange.min ||
        ctorTester->tollbooth_cost_ > kTollBoothCostRange.max) {
      throw std::runtime_error("tollbooth_cost_ is not within it's range");
    }
  }

  // tollbooth_penalty_
  distributor.reset(make_distributor_from_range(kTollBoothPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_autocost_from_json("toll_booth_penalty", (*distributor)(generator)));
    if (ctorTester->tollbooth_penalty_ < kTollBoothPenaltyRange.min ||
        ctorTester->tollbooth_penalty_ > kTollBoothPenaltyRange.max) {
      throw std::runtime_error("tollbooth_penalty_ is not within it's range");
    }
  }

  // ferry_cost_
  distributor.reset(make_distributor_from_range(kFerryCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_autocost_from_json("ferry_cost", (*distributor)(generator)));
    if (ctorTester->ferry_cost_ < kFerryCostRange.min ||
        ctorTester->ferry_cost_ > kFerryCostRange.max) {
      throw std::runtime_error("ferry_cost_ is not within it's range");
    }
  }

  // country_crossing_cost_
  distributor.reset(make_distributor_from_range(kCountryCrossingCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_autocost_from_json("country_crossing_cost", (*distributor)(generator)));
    if (ctorTester->country_crossing_cost_ < kCountryCrossingCostRange.min ||
        ctorTester->country_crossing_cost_ > kCountryCrossingCostRange.max) {
      throw std::runtime_error("country_crossing_cost_ is not within it's range");
    }
  }

  // country_crossing_penalty_
  distributor.reset(make_distributor_from_range(kCountryCrossingPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_autocost_from_json("country_crossing_penalty", (*distributor)(generator)));
    if (ctorTester->country_crossing_penalty_ < kCountryCrossingPenaltyRange.min ||
        ctorTester->country_crossing_penalty_ > kCountryCrossingPenaltyRange.max) {
      throw std::runtime_error("country_crossing_penalty_ is not within it's range");
    }
  }

  // use_ferry_
  distributor.reset(make_distributor_from_range(kUseFerryRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_autocost_from_json("use_ferry", (*distributor)(generator)));
    if (ctorTester->use_ferry_ < kUseFerryRange.min || ctorTester->use_ferry_ > kUseFerryRange.max) {
      throw std::runtime_error("use_ferry_ is not within it's range");
    }
  }
}
} // namespace

int main() {
  test::suite suite("costing");

  suite.test(TEST_CASE(testAutoCostParams));

  return suite.tear_down();
}

#endif
