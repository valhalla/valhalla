#include "sif/motorscootercost.h"
#include "sif/costconstants.h"

#include <iostream>
#include "midgard/constants.h"
#include "baldr/directededge.h"
#include "baldr/nodeinfo.h"
#include "baldr/accessrestriction.h"
#include "midgard/util.h"

#ifdef INLINE_TEST
#include "test/test.h"
#include <random>
#include <boost/property_tree/json_parser.hpp>
#endif

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {

constexpr float kDefaultManeuverPenalty         = 5.0f;   // Seconds
constexpr float kDefaultAlleyPenalty            = 5.0f;   // Seconds
constexpr float kDefaultGateCost                = 30.0f;  // Seconds
constexpr float kDefaultGatePenalty             = 300.0f; // Seconds
constexpr float kDefaultFerryCost               = 300.0f; // Seconds
constexpr float kDefaultCountryCrossingCost     = 600.0f; // Seconds
constexpr float kDefaultCountryCrossingPenalty  = 0.0f;   // Seconds
constexpr float kDefaultUseFerry                = 0.5f;   // Factor between 0 and 1

constexpr float kDefaultUseHills                = 0.5f;   // Factor between 0 and 1
constexpr float kDefaultUsePrimary              = 0.5f;   // Factor between 0 and 1
constexpr uint32_t kDefaultTopSpeed             = 45;     // Kilometers per hour

constexpr Surface kMinimumScooterSurface = Surface::kDirt;

// Maximum ferry penalty (when use_ferry == 0). Can't make this too large
// since a ferry is sometimes required to complete a route.
constexpr float kMaxFerryPenalty = 6.0f * kSecPerHour; // 6 hours

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

// Maximum amount of seconds that will be allowed to be passed in to influence paths
// This can't be too high because sometimes a certain kind of path is required to be taken
constexpr float kMaxSeconds = 12.0f * kSecPerHour; // 12 hours

// Valid ranges and defaults
constexpr ranged_default_t<float> kManeuverPenaltyRange{0, kDefaultManeuverPenalty, kMaxSeconds};
constexpr ranged_default_t<float> kAlleyPenaltyRange{0, kDefaultAlleyPenalty, kMaxSeconds};
constexpr ranged_default_t<float> kGateCostRange{0, kDefaultGateCost, kMaxSeconds};
constexpr ranged_default_t<float> kGatePenaltyRange{0, kDefaultGatePenalty, kMaxSeconds};
constexpr ranged_default_t<float> kFerryCostRange{0, kDefaultFerryCost, kMaxSeconds};
constexpr ranged_default_t<float> kCountryCrossingCostRange{0, kDefaultCountryCrossingCost, kMaxSeconds};
constexpr ranged_default_t<float> kCountryCrossingPenaltyRange{0, kDefaultCountryCrossingPenalty, kMaxSeconds};
constexpr ranged_default_t<float> kUseFerryRange{0, kDefaultUseFerry, 1.0f};
constexpr ranged_default_t<float> kUseHillsRange{0, kDefaultUseHills, 1.0f};
constexpr ranged_default_t<float> kUsePrimaryRange{0, kDefaultUsePrimary, 1.0f};
constexpr ranged_default_t<uint32_t> kTopSpeedRange{0, kDefaultTopSpeed, kMaxSpeedKph};

// Additional penalty to avoid destination only
constexpr float kDestinationOnlyFactor = 0.2f;

// Weighting factor based on road class. These apply penalties to higher class
// roads. These penalties are modulated by the road factor - further
// avoiding higher class roads for those with low propensity for using
// primary roads.
constexpr float kRoadClassFactor[] = {
    1.0f,   // Motorway
    0.5f,   // Trunk
    0.2f,   // Primary
    0.1f,   // Secondary
    0.05f,  // Tertiary
    0.05f,  // Unclassified
    0.0f,   // Residential
    0.5f    // Service, other
};

constexpr uint32_t kMaxGradeFactor = 15;

// Avoid hills "strength". How much do we want to avoid a hill. Combines
// with the usehills factor (1.0 - usehills = avoidhills factor) to create
// a weighting penalty per weighted grade factor. This indicates how strongly
// edges with the specified grade are weighted. Note that speed also is
// influenced by grade, so these weights help further avoid hills.
constexpr float kAvoidHillsStrength[] = {
    1.0f,      // -10%  - Very steep downhill
    0.8f,      // -8%
    0.5f,      // -6.5%
    0.2f,      // -5%   - Moderately steep downhill
    0.1f,      // -3%
    0.0f,      // -1.5%
    0.05f,     // 0%    - Flat
    0.1f,      // 1.5%
    0.3f,      // 3%
    0.8f,      // 5%
    2.0f,      // 6.5%
    3.0f,      // 8%    - Moderately steep uphill
    4.5f,      // 10%
    6.0f,      // 11.5%
    8.0f,     // 13%
    10.0f      // 15%   - Very steep uphill
};

constexpr float kGradeBasedSpeedFactor[] = {
  1.25f,      // -10%  - 45
  1.2f,     // -8%   - 40.5
  1.15f,      // -6.5% - 36
  1.1f,      // -5%   - 30.6
  1.05f,      // -3%   - 25
  1.0f,      // -1.5% - 21.6
  1.0f,      // 0%    - 18
  1.0f,     // 1.5%  - 17
  0.95f,     // 3%    - 15
  0.75f,     // 5%    - 13.5
  0.6f,     // 6.5%  - 12
  0.5f,     // 8%    - 10
  0.45f,      // 10%   - 9
  0.4f,     // 11.5% - 8
  0.35f,      // 13%   - 7
  0.25f       // 15%   - 5.5
};

constexpr float kSurfaceSpeedFactors[] =
        { 1.0f, 1.0f, 0.9f, 0.6f, 0.1f, 0.0f, 0.0f, 0.0f };

}

/**
 * Derived class providing dynamic edge costing for "direct" auto routes. This
 * is a route that is generally shortest time but uses route hierarchies that
 * can result in slightly longer routes that avoid shortcuts on residential
 * roads.
 */
class MotorScooterCost : public DynamicCost {
 public:
  /**
   * Construct motor_scooter costing. Pass in configuration using property tree.
   * @param  config  Property tree with configuration/options.
   */
  MotorScooterCost(const boost::property_tree::ptree& config);

  virtual ~MotorScooterCost();

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
   * @param  tile           Tile for the opposing edge (for looking
   *                        up restrictions).
   * @param  opp_edgeid     Opposing edge Id
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                 const EdgeLabel& pred,
                 const baldr::DirectedEdge* opp_edge,
                 const baldr::GraphTile*& tile,
                 const baldr::GraphId& opp_edgeid) const;

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
   * @param  pred  the opposing current edge in the reverse tree.
   * @param  edge  the opposing predecessor in the reverse tree
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(
      const uint32_t idx, const baldr::NodeInfo* node,
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
   * edges not usable / inaccessible by automobile.
   */
  virtual const EdgeFilter GetEdgeFilter() const {
    // Throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge) {
      if (edge->IsTransition() || edge->is_shortcut() ||
         !(edge->forwardaccess() & kMopedAccess) ||
         edge->surface() > kMinimumScooterSurface)
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
      return !(node->access() & kMopedAccess);
    };
  }

  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes
 public:
  float speedfactor_[kMaxSpeedKph + 1];
  float density_factor_[16];        // Density factor
  float maneuver_penalty_;          // Penalty (seconds) when inconsistent names
  float gate_cost_;                 // Cost (seconds) to go through gate
  float gate_penalty_;              // Penalty (seconds) to go through gate
  float ferry_cost_;                // Cost (seconds) to enter a ferry
  float ferry_penalty_;             // Penalty (seconds) to enter a ferry
  float ferry_factor_;              // Weighting to apply to ferry edges
  float alley_penalty_;             // Penalty (seconds) to use a alley
  float country_crossing_cost_;     // Cost (seconds) to go across a country border
  float country_crossing_penalty_;  // Penalty (seconds) to go across a country border
  float use_ferry_;

  // Density factor used in edge transition costing
  std::vector<float> trans_density_factor_;

  uint32_t top_speed_;  // Top speed the motorized scooter can go. Used to avoid roads
                        // with higher speeds than it

  float use_hills_;     // Scale from 0 (avoid hills) to 1 (don't avoid hills)
  float use_primary_;   // Scale from 0 (avoid primary roads) to 1 (don't avoid primary roads)
  float road_factor_;   // Road factor based on use_primary_

  // Elevation/grade penalty (weighting applied based on the edge's weighted
  // grade (relative value from 0-15)
  float grade_penalty_[16];
};


// Constructor
MotorScooterCost::MotorScooterCost(const boost::property_tree::ptree& pt)
    : DynamicCost(pt, TravelMode::kDrive),
      trans_density_factor_{ 1.0f, 1.0f, 1.0f, 1.0f,
                             1.0f, 1.1f, 1.2f, 1.3f,
                             1.4f, 1.6f, 1.9f, 2.2f,
                             2.5f, 2.8f, 3.1f, 3.5f } {
  maneuver_penalty_ = kManeuverPenaltyRange(
    pt.get<float>("maneuver_penalty", kDefaultManeuverPenalty)
  );
  gate_cost_ = kGateCostRange(
    pt.get<float>("gate_cost", kDefaultGateCost)
  );
  gate_penalty_ = kGatePenaltyRange(
    pt.get<float>("gate_penalty", kDefaultGatePenalty)
  );
  alley_penalty_ = kAlleyPenaltyRange(
    pt.get<float>("alley_penalty", kDefaultAlleyPenalty)
  );
  country_crossing_cost_ = kCountryCrossingCostRange(
    pt.get<float>("country_crossing_cost", kDefaultCountryCrossingCost)
  );
  country_crossing_penalty_ = kCountryCrossingPenaltyRange(
    pt.get<float>("country_crossing_penalty", kDefaultCountryCrossingPenalty)
  );

  // Set the cost (seconds) to enter a ferry (only apply entering since
  // a route must exit a ferry (except artificial test routes ending on
  // a ferry!)
  ferry_cost_ = kFerryCostRange(
    pt.get<float>("ferry_cost", kDefaultFerryCost)
  );

  // Modify ferry penalty and edge weighting based on use_ferry factor
  use_ferry_ = kUseFerryRange(
    pt.get<float>("use_ferry", kDefaultUseFerry)
  );
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
    ferry_factor_  = 1.5f - use_ferry_;
  }

  // Create speed cost table
  speedfactor_[0] = kSecPerHour;  // TODO - what to make speed=0?
  for (uint32_t s = 1; s <= kMaxSpeedKph; s++) {
    speedfactor_[s] = (kSecPerHour * 0.001f) / static_cast<float>(s);
  }

  // Set density factors - used to penalize edges in dense, urban areas
  for (uint32_t d = 0; d < 16; d++) {
    density_factor_[d] = 0.85f + (d * 0.018f);
  }

  top_speed_ = kTopSpeedRange (
    pt.get<float>("top_speed", kDefaultTopSpeed)
  );

  use_hills_ = kUseHillsRange (
      pt.get<float>("use_hills", kDefaultUseHills)
  );

  float avoid_hills = (1.0f - use_hills_);
  for (uint32_t i = 0; i <= kMaxGradeFactor; ++i) {
    grade_penalty_[i] = avoid_hills * kAvoidHillsStrength[i];
  }

  use_primary_ = kUsePrimaryRange (
      pt.get<float>("use_primary", kDefaultUsePrimary));

  // Set the road classification factor. use_roads factors above 0.5 start to
  // reduce the weight difference between road classes while factors below 0.5
  // start to increase the differences.
  road_factor_ = (use_primary_ >= 0.5f) ?
                 1.5f - use_primary_ :
                 3.0f - use_primary_ * 5.0f;

}

// Destructor
MotorScooterCost::~MotorScooterCost() {
}

// Does the costing method allow multiple passes (with relaxed hierarchy
// limits).
bool MotorScooterCost::AllowMultiPass() const {
  return true;
}

// Get the access mode for motor scooter
uint32_t MotorScooterCost::access_mode() const {
  return kMopedAccess;
}

// Check if access is allowed on the specified edge.
bool MotorScooterCost::Allowed(const baldr::DirectedEdge* edge,
                     const EdgeLabel& pred,
                     const baldr::GraphTile*& tile,
                     const baldr::GraphId& edgeid) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(edge->forwardaccess() & kMopedAccess) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
       IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly())) {
    return false;
  }
  return edge->surface() <= kMinimumScooterSurface;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool MotorScooterCost::AllowedReverse(const baldr::DirectedEdge* edge,
                             const EdgeLabel& pred,
                             const baldr::DirectedEdge* opp_edge,
                             const baldr::GraphTile*& tile,
                             const baldr::GraphId& opp_edgeid) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(opp_edge->forwardaccess() & kMopedAccess) ||
       (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
       (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
        IsUserAvoidEdge(opp_edgeid) ||
       (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly())) {
    return false;
  }
  return opp_edge->surface() <= kMinimumScooterSurface;
}

// Check if access is allowed at the specified node.
bool MotorScooterCost::Allowed(const baldr::NodeInfo* node) const  {
  return (node->access() & kMopedAccess);
}

Cost MotorScooterCost::EdgeCost(const baldr::DirectedEdge* edge) const {

  if (edge->use() == Use::kFerry) {
    float sec = (edge->length() * speedfactor_[edge->speed()]);
    return {sec * ferry_factor_, sec };
  }

  uint32_t scooter_speed = (std::min(top_speed_, edge->speed ()) *
      kSurfaceSpeedFactors[static_cast<uint32_t>(edge->surface())] *
      kGradeBasedSpeedFactor[static_cast<uint32_t>(edge->weighted_grade())]);

  float speed_penalty = (edge->speed() > top_speed_) ? (edge->speed() - top_speed_) * 0.05f : 0.0f;
  float factor = 1.0f + (density_factor_[edge->density()] - 0.85f) +
      (road_factor_ * kRoadClassFactor[static_cast<uint32_t>(edge->classification())]) +
      grade_penalty_[static_cast<uint32_t>(edge->weighted_grade())] +
      speed_penalty;

  if (edge->destonly())
  {
    factor += kDestinationOnlyFactor;
  }

  float sec = (edge->length() * speedfactor_[scooter_speed]);
  return {sec * factor, sec};
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost MotorScooterCost::TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred) const {
  // Accumulate cost and penalty
  float seconds = 0.0f;
  float penalty = 0.0f;

  // Special cases with both time and penalty: country crossing
  // and gate
  if (node->type() == NodeType::kBorderControl) {
    seconds += country_crossing_cost_;
    penalty += country_crossing_penalty_;
  } else if (node->type() == NodeType::kGate) {
    seconds += gate_cost_;
    penalty += gate_penalty_;
  }
  // Additional penalties without any time cost
  uint32_t idx = pred.opp_local_idx();
  if (pred.use() != Use::kAlley && edge->use() == Use::kAlley) {
    penalty += alley_penalty_;
  }
  if (pred.use() != Use::kFerry && edge->use() == Use::kFerry) {
    seconds += ferry_cost_;
    penalty += ferry_penalty_;
  }
  // Ignore name inconsistency when entering a link to avoid double penalizing.
  if (!edge->link() && !node->name_consistency(idx, edge->localedgeidx())) {
    // Slight maneuver penalty
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
Cost MotorScooterCost::TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* pred,
                                     const baldr::DirectedEdge* edge) const {
  // Accumulate cost and penalty
  float seconds = 0.0f;
  float penalty = 0.0f;

  // Special cases with both time and penalty: country crossing
  // andgate
  if (node->type() == NodeType::kBorderControl) {
    seconds += country_crossing_cost_;
    penalty += country_crossing_penalty_;
  } else if (node->type() == NodeType::kGate) {
    seconds += gate_cost_;
    penalty += gate_penalty_;
  }

  // Additional penalties without any time cost
  if (pred->use() != Use::kAlley && edge->use() == Use::kAlley) {
    penalty += alley_penalty_;
  }
  if (pred->use() != Use::kFerry && edge->use() == Use::kFerry) {
    seconds += ferry_cost_;
    penalty += ferry_penalty_;
  }
  // Ignore name inconsistency when entering a link to avoid double penalizing.
  if (!edge->link() && !node->name_consistency(idx, edge->localedgeidx())) {
    // Slight maneuver penalty
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
float MotorScooterCost::AStarCostFactor() const {
  return speedfactor_[kMaxSpeedKph];
}

// Returns the current travel type.
uint8_t MotorScooterCost::travel_type() const {
  return static_cast<uint8_t>(VehicleType::kMotorScooter);
}

cost_ptr_t CreateMotorScooterCost(const boost::property_tree::ptree& config) {
  return std::make_shared<MotorScooterCost>(config);
}

}
}

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

MotorScooterCost* make_motorscootercost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({")" << property << R"(":)" << testVal << "}";
  boost::property_tree::ptree costing_ptree;
  boost::property_tree::read_json(ss, costing_ptree);
  return new MotorScooterCost(costing_ptree);
}

template <typename T>
std::uniform_real_distribution<T>* make_real_distributor_from_range (const ranged_default_t<T>& range) {
  T rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<T>(range.min - rangeLength, range.max + rangeLength);
}

template <typename T>
std::uniform_int_distribution<T>* make_int_distributor_from_range (const ranged_default_t<T>& range) {
  T rangeLength = range.max - range.min;
  return new std::uniform_int_distribution<T>(range.min - rangeLength, range.max + rangeLength);
}

void testMotorScooterCostParams() {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::default_random_engine generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> fDistributor;
  std::shared_ptr<std::uniform_int_distribution<uint32_t>> iDistributor;
  std::shared_ptr<MotorScooterCost> ctorTester;

  // maneuver_penalty_
  fDistributor.reset(make_real_distributor_from_range(kManeuverPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("maneuver_penalty", (*fDistributor)(generator)));
    if (ctorTester->maneuver_penalty_ < kManeuverPenaltyRange.min ||
        ctorTester->maneuver_penalty_ > kManeuverPenaltyRange.max) {
      throw std::runtime_error ("maneuver_penalty_ is not within it's range");
    }
  }

  // gate_cost_
  fDistributor.reset(make_real_distributor_from_range(kGateCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("gate_cost", (*fDistributor)(generator)));
    if (ctorTester->gate_cost_ < kGateCostRange.min ||
        ctorTester->gate_cost_ > kGateCostRange.max) {
      throw std::runtime_error ("gate_cost_ is not within it's range");
    }
  }

  // gate_penalty_
  fDistributor.reset(make_real_distributor_from_range(kGatePenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("gate_penalty", (*fDistributor)(generator)));
    if (ctorTester->gate_penalty_ < kGatePenaltyRange.min ||
        ctorTester->gate_penalty_ > kGatePenaltyRange.max) {
      throw std::runtime_error ("gate_penalty_ is not within it's range");
    }
  }

  // alley_penalty_
  fDistributor.reset(make_real_distributor_from_range(kAlleyPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("alley_penalty", (*fDistributor)(generator)));
    if (ctorTester->alley_penalty_ < kAlleyPenaltyRange.min ||
        ctorTester->alley_penalty_ > kAlleyPenaltyRange.max) {
      throw std::runtime_error ("alley_penalty_ is not within it's range");
    }
  }

  // country_crossing_cost_
  fDistributor.reset(make_real_distributor_from_range(kCountryCrossingCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("country_crossing_cost", (*fDistributor)(generator)));
    if (ctorTester->country_crossing_cost_ < kCountryCrossingCostRange.min ||
        ctorTester->country_crossing_cost_ > kCountryCrossingCostRange.max) {
      throw std::runtime_error ("country_crossing_cost_ is not within it's range");
    }
  }

  // country_crossing_penalty_
  fDistributor.reset(make_real_distributor_from_range(kCountryCrossingPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("country_crossing_penalty", (*fDistributor)(generator)));
    if (ctorTester->country_crossing_penalty_ < kCountryCrossingPenaltyRange.min ||
        ctorTester->country_crossing_penalty_ > kCountryCrossingPenaltyRange.max) {
      throw std::runtime_error ("country_crossing_penalty_ is not within it's range");
    }
  }

  // ferry_cost_
  fDistributor.reset(make_real_distributor_from_range(kFerryCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("ferry_cost", (*fDistributor)(generator)));
    if (ctorTester->ferry_cost_ < kFerryCostRange.min ||
        ctorTester->ferry_cost_ > kFerryCostRange.max) {
      throw std::runtime_error ("ferry_cost_ is not within it's range");
    }
  }

  // use_ferry_
  fDistributor.reset(make_real_distributor_from_range(kUseFerryRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("use_ferry", (*fDistributor)(generator)));
    if (ctorTester->use_ferry_ < kUseFerryRange.min ||
        ctorTester->use_ferry_ > kUseFerryRange.max) {
      throw std::runtime_error ("use_ferry_ is not within it's range");
    }
  }

  // top_speed_
  iDistributor.reset(make_int_distributor_from_range(kTopSpeedRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("top_speed", (*iDistributor)(generator)));
    if (ctorTester->top_speed_ < kTopSpeedRange.min ||
        ctorTester->top_speed_ > kTopSpeedRange.max) {
      throw std::runtime_error ("top_speed_ is not within it's range");
    }
  }

  // use_hills_
  fDistributor.reset(make_real_distributor_from_range(kUseHillsRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("use_hills", (*fDistributor)(generator)));
    if (ctorTester->use_hills_ < kUseHillsRange.min ||
        ctorTester->use_hills_ > kUseHillsRange.max) {
      throw std::runtime_error ("use_hills_ is not within it's range");
    }
  }

  // use_primary_
  fDistributor.reset(make_real_distributor_from_range(kUsePrimaryRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("use_primary", (*fDistributor)(generator)));
    if (ctorTester->use_primary_ < kUsePrimaryRange.min ||
        ctorTester->use_primary_ > kUsePrimaryRange.max) {
      throw std::runtime_error ("use_hills_ is not within it's range");
    }
  }
}
}

int main() {
  test::suite suite("costing");

  suite.test(TEST_CASE(testMotorScooterCostParams));

  return suite.tear_down();
}

#endif

