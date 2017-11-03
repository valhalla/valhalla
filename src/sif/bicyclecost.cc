#include "sif/bicyclecost.h"
#include "sif/costconstants.h"

#include "baldr/directededge.h"
#include "baldr/nodeinfo.h"
#include "midgard/constants.h"
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
constexpr float kDefaultDrivewayPenalty         = 300.0f; // Seconds
constexpr float kDefaultAlleyPenalty            = 60.0f;  // Seconds
constexpr float kDefaultGateCost                = 30.0f;  // Seconds
constexpr float kDefaultGatePenalty             = 300.0f; // Seconds
constexpr float kDefaultFerryCost               = 300.0f; // Seconds
constexpr float kDefaultCountryCrossingCost     = 600.0f; // Seconds
constexpr float kDefaultCountryCrossingPenalty  = 0.0f;   // Seconds
constexpr float kDefaultUseRoad                 = 0.25f;  // Factor between 0 and 1
constexpr float kDefaultUseFerry                = 0.5f;   // Factor between 0 and 1
constexpr float kDefaultAvoidBadSurfaces        = 0.25f;  // Factor between 0 and 1

// Maximum ferry penalty (when use_ferry == 0). Can't make this too large
// since a ferry is sometimes required to complete a route.
constexpr float kMaxFerryPenalty = 8.0f * 3600.0f; // 8 hours

// Default turn costs - modified by the stop impact.
constexpr float kTCStraight          = 0.15f;
constexpr float kTCFavorableSlight   = 0.2f;
constexpr float kTCFavorable         = 0.3f;
constexpr float kTCFavorableSharp    = 0.5f;
constexpr float kTCCrossing          = 0.75f;
constexpr float kTCUnfavorableSlight = 0.4f;
constexpr float kTCUnfavorable       = 1.0f;
constexpr float kTCUnfavorableSharp  = 1.5f;
constexpr float kTCReverse           = 5.0f;

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = { kTCStraight, kTCFavorableSlight,
      kTCFavorable, kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
      kTCUnfavorable, kTCUnfavorableSlight };
constexpr float kLeftSideTurnCosts[]  = { kTCStraight, kTCUnfavorableSlight,
      kTCUnfavorable, kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
      kTCFavorable, kTCFavorableSlight };

// Turn stress penalties for low-stress bike.
constexpr float kTPStraight          = 0.0f;
constexpr float kTPFavorableSlight   = 0.25f;
constexpr float kTPFavorable         = 0.75f;
constexpr float kTPFavorableSharp    = 1.0f;
constexpr float kTPUnfavorableSlight = 0.75f;
constexpr float kTPUnfavorable       = 1.75f;
constexpr float kTPUnfavorableSharp  = 2.25f;
constexpr float kTPReverse           = 4.0f;

constexpr float kRightSideTurnPenalties[] = {
    kTPStraight,
    kTPFavorableSlight,
    kTPFavorable,
    kTPFavorableSharp,
    kTPReverse,
    kTPUnfavorableSharp,
    kTPUnfavorable,
    kTPUnfavorableSlight
};
constexpr float kLeftSideTurnPenalties[]  = {
    kTPStraight,
    kTPUnfavorableSlight,
    kTPUnfavorable,
    kTPUnfavorableSharp,
    kTPReverse,
    kTPFavorableSharp,
    kTPFavorable,
    kTPFavorableSlight
};

// Cost of traversing an edge with steps. Make this high but not impassible.
// Equal to about 5 minutes (penalty)
const float kBicycleStepsFactor = 8.0f;

// Default cycling speed on smooth, flat roads - based on bicycle type (KPH)
constexpr float kDefaultCyclingSpeed[] = {
    25.0f,    // Road bicycle: ~15.5 MPH
    20.0f,    // Cross bicycle: ~13 MPH
    18.0f,    // Hybrid or "city" bicycle: ~11.5 MPH
    16.0f     // Mountain bicycle: ~10 MPH
};

constexpr float kDismountSpeed = 5.1f;

// Minimum and maximum average bicycling speed (to validate input).
// Maximum is just above the fastest average speed in Tour de France time trial
constexpr float kMinCyclingSpeed = 5.0f;  // KPH
constexpr float kMaxCyclingSpeed = 60.0f; // KPH

// Speed factors based on surface types (defined for each bicycle type).
// These values determine the percentage by which speed us reduced for
// each surface type. (0 values indicate unusable surface types).
constexpr float kRoadSurfaceSpeedFactors[] =
        { 1.0f, 1.0f, 0.9f, 0.6f, 0.5f, 0.3f, 0.2f, 0.0f };
constexpr float kHybridSurfaceSpeedFactors[] =
        { 1.0f, 1.0f, 1.0f, 0.8f, 0.6f, 0.4f, 0.25f, 0.0f };
constexpr float kCrossSurfaceSpeedFactors[] =
        { 1.0f, 1.0f, 1.0f, 0.8f, 0.7f, 0.5f, 0.4f, 0.0f };
constexpr float kMountainSurfaceSpeedFactors[] =
        { 1.0f, 1.0f, 1.0f, 1.0f, 0.9f, 0.75f, 0.55f, 0.0f };

// Worst allowed surface based on bicycle type
constexpr Surface kWorstAllowedSurface[] =
        { Surface::kCompacted,            // Road bicycle
          Surface::kGravel,               // Cross
          Surface::kDirt,                 // Hybrid
          Surface::kPath };               // Mountain

constexpr float kSurfaceFactors[] =
        { 1.0f, 2.5f, 4.5f, 7.0f };

// Avoid driveways
constexpr float kDrivewayFactor = 20.0f;

// Additional penalty to avoid destination only
constexpr float kDestinationOnlyFactor = 0.1f;

// Weighting factor based on road class. These apply penalties to higher class
// roads. These penalties are modulated by the useroads factor - further
// avoiding higher class roads for those with low propensity for using roads.
constexpr float kRoadClassFactor[] = {
    1.0f,   // Motorway
    0.4f,   // Trunk
    0.2f,   // Primary
    0.1f,   // Secondary
    0.05f,  // Tertiary
    0.05f,  // Unclassified
    0.0f,   // Residential
    0.5f    // Service, other
};

// Speed adjustment factors based on weighted grade. Comments here show an
// example of speed changes based on "grade", using a base speed of 18 MPH
// on flat roads
constexpr float kGradeBasedSpeedFactor[] = {
  2.5f,      // -10%  - 45
  2.25f,     // -8%   - 40.5
  2.0f,      // -6.5% - 36
  1.7f,      // -5%   - 30.6
  1.4f,      // -3%   - 25
  1.2f,      // -1.5% - 21.6
  1.0f,      // 0%    - 18
  0.95f,     // 1.5%  - 17
  0.85f,     // 3%    - 15
  0.75f,     // 5%    - 13.5
  0.65f,     // 6.5%  - 12
  0.55f,     // 8%    - 10
  0.5f,      // 10%   - 9
  0.45f,     // 11.5% - 8
  0.4f,      // 13%   - 7
  0.3f       // 15%   - 5.5
};

// User propensity to use "hilly" roads. Ranges from a value of 0 (avoid
// hills) to 1 (take hills when they offer a more direct, less time, path).
constexpr float kDefaultUseHills = 0.25f;

// Avoid hills "strength". How much do we want to avoid a hill. Combines
// with the usehills factor (1.0 - usehills = avoidhills factor) to create
// a weighting penalty per weighted grade factor. This indicates how strongly
// edges with the specified grade are weighted. Note that speed also is
// influenced by grade, so these weights help further avoid hills.
constexpr float kAvoidHillsStrength[] = {
    2.0f,      // -10%  - Treacherous descent possible
    1.0f,      // -8%   - Steep downhill
    0.5f,      // -6.5% - Good downhill - where is the bottom?
    0.2f,      // -5%   - Picking up speed!
    0.1f,      // -3%   - Modest downhill
    0.0f,      // -1.5% - Smooth slight downhill, ride this all day!
    0.05f,     // 0%    - Flat, no avoidance
    0.1f,      // 1.5%  - These are called "false flat"
    0.3f,      // 3%    - Slight rise
    0.8f,      // 5%    - Small hill
    2.0f,      // 6.5%  - Starting to feel this...
    3.0f,      // 8%    - Moderately steep
    4.5f,      // 10%   - Getting tough
    6.5f,      // 11.5% - Tiring!
    10.0f,     // 13%   - Ooof - this hurts
    12.0f      // 15%   - Only for the strongest!
};

// Edge speed above which extra penalties apply (to avoid roads with higher
// speed traffic). This threshold is adjusted upwards with higher useroads
// factors.
constexpr uint32_t kSpeedPenaltyThreshold = 40;      // 40 KPH ~ 25 MPH

// How much to favor bicycle networks.
constexpr float kBicycleNetworkFactor = 0.95f;

// Maximum amount of seconds that will be allowed to be passed in to influence paths
// This can't be too high because sometimes a certain kind of path is required to be taken
constexpr float kMaxSeconds = 12.0f * kSecPerHour; // 12 hours

// Valid ranges and defaults
constexpr ranged_default_t<float> kManeuverPenaltyRange{0.0f, kDefaultManeuverPenalty, kMaxSeconds};
constexpr ranged_default_t<float> kDrivewayPenaltyRange{0.0f, kDefaultDrivewayPenalty, kMaxSeconds};
constexpr ranged_default_t<float> kAlleyPenaltyRange{0.0f, kDefaultAlleyPenalty, kMaxSeconds};
constexpr ranged_default_t<float> kGateCostRange{0.0f, kDefaultGateCost, kMaxSeconds};
constexpr ranged_default_t<float> kGatePenaltyRange{0.0f, kDefaultGatePenalty, kMaxSeconds};
constexpr ranged_default_t<float> kFerryCostRange{0.0f, kDefaultFerryCost, kMaxSeconds};
constexpr ranged_default_t<float> kCountryCrossingCostRange{0.0f, kDefaultCountryCrossingCost, kMaxSeconds};
constexpr ranged_default_t<float> kCountryCrossingPenaltyRange{0.0f, kDefaultCountryCrossingPenalty, kMaxSeconds};
constexpr ranged_default_t<float> kUseRoadRange{0.0f, kDefaultUseRoad, 1.0f};
constexpr ranged_default_t<float> kUseFerryRange{0.0f, kDefaultUseFerry, 1.0f};
constexpr ranged_default_t<float> kUseHillsRange{0.0f, kDefaultUseHills, 1.0f};
constexpr ranged_default_t<float> kAvoidBadSurfacesRange{0.0f, kDefaultAvoidBadSurfaces, 1.0f};
}

/**
 * Derived class providing dynamic edge costing for bicycle routes.
 */
class BicycleCost : public DynamicCost {
 public:
  /**
   * Constructor. Configuration / options for bicycle costing are provided
   * via a property tree.
   * @param  config  Property tree with configuration/options.
   */
  BicycleCost(const boost::property_tree::ptree& config);

  virtual ~BicycleCost();

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
  virtual float AStarCostFactor() const;

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const;

  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes

  float speedfactor_[kMaxSpeedKph + 1];  // Cost factors based on speed in kph
  float maneuver_penalty_;               // Penalty (seconds) when inconsistent names
  float driveway_penalty_;               // Penalty (seconds) using a driveway
  float gate_cost_;                      // Cost (seconds) to go through gate
  float gate_penalty_;                   // Penalty (seconds) to go through gate
  float alley_penalty_;                  // Penalty (seconds) to use a alley
  float ferry_cost_;                     // Cost (seconds) to exit a ferry
  float ferry_penalty_;                  // Penalty (seconds) to enter a ferry
  float ferry_factor_;                   // Weighting to apply to ferry edges
  float country_crossing_cost_;          // Cost (seconds) to go across a country border
  float country_crossing_penalty_;       // Penalty (seconds) to go across a country border
  float use_roads_;                      // Preference of using roads between 0 and 1
  float road_factor_;                    // Road factor based on use_roads_
  float use_ferry_;                      // Preference of using ferries between 0 and 1
  float use_hills_;                      // Preference of using hills between 0 and 1
  float avoid_bad_surfaces_;             // Preference of avoiding bad surfaces for the bike type

  // Density factor used in edge transition costing
  std::vector<float> trans_density_factor_;

  // Average speed (kph) on smooth, flat roads.
  float speed_;

  // Bicycle type
  BicycleType type_;

  // Minimal surface type that will be penalized for costing
  Surface minimal_surface_penalized_;

  Surface worst_allowed_surface_;

  // Surface speed factors (based on road surface type).
  const float* surface_speed_factor_;

  // Speed penalty factor. Penalties apply above a threshold
  // (based on the use_roads factor)
  float speedpenalty_[kMaxSpeedKph + 1];
  uint32_t speed_penalty_threshold_;
  
  // Elevation/grade penalty (weighting applied based on the edge's weighted
  // grade (relative value from 0-15)
  float grade_penalty[16];

protected:

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. Function/functor is also used to filter
   * edges not usable / inaccessible by bicycle.
   */
  virtual const EdgeFilter GetEdgeFilter() const {
    // Throw back a lambda that checks the access for this type of costing
    Surface s = worst_allowed_surface_;
    return [s](const baldr::DirectedEdge* edge) {
      if ( edge->IsTransition() || edge->is_shortcut() ||
          !(edge->forwardaccess() & kBicycleAccess) ||
           edge->use() == Use::kSteps ||
           edge->surface() > s) {
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
   * @return Function to be used in filtering out nodes
   */
  virtual const NodeFilter GetNodeFilter() const {
    //throw back a lambda that checks the access for this type of costing
    return [](const baldr::NodeInfo* node) {
      return !(node->access() & kBicycleAccess);
    };
  }

};

// Bicycle route costs are distance based with some favor/avoid based on
// attribution.

// Constructor
BicycleCost::BicycleCost(const boost::property_tree::ptree& pt)
    : DynamicCost(pt, TravelMode::kBicycle),
      trans_density_factor_{ 1.0f,  1.0f, 1.0f,  1.0f,
                             1.0f,  1.0f, 1.0f,  1.0f,
                             1.05f, 1.1f, 1.15f, 1.2f,
                             1.25f, 1.3f, 1.4f,  1.5f } {
  // Set hierarchy to allow unlimited transitions
  for (auto& h : hierarchy_limits_) {
    h.max_up_transitions = kUnlimitedTransitions;
  }

  // Transition penalties (similar to auto)
  maneuver_penalty_ = kManeuverPenaltyRange(
    pt.get<float>("maneuver_penalty", kDefaultManeuverPenalty)
  );
  driveway_penalty_ = kDrivewayPenaltyRange(
    pt.get<float>("driveway", kDefaultDrivewayPenalty)
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

  // Get the bicycle type - enter as string and convert to enum
  std::string bicycle_type = pt.get("bicycle_type", "Hybrid");
  if (bicycle_type == "Cross") {
    type_ = BicycleType::kCross;
  } else if (bicycle_type == "Road") {
    type_ = BicycleType::kRoad;
  } else if (bicycle_type == "Mountain") {
    type_ = BicycleType::kMountain;
  } else {
    type_ = BicycleType::kHybrid;
  }

  // Get default speed from the config. This is the average speed on smooth,
  // flat roads. If not present or outside the valid range use a default speed
  // based on the bicycle type.
  uint32_t t = static_cast<uint32_t>(type_);
  ranged_default_t<float> kCycleSpeedRange {
    kMinCyclingSpeed,
    kDefaultCyclingSpeed[t],
    kMaxCyclingSpeed
  };

  speed_ = kCycleSpeedRange(
    pt.get<float>("cycling_speed", kDefaultCyclingSpeed[t])
  );

  avoid_bad_surfaces_ = kAvoidBadSurfacesRange (
    pt.get<float>("avoid_bad_surfaces", kDefaultAvoidBadSurfaces)
  );

  minimal_surface_penalized_ = kWorstAllowedSurface[static_cast<uint32_t> (type_)];

  worst_allowed_surface_ = avoid_bad_surfaces_ == 1.0f ?
          minimal_surface_penalized_ :
          Surface::kPath;

  // Set the surface speed factors for the bicycle type.
  if (type_ == BicycleType::kRoad) {
    surface_speed_factor_ = kRoadSurfaceSpeedFactors;
  } else if (type_ == BicycleType::kHybrid) {
    surface_speed_factor_ = kHybridSurfaceSpeedFactors;
  } else if (type_ == BicycleType::kCross) {
    surface_speed_factor_ = kCrossSurfaceSpeedFactors;
  } else {
    surface_speed_factor_ = kMountainSurfaceSpeedFactors;
  }

  // Willingness to use roads. Make sure this is within range [0, 1].
  use_roads_ = kUseRoadRange(
    pt.get<float>("use_roads", kDefaultUseRoad)
  );

  // Set the road classification factor. use_roads factors above 0.5 start to
  // reduce the weight difference between road classes while factors below 0.5
  // start to increase the differences.
  road_factor_ = (use_roads_ >= 0.5f) ?
                 1.5f - use_roads_ :
                 2.0f - use_roads_ * 2.0f;

  // Set the cost (seconds) to enter a ferry (only apply entering since
  // a route must exit a ferry (except artificial test routes ending on
  // a ferry!)
  ferry_cost_ = kFerryCostRange(
    pt.get<float>("ferry_cost", kDefaultFerryCost)
  );

  // Modify ferry penalty and edge weighting based on use_ferry_ factor
  use_ferry_ = kUseFerryRange(
    pt.get<float>("use_ferry", 0.5f)
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

  // Set the speed penalty threshold and factor. With useroads = 1 the
  // threshold is 70 kph (near 50 MPH).
  speed_penalty_threshold_ = kSpeedPenaltyThreshold +
      static_cast<uint32_t>(use_roads_ * 30.0f);

  // Create speed cost table and penalty table (to avoid division in costing)
  float avoid_roads = (1.0f - use_roads_) * 0.75f + 0.25;
  speedfactor_[0] = kSecPerHour;
  speedpenalty_[0] = 0.0f;
  for (uint32_t s = 1; s <= kMaxSpeedKph; s++) {
    speedfactor_[s] = (kSecPerHour * 0.001f) / static_cast<float>(s);

    float base_pen = 0.0f;
    if (s <= 40) {
      base_pen = (static_cast<float>(s) / 40.0f);
    } else if (s <= 65) {
      base_pen = ((static_cast<float>(s) / 25.0f) - 0.6f);
    } else {
      base_pen = ((static_cast<float>(s) / 50.0) + 0.7f);
    }
    speedpenalty_[s] = (base_pen - 1.0f) * avoid_roads + 1.0f;
  }

  // Populate the grade penalties (based on use_hills factor).
  use_hills_   = kUseHillsRange(
    pt.get<float>("use_hills", kDefaultUseHills)
  );
  float avoid_hills = (1.0f - use_hills_);
  for (uint32_t i = 0; i <= kMaxGradeFactor; i++) {
    grade_penalty[i] = avoid_hills * kAvoidHillsStrength[i];
  }
}

// Destructor
BicycleCost::~BicycleCost() {
}

// Get the access mode used by this costing method.
uint32_t BicycleCost::access_mode() const {
  return kBicycleAccess;
}

// Check if access is allowed on the specified edge.
bool BicycleCost::Allowed(const baldr::DirectedEdge* edge,
                          const EdgeLabel& pred,
                          const baldr::GraphTile*& tile,
                          const baldr::GraphId& edgeid) const {
  // TODO - obtain and check the access restrictions.

  // Check bicycle access and turn restrictions. Bicycles should obey
  // vehicular turn restrictions. Allow Uturns at dead ends only.
  // Skip impassable edges and shortcut edges.
  if (!(edge->forwardaccess() & kBicycleAccess) || edge->is_shortcut() ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
      IsUserAvoidEdge(edgeid)) {
    return false;
  }

  // Disallow transit connections
  // (except when set for multi-modal routes (FUTURE)
  if (edge->use() == Use::kTransitConnection || edge->use() == Use::kEgressConnection ||
      edge->use() == Use::kPlatformConnection /* && !allow_transit_connections_*/) {
    return false;
  }

  // Prohibit certain roads based on surface type and bicycle type
  return edge->surface() <= worst_allowed_surface_;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool BicycleCost::AllowedReverse(const baldr::DirectedEdge* edge,
               const EdgeLabel& pred,
               const baldr::DirectedEdge* opp_edge,
               const baldr::GraphTile*& tile,
               const baldr::GraphId& opp_edgeid) const {
  // TODO - obtain and check the access restrictions.

  // Check access, U-turn (allow at dead-ends), and simple turn restriction.
  // Do not allow transit connection edges.
  if (!(opp_edge->forwardaccess() & kBicycleAccess) ||
        opp_edge->is_shortcut() || opp_edge->use() == Use::kTransitConnection ||
        opp_edge->use() == Use::kEgressConnection || opp_edge->use() == Use::kPlatformConnection ||
       (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
       (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
       IsUserAvoidEdge(opp_edgeid)) {
    return false;
  }

  // Prohibit certain roads based on surface type and bicycle type
  return opp_edge->surface() <= worst_allowed_surface_;
}

// Check if access is allowed at the specified node.
bool BicycleCost::Allowed(const baldr::NodeInfo* node) const {
  return (node->access() & kBicycleAccess);
}

// Returns the cost to traverse the edge and an estimate of the actual time
// (in seconds) to traverse the edge.
Cost BicycleCost::EdgeCost(const baldr::DirectedEdge* edge) const {
  // Stairs/steps - use a high fixed cost so they are generally avoided.
  if (edge->use() == Use::kSteps) {
    float sec = (edge->length() * speedfactor_[1]);
    return {sec * kBicycleStepsFactor, sec };
  }

  // Ferries are a special case - they use the ferry speed (stored on the edge)
  if (edge->use() == Use::kFerry) {
    // Compute elapsed time based on speed. Modulate cost with weighting factors.
    float sec = (edge->length() * speedfactor_[edge->speed()]);
    return { sec * ferry_factor_, sec };
  }

  // If you have to dismount on the edge then we set speed to an average walking speed
  // Otherwise, Update speed based on surface factor. Lower speed for rougher surfaces
  // depending on the bicycle type. Modulate speed based on weighted grade
  // (relative measure of elevation change along the edge)
  uint32_t bike_speed = edge->dismount () ?
      kDismountSpeed :
      static_cast<uint32_t>((speed_ *
        surface_speed_factor_[static_cast<uint32_t>(edge->surface())] *
        kGradeBasedSpeedFactor[edge->weighted_grade()]) + 0.5f);

  // Represents how stressful a roadway is without looking at grade or cycle accommodations
  float roadway_stress = 1.0f;
  // Represents the amount of accommodation that is being made for bicycling
  float accommodation_factor = 1.0f;

  // Special use cases: cycleway, footway, and path
  uint32_t road_speed = static_cast<uint32_t>(edge->speed() + 0.5f);
  if (edge->use() == Use::kCycleway || edge->use() == Use::kFootway || edge->use() == Use::kPath) {

    // Differentiate how segregated the way is from pedestrians
    if (edge->cyclelane() == CycleLane::kSeparated) { // No pedestrians allowed on path
      accommodation_factor = use_roads_ * 0.8f;
    } else if (edge->cyclelane() == CycleLane::kDedicated) { // Segregated lane from pedestrians
      accommodation_factor = 0.1f + use_roads_ * 0.9f;
    } else { // Share path with pedestrians
      accommodation_factor = 0.2f + use_roads_;
    }
  } else if (edge->use() == Use::kMountainBike &&
             type_ == BicycleType::kMountain) {
    // Slightly less reduction than a footway or path because even with a mountain bike
    // these paths can be a little stressful to ride. No traffic though so still favorable
    accommodation_factor = 0.3f + use_roads_;
  } else if (edge->use() == Use::kLivingStreet) {
    roadway_stress = 0.2f + use_roads_ * 0.8f;
  } else if (edge->use() == Use::kTrack) {
    roadway_stress = 0.5f + use_roads_;
  } else if (edge->use() == Use::kDriveway) {
    // Heavily penalize driveways
    roadway_stress = kDrivewayFactor;
  } else {
    // Favor roads where a cycle lane exists
    if (edge->cyclelane() == CycleLane::kShared) {
      accommodation_factor = 0.9f + use_roads_ * 0.05f;
    } else if (edge->cyclelane() == CycleLane::kDedicated) {
      accommodation_factor = 0.4f + use_roads_ * 0.45f;
    } else if (edge->cyclelane() == CycleLane::kSeparated) {
      accommodation_factor = 0.15f + use_roads_ * 0.6f;
    } else if (edge->shoulder()) {
      // If no cycle lane, but there is a shoulder then have a slight preference for this road
      accommodation_factor = 0.7f + use_roads_ * 0.2f;
    } else if (edge->destonly()) {
      // Slight penalty going though destination only areas if no bike lanes
      roadway_stress += kDestinationOnlyFactor;
    }

    // Penalize roads that have more than one lane (in the direction of travel)
    if (edge->lanecount() > 1) {
      roadway_stress += (static_cast<float>(edge->lanecount()) - 1) * 0.05f * road_factor_;
    }

    // Add in penalization for road classification
    roadway_stress += road_factor_ * kRoadClassFactor[static_cast<uint32_t>(edge->classification())];
    // Then multiply by speed so that higher classified roads are more severely punished for being fast.
    roadway_stress *= speedpenalty_[road_speed];
  }

  // We want to try and avoid roads that specify to use a cycling path to the side
  if (edge->use_sidepath()) {
    accommodation_factor += 3.0f * (1.0f - use_roads_);
  }

  // Favor bicycle networks very slightly.
  // TODO - do we need to differentiate between types of network?
  if (edge->bike_network() > 0) {
    accommodation_factor *= kBicycleNetworkFactor;
  }

  // The stress of this road after accommodation but before grade
  float total_stress = accommodation_factor * roadway_stress;

  float surface_factor = 0.0f;
  if (edge->surface () >= minimal_surface_penalized_)
  {
    surface_factor = avoid_bad_surfaces_ * kSurfaceFactors[static_cast<uint32_t> (edge->surface ())
                                   - static_cast<uint32_t> (minimal_surface_penalized_)];
  }

  // Create a final edge factor based on total stress and the weighted grade penalty for the edge.
  float factor = 1.0f + grade_penalty[edge->weighted_grade()] + total_stress + surface_factor;

  // Compute elapsed time based on speed. Modulate cost with weighting factors.
  float sec = (edge->length() * speedfactor_[bike_speed]);
  return { sec * factor, sec };
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost BicycleCost::TransitionCost(const baldr::DirectedEdge* edge,
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

  // Additional penalties without any time cost
  uint32_t idx = pred.opp_local_idx();
  if (pred.use() != Use::kAlley && edge->use() == Use::kAlley) {
    penalty += alley_penalty_;
  }
  if (pred.use() != Use::kDriveway && edge->use() == Use::kDriveway) {
    penalty += driveway_penalty_;
  }
  if ((pred.use() != Use::kFerry && edge->use() == Use::kFerry)) {
    seconds += ferry_cost_;
    penalty += ferry_penalty_;
  }

  // Ignore name inconsistency when entering a link to avoid double penalizing.
  if (!edge->link() && !node->name_consistency(idx, edge->localedgeidx())) {
    // Slight maneuver penalty
    penalty += maneuver_penalty_;
  }

  float class_factor = kRoadClassFactor[static_cast<uint32_t>(edge->classification())];

  // Reduce penalty to make this turn if the road we are turning on has some kind of bicycle accommodation
  float bike_accom = 1.0f;
  if (edge->use() == Use::kCycleway || edge->use() == Use::kFootway || edge->use() == Use::kPath) {
    bike_accom = 0.05f;
    // These uses are classified as "service/other" roads but should not be penalized as such so we change it's factor
    class_factor = 0.1f;
  } else if (edge->use() == Use::kLivingStreet){
    bike_accom = 0.15f;
  } else {
    if (edge->cyclelane() == CycleLane::kShared) {
      bike_accom = 0.5f;
    } else if (edge->cyclelane() == CycleLane::kDedicated) {
      bike_accom = 0.25f;
    } else if (edge->cyclelane() == CycleLane::kSeparated) {
      bike_accom = 0.1f;
    } else if (edge->shoulder()) {
      bike_accom = 0.4f;
    }
  }

  float turn_stress = 1.0f;

  if (edge->stopimpact(idx) > 0) {
    // Increase turn stress depending on the kind of turn that has to be made.
    float turn_penalty = (edge->drive_on_right()) ?
        kRightSideTurnPenalties[static_cast<uint32_t>(edge->turntype(idx))] :
        kLeftSideTurnPenalties[static_cast<uint32_t>(edge->turntype(idx))];
    turn_stress += turn_penalty;

    // Take the higher of the turn degree cost and the crossing cost
    float turn_cost = (edge->drive_on_right()) ?
        kRightSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))] :
        kLeftSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))];
    if (turn_cost < kTCCrossing &&
        edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    }

    // Transition time = densityfactor * stopimpact * turncost
    seconds += trans_density_factor_[node->density()] *
               edge->stopimpact(idx) * turn_cost;
  }

  // Reduce stress by road class factor the closer use_roads_ is to 0
  float avoid_roads = 1.0f - use_roads_;
  turn_stress *= (class_factor * avoid_roads) + use_roads_ + 1.0f;

  // Penalize transition to higher class road.
  if (edge->classification() < pred.classification() && edge->use() != Use::kLivingStreet) {
    penalty += 10.0f * (static_cast<uint32_t>(pred.classification()) -
                        static_cast<uint32_t>(edge->classification()));
    // Reduce the turn stress if there is a traffic signal
    turn_stress += (node->traffic_signal()) ? 0.4 : 1.0;
  }

  // Reduce penalty by bike_accom the closer use_roads_ is to 0
  penalty *= (bike_accom * avoid_roads) + use_roads_;

  // Return cost (time and penalty)
  return { (seconds * (turn_stress + 1.0f)) + penalty, seconds };
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// pred is the opposing current edge in the reverse tree
// edge is the opposing predecessor in the reverse tree
Cost BicycleCost::TransitionCostReverse(const uint32_t idx,
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

  // Additional penalties without any time cost
  if (pred->use() != Use::kAlley && edge->use() == Use::kAlley) {
    penalty += alley_penalty_;
  }
  if (pred->use() != Use::kDriveway && edge->use() == Use::kDriveway) {
    penalty += driveway_penalty_;
  }
  if ((pred->use() != Use::kFerry && edge->use() == Use::kFerry)) {
    seconds += ferry_cost_;
    penalty += ferry_penalty_;
  }

  // Ignore name inconsistency when entering a link to avoid double penalizing.
  if (!edge->link() && !node->name_consistency(idx, edge->localedgeidx())) {
    // Slight maneuver penalty
    penalty += maneuver_penalty_;
  }

  float class_factor = kRoadClassFactor[static_cast<uint32_t>(edge->classification())];

  // Reduce penalty to make this turn if the road we are turning on has some kind of bicycle accommodation
  float bike_accom = 1.0f;
  if (edge->use() == Use::kCycleway || edge->use() == Use::kFootway || edge->use() == Use::kPath) {
    bike_accom = 0.05f;
    // These uses are considered "service/other" roads but should not be penalized as such so we change it's factor
    class_factor = 0.1f;
  } else if (edge->use() == Use::kLivingStreet){
    bike_accom = 0.15f;
  } else {
    if (edge->cyclelane() == CycleLane::kShared) {
      bike_accom = 0.5f;
    } else if (edge->cyclelane() == CycleLane::kDedicated) {
      bike_accom = 0.25f;
    } else if (edge->cyclelane() == CycleLane::kSeparated) {
      bike_accom = 0.1f;
    } else if (edge->shoulder()) {
      bike_accom = 0.4f;
    }
  }

  float turn_stress = 1.0f;

  if (edge->stopimpact(idx) > 0) {
    // Increase turn stress depending on the kind of turn that has to be made.
    float turn_penalty = (edge->drive_on_right()) ?
        kRightSideTurnPenalties[static_cast<uint32_t>(edge->turntype(idx))] :
        kLeftSideTurnPenalties[static_cast<uint32_t>(edge->turntype(idx))];
    turn_stress += turn_penalty;

    // Take the higher of the turn degree cost and the crossing cost
    float turn_cost = (edge->drive_on_right()) ?
        kRightSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))] :
        kLeftSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))];
    if (turn_cost < kTCCrossing &&
        edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    }

    // Transition time = densityfactor * stopimpact * turncost
    seconds += trans_density_factor_[node->density()] *
               edge->stopimpact(idx) * turn_cost;
  }

  // Reduce stress by road class factor the closer use_roads_ is to 0
  float avoid_roads = 1.0f - use_roads_;
  turn_stress *= (class_factor * avoid_roads) + use_roads_ + 1.0f;

  // Penalize transition to higher class road.
  if (edge->classification() < pred->classification() && edge->use() != Use::kLivingStreet) {
    penalty += 10.0f * (static_cast<uint32_t>(pred->classification()) -
                        static_cast<uint32_t>(edge->classification()));
    // Reduce the turn stress if there is a traffic signal
    turn_stress += (node->traffic_signal()) ? 0.4 : 1.0;
  }

  // Reduce penalty by bike_accom the closer use_roads_ is to 0
  penalty *= (bike_accom * avoid_roads) + use_roads_;

  // Return cost (time and penalty)
  return { (seconds * (turn_stress + 1.0f)) + penalty, seconds };
}

/**
 * Get the cost factor for A* heuristics. This factor is multiplied
 * with the distance to the destination to produce an estimate of the
 * minimum cost to the destination. The A* heuristic must underestimate the
 * cost to the destination. So a time based estimate based on speed should
 * assume the maximum speed is used to the destination such that the time
 * estimate is less than the least possible time along roads.
 */
float BicycleCost::AStarCostFactor() const {
  // Assume max speed of 80 kph (50 MPH)
  return speedfactor_[80];
}

// Returns the current travel type.
uint8_t BicycleCost::travel_type() const {
  return static_cast<uint8_t>(type_);
}

cost_ptr_t CreateBicycleCost(const boost::property_tree::ptree& config) {
  return std::make_shared<BicycleCost>(config);
}

}
}

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

BicycleCost* make_bicyclecost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({")" << property << R"(":)" << testVal << "}";
  boost::property_tree::ptree costing_ptree;
  boost::property_tree::read_json(ss, costing_ptree);
  return new BicycleCost(costing_ptree);
}

std::uniform_real_distribution<float>* make_distributor_from_range (const ranged_default_t<float>& range) {
  float rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<float>(range.min - rangeLength, range.max + rangeLength);
}

void testBicycleCostParams() {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::default_random_engine generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> distributor;
  std::shared_ptr<BicycleCost> ctorTester;

  // maneuver_penalty_
  distributor.reset(make_distributor_from_range(kManeuverPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("maneuver_penalty", (*distributor)(generator)));
    if (ctorTester->maneuver_penalty_ < kManeuverPenaltyRange.min ||
        ctorTester->maneuver_penalty_ > kManeuverPenaltyRange.max) {
      throw std::runtime_error ("maneuver_penalty_ is not within it's range");
    }
  }

  // driveway_penalty_
  distributor.reset(make_distributor_from_range(kDrivewayPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("driveway", (*distributor)(generator)));
    if (ctorTester->driveway_penalty_ < kDrivewayPenaltyRange.min ||
        ctorTester->driveway_penalty_ > kDrivewayPenaltyRange.max) {
      throw std::runtime_error ("driveway_penalty_ is not within it's range");
    }
  }

  // alley_penalty_
  distributor.reset(make_distributor_from_range(kAlleyPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("alley_penalty", (*distributor)(generator)));
    if (ctorTester->alley_penalty_ < kAlleyPenaltyRange.min ||
        ctorTester->alley_penalty_ > kAlleyPenaltyRange.max) {
      throw std::runtime_error ("alley_penalty_ is not within it's range");
    }
  }

  // gate_cost_
  distributor.reset(make_distributor_from_range(kGateCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("gate_cost", (*distributor)(generator)));
    if (ctorTester->gate_cost_ < kGateCostRange.min ||
        ctorTester->gate_cost_ > kGateCostRange.max) {
      throw std::runtime_error ("gate_cost_ is not within it's range");
    }
  }

  // gate_penalty_
  distributor.reset(make_distributor_from_range(kGatePenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("gate_penalty", (*distributor)(generator)));
    if (ctorTester->gate_penalty_ < kGatePenaltyRange.min ||
        ctorTester->gate_penalty_ > kGatePenaltyRange.max) {
      throw std::runtime_error ("gate_penalty_ is not within it's range");
    }
  }

  // ferry_cost_
  distributor.reset(make_distributor_from_range(kFerryCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("ferry_cost", (*distributor)(generator)));
    if (ctorTester->ferry_cost_ < kFerryCostRange.min ||
        ctorTester->ferry_cost_ > kFerryCostRange.max) {
      throw std::runtime_error ("ferry_cost_ is not within it's range");
    }
  }

  // country_crossing_cost_
  distributor.reset(make_distributor_from_range(kCountryCrossingCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("country_crossing_cost", (*distributor)(generator)));
    if (ctorTester->country_crossing_cost_ < kCountryCrossingCostRange.min ||
        ctorTester->country_crossing_cost_ > kCountryCrossingCostRange.max) {
      throw std::runtime_error ("country_crossing_cost_ is not within it's range");
    }
  }

  // country_crossing_penalty_
  distributor.reset(make_distributor_from_range(kCountryCrossingPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("country_crossing_penalty", (*distributor)(generator)));
    if (ctorTester->country_crossing_penalty_ < kCountryCrossingPenaltyRange.min ||
        ctorTester->country_crossing_penalty_ > kCountryCrossingPenaltyRange.max) {
      throw std::runtime_error ("country_crossing_penalty_ is not within it's range");
    }
  }

  // use_roads_
  distributor.reset(make_distributor_from_range(kUseRoadRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("use_roads", (*distributor)(generator)));
    if (ctorTester->use_roads_ < kUseRoadRange.min ||
        ctorTester->use_roads_ > kUseRoadRange.max) {
      throw std::runtime_error ("use_roads_ is not within it's range");
    }
  }

  // speed_
  constexpr ranged_default_t<float> kRoadCyclingSpeedRange {kMinCyclingSpeed, kDefaultCyclingSpeed[0], kMaxCyclingSpeed};
  distributor.reset(make_distributor_from_range(kRoadCyclingSpeedRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("cycling_speed", (*distributor)(generator)));
    if (ctorTester->speed_ < kRoadCyclingSpeedRange.min ||
        ctorTester->speed_ > kRoadCyclingSpeedRange.max) {
      throw std::runtime_error ("speed_ is not within it's range");
    }
  }

  // use_ferry_
  distributor.reset(make_distributor_from_range(kUseFerryRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("use_ferry", (*distributor)(generator)));
    if (ctorTester->use_ferry_ < kUseFerryRange.min ||
        ctorTester->use_ferry_ > kUseFerryRange.max) {
      throw std::runtime_error ("use_ferry_ is not within it's range");
    }
  }
  
  // use_hills_
  distributor.reset(make_distributor_from_range(kUseHillsRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("use_hills", (*distributor)(generator)));
    if (ctorTester->use_hills_ < kUseHillsRange.min ||
        ctorTester->use_hills_ > kUseHillsRange.max) {
      throw std::runtime_error ("use_hills_ is not within it's range");
    }
  }

}
}

int main() {
  test::suite suite("costing");

  suite.test(TEST_CASE(testBicycleCostParams));

  return suite.tear_down();
}

#endif
