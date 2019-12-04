#include "sif/bicyclecost.h"
#include "sif/costconstants.h"

#include "baldr/accessrestriction.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "midgard/constants.h"
#include "midgard/util.h"

#ifdef INLINE_TEST
#include "test/test.h"
#include "worker.h"
#include <random>
#endif

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {

// Base transition costs
// TODO - can we define these in dynamiccost.h and override here if they differ?
constexpr float kDefaultDestinationOnlyPenalty = 600.0f; // Seconds
constexpr float kDefaultManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultAlleyPenalty = 60.0f;            // Seconds
constexpr float kDefaultGateCost = 30.0f;                // Seconds
constexpr float kDefaultGatePenalty = 600.0f;            // Seconds
constexpr float kDefaultFerryCost = 300.0f;              // Seconds
constexpr float kDefaultCountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultCountryCrossingPenalty = 0.0f;   // Seconds

// Other options
constexpr float kDefaultDrivewayPenalty = 300.0f; // Seconds
constexpr float kDefaultUseRoad = 0.25f;          // Factor between 0 and 1
constexpr float kDefaultUseFerry = 0.5f;          // Factor between 0 and 1
constexpr float kDefaultAvoidBadSurfaces = 0.25f; // Factor between 0 and 1
const std::string kDefaultBicycleType = "Hybrid"; // Bicycle type

// Default turn costs - modified by the stop impact.
constexpr float kTCStraight = 0.15f;
constexpr float kTCFavorableSlight = 0.2f;
constexpr float kTCFavorable = 0.3f;
constexpr float kTCFavorableSharp = 0.5f;
constexpr float kTCCrossing = 0.75f;
constexpr float kTCUnfavorableSlight = 0.4f;
constexpr float kTCUnfavorable = 1.0f;
constexpr float kTCUnfavorableSharp = 1.5f;
constexpr float kTCReverse = 5.0f;

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = {kTCStraight,       kTCFavorableSlight,  kTCFavorable,
                                         kTCFavorableSharp, kTCReverse,          kTCUnfavorableSharp,
                                         kTCUnfavorable,    kTCUnfavorableSlight};
constexpr float kLeftSideTurnCosts[] = {kTCStraight,         kTCUnfavorableSlight, kTCUnfavorable,
                                        kTCUnfavorableSharp, kTCReverse,           kTCFavorableSharp,
                                        kTCFavorable,        kTCFavorableSlight};

// Turn stress penalties for low-stress bike.
constexpr float kTPStraight = 0.0f;
constexpr float kTPFavorableSlight = 0.25f;
constexpr float kTPFavorable = 0.75f;
constexpr float kTPFavorableSharp = 1.0f;
constexpr float kTPUnfavorableSlight = 0.75f;
constexpr float kTPUnfavorable = 1.75f;
constexpr float kTPUnfavorableSharp = 2.25f;
constexpr float kTPReverse = 4.0f;

constexpr float kRightSideTurnPenalties[] = {kTPStraight,    kTPFavorableSlight,
                                             kTPFavorable,   kTPFavorableSharp,
                                             kTPReverse,     kTPUnfavorableSharp,
                                             kTPUnfavorable, kTPUnfavorableSlight};
constexpr float kLeftSideTurnPenalties[] = {kTPStraight,    kTPUnfavorableSlight,
                                            kTPUnfavorable, kTPUnfavorableSharp,
                                            kTPReverse,     kTPFavorableSharp,
                                            kTPFavorable,   kTPFavorableSlight};

// Additional stress factor for designated truck routes
const float kTruckStress = 0.5f;

// Cost of traversing an edge with steps. Make this high but not impassible.
const float kBicycleStepsFactor = 8.0f;

// Default cycling speed on smooth, flat roads - based on bicycle type (KPH)
constexpr float kDefaultCyclingSpeed[] = {
    25.0f, // Road bicycle: ~15.5 MPH
    20.0f, // Cross bicycle: ~13 MPH
    18.0f, // Hybrid or "city" bicycle: ~11.5 MPH
    16.0f  // Mountain bicycle: ~10 MPH
};

constexpr float kDismountSpeed = 5.1f;

// Minimum and maximum average bicycling speed (to validate input).
// Maximum is just above the fastest average speed in Tour de France time trial
constexpr float kMinCyclingSpeed = 5.0f;  // KPH
constexpr float kMaxCyclingSpeed = 60.0f; // KPH

// Speed factors based on surface types (defined for each bicycle type).
// These values determine the percentage by which speed us reduced for
// each surface type. (0 values indicate unusable surface types).
constexpr float kRoadSurfaceSpeedFactors[] = {1.0f, 1.0f, 0.9f, 0.6f, 0.5f, 0.3f, 0.2f, 0.0f};
constexpr float kHybridSurfaceSpeedFactors[] = {1.0f, 1.0f, 1.0f, 0.8f, 0.6f, 0.4f, 0.25f, 0.0f};
constexpr float kCrossSurfaceSpeedFactors[] = {1.0f, 1.0f, 1.0f, 0.8f, 0.7f, 0.5f, 0.4f, 0.0f};
constexpr float kMountainSurfaceSpeedFactors[] = {1.0f, 1.0f, 1.0f, 1.0f, 0.9f, 0.75f, 0.55f, 0.0f};

// Worst allowed surface based on bicycle type
constexpr Surface kWorstAllowedSurface[] = {Surface::kCompacted, // Road bicycle
                                            Surface::kGravel,    // Cross
                                            Surface::kDirt,      // Hybrid
                                            Surface::kPath};     // Mountain

constexpr float kSurfaceFactors[] = {1.0f, 2.5f, 4.5f, 7.0f};

// Weighting factor based on road class. These apply penalties to higher class
// roads. These penalties are modulated by the useroads factor - further
// avoiding higher class roads for those with low propensity for using roads.
constexpr float kRoadClassFactor[] = {
    1.0f,  // Motorway
    0.4f,  // Trunk
    0.2f,  // Primary
    0.1f,  // Secondary
    0.05f, // Tertiary
    0.05f, // Unclassified
    0.0f,  // Residential
    0.5f   // Service, other
};

// Speed adjustment factors based on weighted grade. Comments here show an
// example of speed changes based on "grade", using a base speed of 18 MPH
// on flat roads
constexpr float kGradeBasedSpeedFactor[] = {
    2.2f,  // -10%  - 39.6
    2.0f,  // -8%   - 36
    1.9f,  // -6.5% - 34.2
    1.7f,  // -5%   - 30.6
    1.4f,  // -3%   - 25
    1.2f,  // -1.5% - 21.6
    1.0f,  // 0%    - 18
    0.95f, // 1.5%  - 17
    0.85f, // 3%    - 15
    0.75f, // 5%    - 13.5
    0.65f, // 6.5%  - 12
    0.55f, // 8%    - 10
    0.5f,  // 10%   - 9
    0.45f, // 11.5% - 8
    0.4f,  // 13%   - 7
    0.3f   // 15%   - 5.5
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
    2.0f,  // -10%  - Treacherous descent possible
    1.0f,  // -8%   - Steep downhill
    0.5f,  // -6.5% - Good downhill - where is the bottom?
    0.2f,  // -5%   - Picking up speed!
    0.1f,  // -3%   - Modest downhill
    0.0f,  // -1.5% - Smooth slight downhill, ride this all day!
    0.05f, // 0%    - Flat, no avoidance
    0.1f,  // 1.5%  - These are called "false flat"
    0.3f,  // 3%    - Slight rise
    0.8f,  // 5%    - Small hill
    2.0f,  // 6.5%  - Starting to feel this...
    3.0f,  // 8%    - Moderately steep
    4.5f,  // 10%   - Getting tough
    6.5f,  // 11.5% - Tiring!
    10.0f, // 13%   - Ooof - this hurts
    12.0f  // 15%   - Only for the strongest!
};

// Edge speed above which extra penalties apply (to avoid roads with higher
// speed traffic). This threshold is adjusted upwards with higher useroads
// factors.
constexpr uint32_t kSpeedPenaltyThreshold = 40; // 40 KPH ~ 25 MPH

// How much to favor bicycle networks.
constexpr float kBicycleNetworkFactor = 0.95f;

// Valid ranges and defaults
constexpr ranged_default_t<float> kDestinationOnlyPenaltyRange{0, kDefaultDestinationOnlyPenalty,
                                                               kMaxPenalty};
constexpr ranged_default_t<float> kManeuverPenaltyRange{0.0f, kDefaultManeuverPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kDrivewayPenaltyRange{0.0f, kDefaultDrivewayPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kAlleyPenaltyRange{0.0f, kDefaultAlleyPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kGateCostRange{0.0f, kDefaultGateCost, kMaxPenalty};
constexpr ranged_default_t<float> kGatePenaltyRange{0.0f, kDefaultGatePenalty, kMaxPenalty};
constexpr ranged_default_t<float> kFerryCostRange{0.0f, kDefaultFerryCost, kMaxPenalty};
constexpr ranged_default_t<float> kCountryCrossingCostRange{0.0f, kDefaultCountryCrossingCost,
                                                            kMaxPenalty};
constexpr ranged_default_t<float> kCountryCrossingPenaltyRange{0.0f, kDefaultCountryCrossingPenalty,
                                                               kMaxPenalty};
constexpr ranged_default_t<float> kUseRoadRange{0.0f, kDefaultUseRoad, 1.0f};
constexpr ranged_default_t<float> kUseFerryRange{0.0f, kDefaultUseFerry, 1.0f};
constexpr ranged_default_t<float> kUseHillsRange{0.0f, kDefaultUseHills, 1.0f};
constexpr ranged_default_t<float> kAvoidBadSurfacesRange{0.0f, kDefaultAvoidBadSurfaces, 1.0f};
} // namespace

/**
 * Derived class providing dynamic edge costing for bicycle routes.
 */
class BicycleCost : public DynamicCost {
public:
  /**
   * Construct bicycle costing. Pass in cost type and options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  options pbf with request options.
   */
  BicycleCost(const Costing costing, const Options& options);

  // virtual destructor
  virtual ~BicycleCost() {
  }

  /**
   * Get the access mode used by this costing method.
   * @return  Returns access mode.
   */
  uint32_t access_mode() const {
    return kBicycleAccess;
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
                       const uint32_t tz_index,
                       bool& time_restricted) const;

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
                              const uint32_t tz_index,
                              bool& has_time_restrictions) const;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present. (TODO - others?)
   * @param  node  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const {
    return (node->access() & kBicycleAccess);
  }

  /**
   * Only transit costings are valid for this method call, hence we throw
   * @param edge
   * @param departure
   * @param curr_time
   * @return
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::TransitDeparture* departure,
                        const uint32_t curr_time) const {
    throw std::runtime_error("BicycleCost::EdgeCost does not support transit edges");
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge      Pointer to a directed edge.
   * @param   tile      Current tile.
   * @param   seconds   Time of week in seconds.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphTile* tile,
                        const uint32_t seconds) const;

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
    // Assume max speed of 2 * the average speed set for costing
    return speedfactor_[2 * static_cast<uint32_t>(speed_)];
  }

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const {
    return static_cast<uint8_t>(type_);
  }

  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes

  float speedfactor_[kMaxSpeedKph + 1]; // Cost factors based on speed in kph
  float use_roads_;                     // Preference of using roads between 0 and 1
  float road_factor_;                   // Road factor based on use_roads_
  float avoid_bad_surfaces_;            // Preference of avoiding bad surfaces for the bike type

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
    float a = avoid_bad_surfaces_;
    return [s, a](const baldr::DirectedEdge* edge) {
      if (edge->is_shortcut() || !(edge->forwardaccess() & kBicycleAccess) ||
          edge->use() == Use::kSteps || (a == 1.0f && edge->surface() > s)) {
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
    // throw back a lambda that checks the access for this type of costing
    return [](const baldr::NodeInfo* node) { return !(node->access() & kBicycleAccess); };
  }
};

// Bicycle route costs are distance based with some favor/avoid based on
// attribution. Speed is derived based on bicycle type or user input and
// is modulated based on surface type and grade factors.

// Constructor
BicycleCost::BicycleCost(const Costing costing, const Options& options)
    : DynamicCost(options, TravelMode::kBicycle) {
  // Grab the costing options based on the specified costing type
  const CostingOptions& costing_options = options.costing_options(static_cast<int>(costing));

  // Set hierarchy to allow unlimited transitions
  for (auto& h : hierarchy_limits_) {
    h.max_up_transitions = kUnlimitedTransitions;
  }

  // Get the base costs
  get_base_costs(costing_options);

  // Get the bicycle type - enter as string and convert to enum
  std::string bicycle_type = costing_options.transport_type();
  if (bicycle_type == "Cross") {
    type_ = BicycleType::kCross;
  } else if (bicycle_type == "Road") {
    type_ = BicycleType::kRoad;
  } else if (bicycle_type == "Mountain") {
    type_ = BicycleType::kMountain;
  } else {
    type_ = BicycleType::kHybrid;
  }

  speed_ = costing_options.cycling_speed();
  avoid_bad_surfaces_ = costing_options.avoid_bad_surfaces();
  minimal_surface_penalized_ = kWorstAllowedSurface[static_cast<uint32_t>(type_)];
  worst_allowed_surface_ = avoid_bad_surfaces_ == 1.0f ? minimal_surface_penalized_ : Surface::kPath;

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
  use_roads_ = costing_options.use_roads();

  // Set the road classification factor. use_roads factors above 0.5 start to
  // reduce the weight difference between road classes while factors below 0.5
  // start to increase the differences.
  road_factor_ = (use_roads_ >= 0.5f) ? 1.5f - use_roads_ : 2.0f - use_roads_ * 2.0f;

  // Set the speed penalty threshold and factor. With useroads = 1 the
  // threshold is 70 kph (near 50 MPH).
  speed_penalty_threshold_ = kSpeedPenaltyThreshold + static_cast<uint32_t>(use_roads_ * 30.0f);

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

  // Populate the grade penalties (based on use_hills factor - value between 0 and 1)
  float use_hills = costing_options.use_hills();
  float avoid_hills = (1.0f - use_hills);
  for (uint32_t i = 0; i <= kMaxGradeFactor; i++) {
    grade_penalty[i] = avoid_hills * kAvoidHillsStrength[i];
  }
}

// Check if access is allowed on the specified edge.
bool BicycleCost::Allowed(const baldr::DirectedEdge* edge,
                          const EdgeLabel& pred,
                          const baldr::GraphTile*& tile,
                          const baldr::GraphId& edgeid,
                          const uint64_t current_time,
                          const uint32_t tz_index,
                          bool& has_time_restrictions) const {
  // Check bicycle access and turn restrictions. Bicycles should obey
  // vehicular turn restrictions. Allow Uturns at dead ends only.
  // Skip impassable edges and shortcut edges.
  if (!(edge->forwardaccess() & kBicycleAccess) || edge->is_shortcut() ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) || IsUserAvoidEdge(edgeid)) {
    return false;
  }

  // Disallow transit connections
  // (except when set for multi-modal routes (FUTURE)
  if (edge->use() == Use::kTransitConnection || edge->use() == Use::kEgressConnection ||
      edge->use() == Use::kPlatformConnection /* && !allow_transit_connections_*/) {
    return false;
  }

  // Prohibit certain roads based on surface type and bicycle type
  if (edge->surface() > worst_allowed_surface_) {
    return false;
  }
  return DynamicCost::EvaluateRestrictions(kBicycleAccess, edge, tile, edgeid, current_time, tz_index,
                                           has_time_restrictions);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool BicycleCost::AllowedReverse(const baldr::DirectedEdge* edge,
                                 const EdgeLabel& pred,
                                 const baldr::DirectedEdge* opp_edge,
                                 const baldr::GraphTile*& tile,
                                 const baldr::GraphId& opp_edgeid,
                                 const uint64_t current_time,
                                 const uint32_t tz_index,
                                 bool& has_time_restrictions) const {
  // Check access, U-turn (allow at dead-ends), and simple turn restriction.
  // Do not allow transit connection edges.
  if (!(opp_edge->forwardaccess() & kBicycleAccess) || opp_edge->is_shortcut() ||
      opp_edge->use() == Use::kTransitConnection || opp_edge->use() == Use::kEgressConnection ||
      opp_edge->use() == Use::kPlatformConnection ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (opp_edge->restrictions() & (1 << pred.opp_local_idx())) || IsUserAvoidEdge(opp_edgeid)) {
    return false;
  }

  // Prohibit certain roads based on surface type and bicycle type
  if (edge->surface() > worst_allowed_surface_) {
    return false;
  }
  return DynamicCost::EvaluateRestrictions(kBicycleAccess, edge, tile, opp_edgeid, current_time,
                                           tz_index, has_time_restrictions);
}

// Returns the cost to traverse the edge and an estimate of the actual time
// (in seconds) to traverse the edge.
Cost BicycleCost::EdgeCost(const baldr::DirectedEdge* edge,
                           const baldr::GraphTile* tile,
                           const uint32_t seconds) const {
  auto speed = tile->GetSpeed(edge, flow_mask_, seconds);

  // Stairs/steps - high cost (travel speed = 1kph) so they are generally avoided.
  if (edge->use() == Use::kSteps) {
    float sec = (edge->length() * speedfactor_[1]);
    return {sec * kBicycleStepsFactor, sec};
  }

  // Ferries are a special case - they use the ferry speed (stored on the edge)
  if (edge->use() == Use::kFerry) {
    // Compute elapsed time based on speed. Modulate cost with weighting factors.
    float sec = (edge->length() * speedfactor_[speed]);
    return {sec * ferry_factor_, sec};
  }

  // If you have to dismount on the edge then we set speed to an average walking speed
  // Otherwise, Update speed based on surface factor. Lower speed for rougher surfaces
  // depending on the bicycle type. Modulate speed based on weighted grade
  // (relative measure of elevation change along the edge)
  uint32_t bike_speed =
      edge->dismount() ? kDismountSpeed
                       : static_cast<uint32_t>(
                             (speed_ * surface_speed_factor_[static_cast<uint32_t>(edge->surface())] *
                              kGradeBasedSpeedFactor[edge->weighted_grade()]) +
                             0.5f);

  // Represents how stressful a roadway is without looking at grade or cycle accommodations
  float roadway_stress = 1.0f;
  // Represents the amount of accommodation that is being made for bicycling
  float accommodation_factor = 1.0f;

  // Special use cases: cycleway, footway, and path
  uint32_t road_speed = static_cast<uint32_t>(speed + 0.5f);
  if (edge->use() == Use::kCycleway || edge->use() == Use::kFootway || edge->use() == Use::kPath) {

    // Differentiate how segregated the way is from pedestrians
    if (edge->cyclelane() == CycleLane::kSeparated) { // No pedestrians allowed on path
      accommodation_factor = use_roads_ * 0.8f;
    } else if (edge->cyclelane() == CycleLane::kDedicated) { // Segregated lane from pedestrians
      accommodation_factor = 0.1f + use_roads_ * 0.9f;
    } else { // Share path with pedestrians
      accommodation_factor = 0.2f + use_roads_;
    }
  } else if (edge->use() == Use::kMountainBike && type_ == BicycleType::kMountain) {
    // Slightly less reduction than a footway or path because even with a mountain bike
    // these paths can be a little stressful to ride. No traffic though so still favorable
    accommodation_factor = 0.3f + use_roads_;
  } else if (edge->use() == Use::kLivingStreet) {
    roadway_stress = 0.2f + use_roads_ * 0.8f;
  } else if (edge->use() == Use::kTrack) {
    roadway_stress = 0.5f + use_roads_;
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
    }

    // Penalize roads that have more than one lane (in the direction of travel)
    if (edge->lanecount() > 1) {
      roadway_stress += (static_cast<float>(edge->lanecount()) - 1) * 0.05f * road_factor_;
    }

    // Designated truck routes add to roadway stress
    if (edge->truck_route()) {
      roadway_stress += kTruckStress;
    }

    // Add in penalization for road classification
    roadway_stress += road_factor_ * kRoadClassFactor[static_cast<uint32_t>(edge->classification())];
    // Then multiply by speed so that higher classified roads are more severely punished for being
    // fast.
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
  if (edge->surface() >= minimal_surface_penalized_) {
    surface_factor =
        avoid_bad_surfaces_ * kSurfaceFactors[static_cast<uint32_t>(edge->surface()) -
                                              static_cast<uint32_t>(minimal_surface_penalized_)];
  }

  // Create a final edge factor based on total stress and the weighted grade penalty for the edge.
  float factor = 1.0f + grade_penalty[edge->weighted_grade()] + total_stress + surface_factor;

  // Compute elapsed time based on speed. Modulate cost with weighting factors.
  float sec = (edge->length() * speedfactor_[bike_speed]);
  return {sec * factor, sec};
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost BicycleCost::TransitionCost(const baldr::DirectedEdge* edge,
                                 const baldr::NodeInfo* node,
                                 const EdgeLabel& pred) const {
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = base_transition_cost(node, edge, pred, idx);

  // Accumulate cost and penalty
  float seconds = 0.0f;
  float penalty = 0.0f;
  float class_factor = kRoadClassFactor[static_cast<uint32_t>(edge->classification())];

  // Reduce penalty to make this turn if the road we are turning on has some kind of bicycle
  // accommodation
  float bike_accom = 1.0f;
  if (edge->use() == Use::kCycleway || edge->use() == Use::kFootway || edge->use() == Use::kPath) {
    bike_accom = 0.05f;
    // These uses are classified as "service/other" roads but should not be penalized as such so we
    // change it's factor
    class_factor = 0.1f;
  } else if (edge->use() == Use::kLivingStreet) {
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
    float turn_penalty = (node->drive_on_right())
                             ? kRightSideTurnPenalties[static_cast<uint32_t>(edge->turntype(idx))]
                             : kLeftSideTurnPenalties[static_cast<uint32_t>(edge->turntype(idx))];
    turn_stress += turn_penalty;

    // Take the higher of the turn degree cost and the crossing cost
    float turn_cost = (node->drive_on_right())
                          ? kRightSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))]
                          : kLeftSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))];
    if (turn_cost < kTCCrossing && edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    }

    // Transition time = stopimpact * turncost
    seconds += edge->stopimpact(idx) * turn_cost;
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
  c.cost += (seconds * (turn_stress + 1.0f)) + penalty;
  c.secs += seconds;
  return c;
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// pred is the opposing current edge in the reverse tree
// edge is the opposing predecessor in the reverse tree
Cost BicycleCost::TransitionCostReverse(const uint32_t idx,
                                        const baldr::NodeInfo* node,
                                        const baldr::DirectedEdge* pred,
                                        const baldr::DirectedEdge* edge) const {
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  Cost c = base_transition_cost(node, edge, pred, idx);

  // Additional costs
  float seconds = 0.0f;
  float penalty = 0.0f;

  // Reduce penalty to make this turn if the road we are turning on has some kind of bicycle
  // accommodation
  float class_factor = kRoadClassFactor[static_cast<uint32_t>(edge->classification())];
  float bike_accom = 1.0f;
  if (edge->use() == Use::kCycleway || edge->use() == Use::kFootway || edge->use() == Use::kPath) {
    bike_accom = 0.05f;
    // These uses are considered "service/other" roads but should not be penalized as such so we
    // change it's factor
    class_factor = 0.1f;
  } else if (edge->use() == Use::kLivingStreet) {
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
    float turn_penalty = (node->drive_on_right())
                             ? kRightSideTurnPenalties[static_cast<uint32_t>(edge->turntype(idx))]
                             : kLeftSideTurnPenalties[static_cast<uint32_t>(edge->turntype(idx))];
    turn_stress += turn_penalty;

    // Take the higher of the turn degree cost and the crossing cost
    float turn_cost = (node->drive_on_right())
                          ? kRightSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))]
                          : kLeftSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))];
    if (turn_cost < kTCCrossing && edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    }

    // Transition time = stopimpact * turncost
    seconds += edge->stopimpact(idx) * turn_cost;
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
  c.cost += (seconds * (turn_stress + 1.0f)) + penalty;
  c.secs += seconds;
  return c;
}

void ParseBicycleCostOptions(const rapidjson::Document& doc,
                             const std::string& costing_options_key,
                             CostingOptions* pbf_costing_options) {
  auto json_costing_options = rapidjson::get_child_optional(doc, costing_options_key.c_str());

  if (json_costing_options) {
    // TODO: farm more common stuff out to parent class
    ParseCostOptions(*json_costing_options, pbf_costing_options);

    // If specified, parse json and set pbf values

    // maneuver_penalty
    pbf_costing_options->set_maneuver_penalty(kManeuverPenaltyRange(
        rapidjson::get_optional<float>(*json_costing_options, "/maneuver_penalty")
            .get_value_or(kDefaultManeuverPenalty)));

    // destination_only_penalty
    pbf_costing_options->set_destination_only_penalty(kDestinationOnlyPenaltyRange(
        rapidjson::get_optional<float>(*json_costing_options, "/destination_only_penalty")
            .get_value_or(kDefaultDestinationOnlyPenalty)));

    // alley_penalty
    pbf_costing_options->set_alley_penalty(
        kAlleyPenaltyRange(rapidjson::get_optional<float>(*json_costing_options, "/alley_penalty")
                               .get_value_or(kDefaultAlleyPenalty)));

    // gate_cost
    pbf_costing_options->set_gate_cost(
        kGateCostRange(rapidjson::get_optional<float>(*json_costing_options, "/gate_cost")
                           .get_value_or(kDefaultGateCost)));

    // gate_penalty
    pbf_costing_options->set_gate_penalty(
        kGatePenaltyRange(rapidjson::get_optional<float>(*json_costing_options, "/gate_penalty")
                              .get_value_or(kDefaultGatePenalty)));

    // ferry_cost
    pbf_costing_options->set_ferry_cost(
        kFerryCostRange(rapidjson::get_optional<float>(*json_costing_options, "/ferry_cost")
                            .get_value_or(kDefaultFerryCost)));

    // country_crossing_cost
    pbf_costing_options->set_country_crossing_cost(kCountryCrossingCostRange(
        rapidjson::get_optional<float>(*json_costing_options, "/country_crossing_cost")
            .get_value_or(kDefaultCountryCrossingCost)));

    // country_crossing_penalty
    pbf_costing_options->set_country_crossing_penalty(kCountryCrossingPenaltyRange(
        rapidjson::get_optional<float>(*json_costing_options, "/country_crossing_penalty")
            .get_value_or(kDefaultCountryCrossingPenalty)));

    // use_roads
    pbf_costing_options->set_use_roads(
        kUseRoadRange(rapidjson::get_optional<float>(*json_costing_options, "/use_roads")
                          .get_value_or(kDefaultUseRoad)));

    // use_hills
    pbf_costing_options->set_use_hills(
        kUseHillsRange(rapidjson::get_optional<float>(*json_costing_options, "/use_hills")
                           .get_value_or(kDefaultUseHills)));

    // use_ferry
    pbf_costing_options->set_use_ferry(
        kUseFerryRange(rapidjson::get_optional<float>(*json_costing_options, "/use_ferry")
                           .get_value_or(kDefaultUseFerry)));

    // avoid_bad_surfaces
    pbf_costing_options->set_avoid_bad_surfaces(kAvoidBadSurfacesRange(
        rapidjson::get_optional<float>(*json_costing_options, "/avoid_bad_surfaces")
            .get_value_or(kDefaultAvoidBadSurfaces)));

    // bicycle_type
    pbf_costing_options->set_transport_type(
        rapidjson::get_optional<std::string>(*json_costing_options, "/bicycle_type")
            .get_value_or(kDefaultBicycleType));

    // convert string to enum, set ranges and defaults based on enum
    BicycleType type;
    if (pbf_costing_options->transport_type() == "Cross") {
      type = BicycleType::kCross;
    } else if (pbf_costing_options->transport_type() == "Road") {
      type = BicycleType::kRoad;
    } else if (pbf_costing_options->transport_type() == "Mountain") {
      type = BicycleType::kMountain;
    } else {
      type = BicycleType::kHybrid;
    }

    // This is the average speed on smooth, flat roads. If not present or outside the
    // valid range use a default speed based on the bicycle type.
    uint32_t t = static_cast<uint32_t>(type);
    ranged_default_t<float> kCycleSpeedRange{kMinCyclingSpeed, kDefaultCyclingSpeed[t],
                                             kMaxCyclingSpeed};

    // Set type specific defaults, override with URL inputs
    // cycling_speed
    pbf_costing_options->set_cycling_speed(
        kCycleSpeedRange(rapidjson::get_optional<float>(*json_costing_options, "/cycling_speed")
                             .get_value_or(kDefaultCyclingSpeed[t])));

  } else {
    // Set pbf values to defaults
    pbf_costing_options->set_maneuver_penalty(kDefaultManeuverPenalty);
    pbf_costing_options->set_destination_only_penalty(kDefaultDestinationOnlyPenalty);
    pbf_costing_options->set_alley_penalty(kDefaultAlleyPenalty);
    pbf_costing_options->set_gate_cost(kDefaultGateCost);
    pbf_costing_options->set_gate_penalty(kDefaultGatePenalty);
    pbf_costing_options->set_ferry_cost(kDefaultFerryCost);
    pbf_costing_options->set_country_crossing_cost(kDefaultCountryCrossingCost);
    pbf_costing_options->set_country_crossing_penalty(kDefaultCountryCrossingPenalty);
    pbf_costing_options->set_use_roads(kDefaultUseRoad);
    pbf_costing_options->set_use_hills(kDefaultUseHills);
    pbf_costing_options->set_use_ferry(kDefaultUseFerry);
    pbf_costing_options->set_avoid_bad_surfaces(kDefaultAvoidBadSurfaces);
    pbf_costing_options->set_transport_type(kDefaultBicycleType);
    pbf_costing_options->set_cycling_speed(
        kDefaultCyclingSpeed[static_cast<uint32_t>(BicycleType::kHybrid)]);
    pbf_costing_options->set_flow_mask(kDefaultFlowMask);
  }
}

cost_ptr_t CreateBicycleCost(const Costing costing, const Options& options) {
  return std::make_shared<BicycleCost>(costing, options);
}

} // namespace sif
} // namespace valhalla

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

class TestBicycleCost : public BicycleCost {
public:
  TestBicycleCost(const Costing costing, const Options& options) : BicycleCost(costing, options){};

  using BicycleCost::alley_penalty_;
  using BicycleCost::country_crossing_cost_;
  using BicycleCost::destination_only_penalty_;
  using BicycleCost::ferry_transition_cost_;
  using BicycleCost::gate_cost_;
  using BicycleCost::maneuver_penalty_;
};

TestBicycleCost* make_bicyclecost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({"costing_options":{"bicycle":{")" << property << R"(":)" << testVal << "}}}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return new TestBicycleCost(valhalla::Costing::bicycle, request.options());
}

std::uniform_real_distribution<float>*
make_distributor_from_range(const ranged_default_t<float>& range) {
  float rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<float>(range.min - rangeLength, range.max + rangeLength);
}

void testBicycleCostParams() {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> distributor;
  std::shared_ptr<TestBicycleCost> ctorTester;

  // maneuver_penalty_
  distributor.reset(make_distributor_from_range(kManeuverPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("maneuver_penalty", (*distributor)(generator)));
    if (ctorTester->maneuver_penalty_ < kManeuverPenaltyRange.min ||
        ctorTester->maneuver_penalty_ > kManeuverPenaltyRange.max) {
      throw std::runtime_error("maneuver_penalty_ is not within it's range");
    }
  }

  // alley_penalty_
  distributor.reset(make_distributor_from_range(kAlleyPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("alley_penalty", (*distributor)(generator)));
    if (ctorTester->alley_penalty_ < kAlleyPenaltyRange.min ||
        ctorTester->alley_penalty_ > kAlleyPenaltyRange.max) {
      throw std::runtime_error("alley_penalty_ is not within it's range");
    }
  }

  // destination_only_penalty_
  distributor.reset(make_distributor_from_range(kDestinationOnlyPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_bicyclecost_from_json("destination_only_penalty", (*distributor)(generator)));
    if (ctorTester->destination_only_penalty_ < kDestinationOnlyPenaltyRange.min ||
        ctorTester->destination_only_penalty_ > kDestinationOnlyPenaltyRange.max) {
      throw std::runtime_error("destination_only_penalty_ is not within it's range");
    }
  }

  // gate_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(kGateCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("gate_cost", (*distributor)(generator)));
    if (ctorTester->gate_cost_.secs < kGateCostRange.min ||
        ctorTester->gate_cost_.secs > kGateCostRange.max) {
      throw std::runtime_error("gate_cost_ is not within it's range");
    }
  }

  // gate_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(kGatePenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("gate_penalty", (*distributor)(generator)));
    if (ctorTester->gate_cost_.cost < kGatePenaltyRange.min ||
        ctorTester->gate_cost_.cost > kGatePenaltyRange.max + kDefaultGateCost) {
      throw std::runtime_error("gate_penalty_ is not within it's range");
    }
  }

  // country_crossing_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(kCountryCrossingCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("country_crossing_cost", (*distributor)(generator)));
    if (ctorTester->country_crossing_cost_.secs < kCountryCrossingCostRange.min ||
        ctorTester->country_crossing_cost_.secs > kCountryCrossingCostRange.max) {
      throw std::runtime_error("country_crossing_cost_ is not within it's range");
    }
  }

  // country_crossing_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(kCountryCrossingPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_bicyclecost_from_json("country_crossing_penalty", (*distributor)(generator)));
    if (ctorTester->country_crossing_cost_.cost < kCountryCrossingPenaltyRange.min ||
        ctorTester->country_crossing_cost_.cost >
            kCountryCrossingPenaltyRange.max + kDefaultCountryCrossingCost) {
      throw std::runtime_error("country_crossing_penalty_ is not within it's range");
    }
  }

  // ferry_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(kFerryCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("ferry_cost", (*distributor)(generator)));
    if (ctorTester->ferry_transition_cost_.secs < kFerryCostRange.min ||
        ctorTester->ferry_transition_cost_.secs > kFerryCostRange.max) {
      throw std::runtime_error("ferry_cost_ is not within it's range");
    }
  }

  /**
   // use_ferry_
   distributor.reset(make_distributor_from_range(kUseFerryRange));
   for (unsigned i = 0; i < testIterations; ++i) {
     ctorTester.reset(make_bicyclecost_from_json("use_ferry", (*distributor)(generator)));
     if (ctorTester->use_ferry_ < kUseFerryRange.min || ctorTester->use_ferry_ > kUseFerryRange.max) {
       throw std::runtime_error("use_ferry_ is not within it's range");
     }
   }

   // use_hills
   distributor.reset(make_distributor_from_range(kUseHillsRange));
   for (unsigned i = 0; i < testIterations; ++i) {
     ctorTester.reset(make_bicyclecost_from_json("use_hills", (*distributor)(generator)));
     if (ctorTester->use_hills < kUseHillsRange.min || ctorTester->use_hills > kUseHillsRange.max) {
       throw std::runtime_error("use_hills is not within it's range");
     }
   }
   */

  // use_roads_
  distributor.reset(make_distributor_from_range(kUseRoadRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("use_roads", (*distributor)(generator)));
    if (ctorTester->use_roads_ < kUseRoadRange.min || ctorTester->use_roads_ > kUseRoadRange.max) {
      throw std::runtime_error("use_roads_ is not within it's range");
    }
  }

  // speed_
  constexpr ranged_default_t<float> kRoadCyclingSpeedRange{kMinCyclingSpeed, kDefaultCyclingSpeed[0],
                                                           kMaxCyclingSpeed};
  distributor.reset(make_distributor_from_range(kRoadCyclingSpeedRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("cycling_speed", (*distributor)(generator)));
    if (ctorTester->speed_ < kRoadCyclingSpeedRange.min ||
        ctorTester->speed_ > kRoadCyclingSpeedRange.max) {
      throw std::runtime_error("speed_ is not within it's range");
    }
  }
}
} // namespace

int main() {
  test::suite suite("costing");

  suite.test(TEST_CASE(testBicycleCostParams));

  return suite.tear_down();
}

#endif
