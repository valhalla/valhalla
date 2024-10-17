#include "sif/bicyclecost.h"
#include "baldr/accessrestriction.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "midgard/constants.h"
#include "midgard/util.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
#include <cassert>

#ifdef INLINE_TEST
#include "test.h"
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
constexpr float kDefaultAlleyPenalty = 60.0f; // Seconds
constexpr float kDefaultGatePenalty = 300.0f; // Seconds
constexpr float kDefaultBssCost = 120.0f;     // Seconds
constexpr float kDefaultBssPenalty = 0.0f;    // Seconds

// Other options
constexpr float kDefaultUseRoad = 0.25f;          // Factor between 0 and 1
constexpr float kDefaultAvoidBadSurfaces = 0.25f; // Factor between 0 and 1
constexpr float kDefaultUseLivingStreets = 0.5f;  // Factor between 0 and 1
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

// Cycle lane transition costing factors
constexpr float kCycleLaneTransitionFactor[] = {
    1.0f,  // No shoulder or cycle lane
    0.5f,  // No shoulder, shared cycle lane
    0.25f, // No shoulder, dedicated cycle lane
    0.1f,  // No shoulder, separated cycle lane
    0.4f,  // Shoulder, no cycle lane
    0.5f,  // Shoulder, shared cycle lane
    0.25f, // Shoulder, dedicated cycle lane
    0.1f   // Shoulder, separated cycle lane
};

// Factor when transitioning onto a living street
constexpr float kLivingStreetTransitionFactor = 0.15f;

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
constexpr ranged_default_t<float> kUseRoadRange{0.0f, kDefaultUseRoad, 1.0f};
constexpr ranged_default_t<float> kUseHillsRange{0.0f, kDefaultUseHills, 1.0f};
constexpr ranged_default_t<float> kAvoidBadSurfacesRange{0.0f, kDefaultAvoidBadSurfaces, 1.0f};

constexpr ranged_default_t<float> kBSSCostRange{0, kDefaultBssCost, kMaxPenalty};
constexpr ranged_default_t<float> kBSSPenaltyRange{0, kDefaultBssPenalty, kMaxPenalty};

BaseCostingOptionsConfig GetBaseCostOptsConfig() {
  BaseCostingOptionsConfig cfg{};
  // override defaults
  cfg.alley_penalty_.def = kDefaultAlleyPenalty;
  cfg.gate_penalty_.def = kDefaultGatePenalty;
  cfg.disable_toll_booth_ = true;
  cfg.disable_rail_ferry_ = true;
  cfg.use_living_streets_.def = kDefaultUseLivingStreets;
  return cfg;
}

const BaseCostingOptionsConfig kBaseCostOptsConfig = GetBaseCostOptsConfig();
} // namespace

/**
 * Derived class providing dynamic edge costing for bicycle routes.
 */
class BicycleCost : public DynamicCost {
public:
  /**
   * Construct bicycle costing. Pass in cost type and costing_options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  costing_options pbf with request costing_options.
   */
  BicycleCost(const Costing& costing_options);

  // virtual destructor
  virtual ~BicycleCost() {
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge           Pointer to a directed edge.
   * @param  is_dest        Is a directed edge the destination?
   * @param  pred           Predecessor edge information.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the directed edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const bool is_dest,
                       const EdgeLabel& pred,
                       const graph_tile_ptr& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       uint8_t& restriction_idx) const override;

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
                              const graph_tile_ptr& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index,
                              uint8_t& restriction_idx) const override;

  /**
   * Only transit costings are valid for this method call, hence we throw
   * @param edge
   * @param departure
   * @param curr_time
   * @return
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge*,
                        const baldr::TransitDeparture*,
                        const uint32_t) const override {
    throw std::runtime_error("BicycleCost::EdgeCost does not support transit edges");
  }

  bool IsClosed(const baldr::DirectedEdge*, const graph_tile_ptr&) const override {
    return false;
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge       Pointer to a directed edge.
   * @param   tile       Current tile.
   * @param   time_info  Time info about edge passing.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const graph_tile_ptr&,
                        const baldr::TimeInfo&,
                        uint8_t&) const override;

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
                              const EdgeLabel& pred) const override;

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * @param  idx   Directed edge local index
   * @param  node  Node (intersection) where transition occurs.
   * @param  pred  the opposing current edge in the reverse tree.
   * @param  edge  the opposing predecessor in the reverse tree
   * @param  has_measured_speed Do we have any of the measured speed types set?
   * @param  internal_turn  Did we make an turn on a short internal edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* pred,
                                     const baldr::DirectedEdge* edge,
                                     const bool /*has_measured_speed*/,
                                     const InternalTurn /*internal_turn*/) const override;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const override {
    // Assume max speed of 2 * the average speed set for costing
    return speedfactor_[static_cast<uint32_t>(2 * speed_)];
  }

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const override {
    return static_cast<uint8_t>(type_);
  }

  virtual Cost BSSCost() const override {
    return {kDefaultBssCost, kDefaultBssPenalty};
  };

  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes

  std::vector<float> speedfactor_; // Cost factors based on speed in kph
  float use_roads_;                // Preference of using roads between 0 and 1
  float avoid_roads_;              // Inverse of use roads
  float road_factor_;              // Road factor based on use_roads_
  float sidepath_factor_;          // Factor to use when use_sidepath is set on an edge
  float livingstreet_factor_;      // Factor to use for living streets
  float track_factor_;             // Factor to use tracks
  float avoid_bad_surfaces_;       // Preference of avoiding bad surfaces for the bike type

  // Average speed (kph) on smooth, flat roads.
  float speed_;

  // Bicycle type
  BicycleType type_;

  // Minimal surface type that will be penalized for costing
  Surface minimal_surface_penalized_;
  Surface worst_allowed_surface_;

  // Cycle lane accommodation factors
  float cyclelane_factor_[8];
  float path_cyclelane_factor_[4];

  // Surface speed factors (based on road surface type).
  const float* surface_speed_factor_;

  // Road speed penalty factor. Penalties apply above a threshold (based on the use_roads factor)
  float speedpenalty_[kMaxSpeedKph + 1];
  uint32_t speed_penalty_threshold_;

  // Elevation/grade penalty (weighting applied based on the edge's weighted
  // grade (relative value from 0-15)
  float grade_penalty[16];

protected:
  /**
   * Function to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. It's also used to filter
   * edges not usable / inaccessible by bicycle.
   */
  bool Allowed(const baldr::DirectedEdge* edge,
               const graph_tile_ptr& tile,
               uint16_t disallow_mask = kDisallowNone) const override {
    return DynamicCost::Allowed(edge, tile, disallow_mask) && !edge->bss_connection() &&
           edge->use() != Use::kSteps &&
           (avoid_bad_surfaces_ != 1.0f || edge->surface() <= worst_allowed_surface_);
  }
};

// Bicycle route costs are distance based with some favor/avoid based on
// attribution. Speed is derived based on bicycle type or user input and
// is modulated based on surface type and grade factors.

// Constructor
BicycleCost::BicycleCost(const Costing& costing)
    : DynamicCost(costing, TravelMode::kBicycle, kBicycleAccess) {
  const auto& costing_options = costing.options();

  // Set hierarchy to allow unlimited transitions
  for (auto& h : hierarchy_limits_) {
    h.max_up_transitions = kUnlimitedTransitions;
  }

  // Get the base costs
  get_base_costs(costing);

  // Get the bicycle type - enter as string and convert to enum
  const std::string& bicycle_type = costing_options.transport_type();
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
  avoid_roads_ = 1.0f - use_roads_;

  // Set the road classification factor. use_roads factors above 0.5 start to
  // reduce the weight difference between road classes while factors below 0.5
  // start to increase the differences.
  road_factor_ = (use_roads_ >= 0.5f) ? 1.5f - use_roads_ : 2.0f - use_roads_ * 2.0f;

  // Set edge costing factors
  sidepath_factor_ = 3.0f * (1.0f - use_roads_);
  livingstreet_factor_ = 0.2f + use_roads_ * 0.8f;
  track_factor_ = 0.5f + use_roads_;
  cyclelane_factor_[0] = 1.0f;                          // No shoulder or cycle lane
  cyclelane_factor_[1] = 0.9f + use_roads_ * 0.05f;     // No shoulder, shared cycle lane
  cyclelane_factor_[2] = 0.4f + use_roads_ * 0.45f;     // No shoulder, dedicated cycle lane
  cyclelane_factor_[3] = 0.15f + use_roads_ * 0.6f;     // No shoulder, separated cycle lane
  cyclelane_factor_[4] = 0.7f + use_roads_ * 0.2f;      // Shoulder, no cycle lane
  cyclelane_factor_[5] = 0.9f + use_roads_ * 0.05f;     // Shoulder, shared cycle lane
  cyclelane_factor_[6] = 0.4f + use_roads_ * 0.45f;     // Shoulder, dedicated cycle lane
  cyclelane_factor_[7] = 0.15f + use_roads_ * 0.6f;     // Shoulder, separated cycle lane
  path_cyclelane_factor_[0] = 0.2f + use_roads_;        // Share path with pedestrians
  path_cyclelane_factor_[1] = 0.2f + use_roads_;        // Share path with pedestrians
  path_cyclelane_factor_[2] = 0.1f + use_roads_ * 0.9f; // Segregated lane from pedestrians
  path_cyclelane_factor_[3] = use_roads_ * 0.8f;        // No pedestrians allowed on path

  // Set the speed penalty threshold and factor. With useroads = 1 the
  // threshold is 70 kph (near 50 MPH).
  speed_penalty_threshold_ = kSpeedPenaltyThreshold + static_cast<uint32_t>(use_roads_ * 30.0f);

  // Create speed cost table and penalty table (to avoid division in costing)
  float avoid_roads = (1.0f - use_roads_) * 0.75f + 0.25;
  speedfactor_.resize(kMaxSpeedKph + 1, 0);
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
                          const bool is_dest,
                          const EdgeLabel& pred,
                          const graph_tile_ptr& tile,
                          const baldr::GraphId& edgeid,
                          const uint64_t current_time,
                          const uint32_t tz_index,
                          uint8_t& restriction_idx) const {
  // Check bicycle access and turn restrictions. Bicycles should obey
  // vehicular turn restrictions. Allow Uturns at dead ends only.
  // Skip impassable edges and shortcut edges.
  if (!IsAccessible(edge) || edge->is_shortcut() ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx() &&
       pred.mode() == TravelMode::kBicycle) ||
      (!ignore_turn_restrictions_ && (pred.restrictions() & (1 << edge->localedgeidx()))) ||
      IsUserAvoidEdge(edgeid) || CheckExclusions(edge, pred)) {
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
  return DynamicCost::EvaluateRestrictions(access_mask_, edge, is_dest, tile, edgeid, current_time,
                                           tz_index, restriction_idx);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool BicycleCost::AllowedReverse(const baldr::DirectedEdge* edge,
                                 const EdgeLabel& pred,
                                 const baldr::DirectedEdge* opp_edge,
                                 const graph_tile_ptr& tile,
                                 const baldr::GraphId& opp_edgeid,
                                 const uint64_t current_time,
                                 const uint32_t tz_index,
                                 uint8_t& restriction_idx) const {
  // Check access, U-turn (allow at dead-ends), and simple turn restriction.
  // Do not allow transit connection edges.
  if (!IsAccessible(opp_edge) || opp_edge->is_shortcut() ||
      opp_edge->use() == Use::kTransitConnection || opp_edge->use() == Use::kEgressConnection ||
      opp_edge->use() == Use::kPlatformConnection ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx() &&
       pred.mode() == TravelMode::kBicycle) ||
      (!ignore_turn_restrictions_ && (opp_edge->restrictions() & (1 << pred.opp_local_idx()))) ||
      IsUserAvoidEdge(opp_edgeid) || CheckExclusions(opp_edge, pred)) {
    return false;
  }

  // Prohibit certain roads based on surface type and bicycle type
  if (edge->surface() > worst_allowed_surface_) {
    return false;
  }
  return DynamicCost::EvaluateRestrictions(access_mask_, opp_edge, false, tile, opp_edgeid,
                                           current_time, tz_index, restriction_idx);
}

// Returns the cost to traverse the edge and an estimate of the actual time
// (in seconds) to traverse the edge.
Cost BicycleCost::EdgeCost(const baldr::DirectedEdge* edge,
                           const graph_tile_ptr&,
                           const baldr::TimeInfo&,
                           uint8_t&) const {
  // Stairs/steps - high cost (travel speed = 1kph) so they are generally avoided.
  if (edge->use() == Use::kSteps) {
    float sec = (edge->length() * speedfactor_[1]);
    return {shortest_ ? edge->length() : sec * kBicycleStepsFactor, sec};
  }

  // Ferries are a special case - they use the ferry speed (stored on the edge)
  if (edge->use() == Use::kFerry) {
    // Compute elapsed time based on speed. Modulate cost with weighting factors.
    assert(edge->speed() < speedfactor_.size());
    float sec = (edge->length() * speedfactor_[edge->speed()]);
    return {shortest_ ? edge->length() : sec * ferry_factor_, sec};
  }

  // Represents how stressful a roadway is without looking at grade or cycle accommodations
  float roadway_stress = 1.0f;

  // Represents the amount of accommodation that is being made for bicycling
  float accommodation_factor = 1.0f;

  // Special use cases: cycleway, footway, path, living street, track
  if (edge->use() == Use::kCycleway || edge->use() == Use::kFootway || edge->use() == Use::kPath) {
    // Differentiate how segregated the cycleway/path is from pedestrians
    accommodation_factor = path_cyclelane_factor_[static_cast<uint32_t>(edge->cyclelane())];
  } else if (edge->use() == Use::kMountainBike && type_ == BicycleType::kMountain) {
    // Slightly less reduction than a footway or path because even with a mountain bike
    // these paths can be a little stressful to ride. No traffic though so still favorable
    accommodation_factor = 0.3f + use_roads_;
  } else if (edge->use() == Use::kLivingStreet) {
    roadway_stress = livingstreet_factor_;
  } else if (edge->use() == Use::kTrack) {
    roadway_stress = track_factor_;
  } else {
    // Favor roads where a cycle lane and/or shoulder exists
    accommodation_factor =
        cyclelane_factor_[edge->shoulder() * 4 + static_cast<uint32_t>(edge->cyclelane())];

    // Penalize roads that have more than one lane (in the direction of travel)
    if (edge->lanecount() > 1) {
      roadway_stress += (static_cast<float>(edge->lanecount()) - 1) * 0.05f * road_factor_;
    }

    // Designated truck routes add to roadway stress
    if (edge->truck_route()) {
      roadway_stress += kTruckStress;
    }

    // Add in penalization for road classification (higher class roads are more stress)
    roadway_stress += road_factor_ * kRoadClassFactor[static_cast<uint32_t>(edge->classification())];

    // Multiply by speed so that higher classified roads are more severely punished for being fast.
    // Use the speed assigned to the directed edge. Even if we had traffic information we shouldn't
    // use it here. High speed penalized edges so we want the "default" speed rather than a traffic
    // influenced speed anyway.
    roadway_stress *= speedpenalty_[edge->speed()];
  }

  // We want to try and avoid roads that specify to use a cycling path to the side
  if (edge->use_sidepath()) {
    accommodation_factor += sidepath_factor_;
  }

  // Favor bicycle networks slightly
  if (edge->bike_network()) {
    accommodation_factor *= kBicycleNetworkFactor;
  }

  // Create an edge factor based on total stress (sum of accommodation factor and roadway
  // stress) and the weighted grade penalty for the edge.
  float factor =
      1.0f + grade_penalty[edge->weighted_grade()] + (accommodation_factor * roadway_stress);

  // If surface is worse than the minimum we add a surface factor
  if (edge->surface() >= minimal_surface_penalized_) {
    factor +=
        avoid_bad_surfaces_ * kSurfaceFactors[static_cast<uint32_t>(edge->surface()) -
                                              static_cast<uint32_t>(minimal_surface_penalized_)];
  }

  // Compute bicycle speed. If you have to dismount on the edge then set speed to an average
  // walking speed. Otherwise, set speed based on surface factor and grade. Lower bike speed
  // for rougher surfaces (amount depends on on the bicycle type). Weighted grade (relative
  // measure of elevation change along the edge) modulates speed based on elevation changes.
  uint32_t bike_speed =
      edge->dismount() ? kDismountSpeed
                       : static_cast<uint32_t>(
                             (speed_ * surface_speed_factor_[static_cast<uint32_t>(edge->surface())] *
                              kGradeBasedSpeedFactor[edge->weighted_grade()]) +
                             0.5f);

  // Compute elapsed time based on speed. Modulate cost with weighting factors.
  float sec = (edge->length() * speedfactor_[bike_speed]);
  return {shortest_ ? edge->length() : sec * factor, sec};
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost BicycleCost::TransitionCost(const baldr::DirectedEdge* edge,
                                 const baldr::NodeInfo* node,
                                 const EdgeLabel& pred) const {
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = base_transition_cost(node, edge, &pred, idx);

  // Reduce penalty to make this turn if the road we are turning on has some kind of bicycle
  // accommodation
  float class_factor = kRoadClassFactor[static_cast<uint32_t>(edge->classification())];
  float bike_accom = 1.0f;
  if (edge->use() == Use::kCycleway || edge->use() == Use::kFootway || edge->use() == Use::kPath) {
    bike_accom = 0.05f;
    // These uses are classified as "service/other" roads but should not be penalized as such so we
    // change it's factor
    class_factor = 0.1f;
  } else if (edge->use() == Use::kLivingStreet) {
    bike_accom = kLivingStreetTransitionFactor;
  } else {
    bike_accom =
        kCycleLaneTransitionFactor[edge->shoulder() * 4 + static_cast<uint32_t>(edge->cyclelane())];
  }

  float seconds = 0.0f;
  float turn_stress = 1.0f;
  if (edge->stopimpact(idx) > 0) {
    // Increase turn stress depending on the kind of turn that has to be made.
    uint32_t turn_type = static_cast<uint32_t>(edge->turntype(idx));
    float turn_penalty = (node->drive_on_right()) ? kRightSideTurnPenalties[turn_type]
                                                  : kLeftSideTurnPenalties[turn_type];
    turn_stress += turn_penalty;

    // Take the higher of the turn degree cost and the crossing cost
    float turn_cost =
        (node->drive_on_right()) ? kRightSideTurnCosts[turn_type] : kLeftSideTurnCosts[turn_type];
    if (turn_cost < kTCCrossing && edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    }

    // Transition time = stopimpact * turncost
    seconds += edge->stopimpact(idx) * turn_cost;
  }

  // Reduce stress by road class factor the closer use_roads_ is to 0
  turn_stress *= (class_factor * avoid_roads_) + use_roads_ + 1.0f;

  // Penalize transition to higher class road.
  float penalty = 0.0f;
  if (edge->classification() < pred.classification() && edge->use() != Use::kLivingStreet) {
    penalty += 10.0f * (static_cast<uint32_t>(pred.classification()) -
                        static_cast<uint32_t>(edge->classification()));
    // Reduce the turn stress if there is a traffic signal
    turn_stress += (node->traffic_signal()) ? 0.4 : 1.0;

    // Reduce penalty by bike_accom the closer use_roads_ is to 0
    penalty *= (bike_accom * avoid_roads_) + use_roads_;
  }

  // Return cost (time and penalty)
  c.cost += shortest_ ? 0 : seconds * (turn_stress + 1.0f) + penalty;
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
                                        const baldr::DirectedEdge* edge,
                                        const bool /*has_measured_speed*/,
                                        const InternalTurn /*internal_turn*/) const {

  // Bicycles should be able to make uturns on short internal edges; therefore, InternalTurn
  // is ignored for now.
  // TODO: do we want to update the cost if we have flow or speed from traffic.

  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  Cost c = base_transition_cost(node, edge, pred, idx);

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
    bike_accom = kLivingStreetTransitionFactor;
  } else {
    bike_accom =
        kCycleLaneTransitionFactor[edge->shoulder() * 4 + static_cast<uint32_t>(edge->cyclelane())];
  }

  float seconds = 0.0f;
  float turn_stress = 1.0f;
  if (edge->stopimpact(idx) > 0) {
    // Increase turn stress depending on the kind of turn that has to be made.
    uint32_t turn_type = static_cast<uint32_t>(edge->turntype(idx));
    float turn_penalty = (node->drive_on_right()) ? kRightSideTurnPenalties[turn_type]
                                                  : kLeftSideTurnPenalties[turn_type];
    turn_stress += turn_penalty;

    // Take the higher of the turn degree cost and the crossing cost
    float turn_cost =
        (node->drive_on_right()) ? kRightSideTurnCosts[turn_type] : kLeftSideTurnCosts[turn_type];
    if (turn_cost < kTCCrossing && edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    }

    // Transition time = stopimpact * turncost
    seconds += edge->stopimpact(idx) * turn_cost;
  }

  // Reduce stress by road class factor the closer use_roads_ is to 0
  turn_stress *= (class_factor * avoid_roads_) + use_roads_ + 1.0f;

  // Penalize transition to higher class road.
  float penalty = 0.0f;
  if (edge->classification() < pred->classification() && edge->use() != Use::kLivingStreet) {
    penalty += 10.0f * (static_cast<uint32_t>(pred->classification()) -
                        static_cast<uint32_t>(edge->classification()));
    // Reduce the turn stress if there is a traffic signal
    turn_stress += (node->traffic_signal()) ? 0.4 : 1.0;

    // Reduce penalty by bike_accom the closer use_roads_ is to 0
    penalty *= (bike_accom * avoid_roads_) + use_roads_;
  }

  // Return cost (time and penalty)
  c.cost += shortest_ ? 0.f : seconds * (turn_stress + 1.0f) + penalty;
  c.secs += seconds;
  return c;
}

void ParseBicycleCostOptions(const rapidjson::Document& doc,
                             const std::string& costing_options_key,
                             Costing* c) {
  c->set_type(Costing::bicycle);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  ParseBaseCostOptions(json, c, kBaseCostOptsConfig);
  JSON_PBF_RANGED_DEFAULT(co, kUseRoadRange, json, "/use_roads", use_roads);
  JSON_PBF_RANGED_DEFAULT(co, kUseHillsRange, json, "/use_hills", use_hills);
  JSON_PBF_RANGED_DEFAULT(co, kAvoidBadSurfacesRange, json, "/avoid_bad_surfaces",
                          avoid_bad_surfaces);
  JSON_PBF_DEFAULT(co, kDefaultBicycleType, json, "/bicycle_type", transport_type);

  // convert string to enum, set ranges and defaults based on enum
  BicycleType type;
  if (co->transport_type() == "Cross") {
    type = BicycleType::kCross;
  } else if (co->transport_type() == "Road") {
    type = BicycleType::kRoad;
  } else if (co->transport_type() == "Mountain") {
    type = BicycleType::kMountain;
  } else {
    type = BicycleType::kHybrid;
  }

  // This is the average speed on smooth, flat roads. If not present or outside the
  // valid range use a default speed based on the bicycle type.
  uint32_t t = static_cast<uint32_t>(type);
  ranged_default_t<float> kCycleSpeedRange{kMinCyclingSpeed, kDefaultCyclingSpeed[t],
                                           kMaxCyclingSpeed};

  JSON_PBF_RANGED_DEFAULT(co, kCycleSpeedRange, json, "/cycling_speed", cycling_speed);
  JSON_PBF_RANGED_DEFAULT(co, kBSSCostRange, json, "/bss_return_cost", bike_share_cost);
  JSON_PBF_RANGED_DEFAULT(co, kBSSPenaltyRange, json, "/bss_return_penalty", bike_share_penalty);
}

cost_ptr_t CreateBicycleCost(const Costing& costing_options) {
  return std::make_shared<BicycleCost>(costing_options);
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
  TestBicycleCost(const Costing& costing_options) : BicycleCost(costing_options){};

  using BicycleCost::alley_penalty_;
  using BicycleCost::country_crossing_cost_;
  using BicycleCost::destination_only_penalty_;
  using BicycleCost::ferry_transition_cost_;
  using BicycleCost::gate_cost_;
  using BicycleCost::maneuver_penalty_;
  using BicycleCost::service_penalty_;
};

TestBicycleCost* make_bicyclecost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({"costing": "bicycle", "costing_options":{"bicycle":{")" << property << R"(":)" << testVal
     << "}}}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return new TestBicycleCost(request.options().costings().find(Costing::bicycle)->second);
}

std::uniform_real_distribution<float>*
make_distributor_from_range(const ranged_default_t<float>& range) {
  float rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<float>(range.min - rangeLength, range.max + rangeLength);
}

TEST(BicycleCost, testBicycleCostParams) {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> distributor;
  std::shared_ptr<TestBicycleCost> ctorTester;

  const auto& defaults = kBaseCostOptsConfig;

  // maneuver_penalty_
  distributor.reset(make_distributor_from_range(defaults.maneuver_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("maneuver_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->maneuver_penalty_,
                test::IsBetween(ctorTester->maneuver_penalty_, defaults.maneuver_penalty_.max));
  }

  // alley_penalty_
  distributor.reset(make_distributor_from_range(defaults.alley_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("alley_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->alley_penalty_,
                test::IsBetween(defaults.alley_penalty_.min, defaults.alley_penalty_.max));
  }

  // service_penalty_
  distributor.reset(make_distributor_from_range(defaults.service_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("service_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->service_penalty_,
                test::IsBetween(defaults.service_penalty_.min, defaults.service_penalty_.max));
  }

  // destination_only_penalty_
  distributor.reset(make_distributor_from_range(defaults.dest_only_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_bicyclecost_from_json("destination_only_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->destination_only_penalty_,
                test::IsBetween(defaults.dest_only_penalty_.min, defaults.dest_only_penalty_.max));
  }

  // gate_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.gate_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("gate_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.secs,
                test::IsBetween(defaults.gate_cost_.min, defaults.gate_cost_.max));
  }

  // gate_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.gate_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("gate_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.cost,
                test::IsBetween(defaults.gate_penalty_.min, defaults.gate_penalty_.max));
  }

  // country_crossing_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.country_crossing_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("country_crossing_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.secs,
                test::IsBetween(defaults.country_crossing_cost_.min,
                                defaults.country_crossing_cost_.max));
  }

  // country_crossing_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.country_crossing_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_bicyclecost_from_json("country_crossing_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.cost,
                test::IsBetween(defaults.country_crossing_penalty_.min,
                                defaults.country_crossing_penalty_.max +
                                    defaults.country_crossing_cost_.def));
  }

  // ferry_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.ferry_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("ferry_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->ferry_transition_cost_.secs,
                test::IsBetween(defaults.ferry_cost_.min, defaults.ferry_cost_.max));
  }

  /**
   // use_ferry_
   distributor.reset(make_distributor_from_range(defaults.use_ferry_));
   for (unsigned i = 0; i < testIterations; ++i) {
     ctorTester.reset(make_bicyclecost_from_json("use_ferry", (*distributor)(generator)));
EXPECT_THAT(ctorTester->use_ferry_ , test::IsBetween(defaults.use_ferry_.min,
defaults.use_ferry_.max));
   }

   // use_hills
   distributor.reset(make_distributor_from_range(kUseHillsRange));
   for (unsigned i = 0; i < testIterations; ++i) {
     ctorTester.reset(make_bicyclecost_from_json("use_hills", (*distributor)(generator)));
     EXPECT_THAT(ctorTester->use_hills , test::IsBetween( kUseHillsRange.min ,kUseHillsRange.max));
   }
   */

  // use_roads_
  distributor.reset(make_distributor_from_range(kUseRoadRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("use_roads", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->use_roads_, test::IsBetween(kUseRoadRange.min, kUseRoadRange.max));
  }

  // speed_
  constexpr ranged_default_t<float> kRoadCyclingSpeedRange{kMinCyclingSpeed, kDefaultCyclingSpeed[0],
                                                           kMaxCyclingSpeed};
  distributor.reset(make_distributor_from_range(kRoadCyclingSpeedRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_bicyclecost_from_json("cycling_speed", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->speed_,
                test::IsBetween(kRoadCyclingSpeedRange.min, kRoadCyclingSpeedRange.max));
  }
}
} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
