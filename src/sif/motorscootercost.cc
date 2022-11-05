#include "sif/motorscootercost.h"
#include "baldr/accessrestriction.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "midgard/constants.h"
#include "midgard/util.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
#include "sif/osrm_car_duration.h"
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

// Base transition costs (not toll booth penalties since scooters likely don't take toll roads)
constexpr float kDefaultDestinationOnlyPenalty = 120.0f; // Seconds

// Other options
constexpr float kDefaultUseHills = 0.5f;   // Factor between 0 and 1
constexpr float kDefaultUsePrimary = 0.5f; // Factor between 0 and 1

constexpr uint32_t kMinimumTopSpeed = 20;  // Kilometers per hour
constexpr uint32_t kDefaultTopSpeed = 45;  // Kilometers per hour
constexpr uint32_t kMaximumTopSpeed = 120; // Kilometers per hour
constexpr Surface kMinimumScooterSurface = Surface::kDirt;

// Default turn costs
constexpr float kTCStraight = 0.5f;
constexpr float kTCSlight = 0.75f;
constexpr float kTCFavorable = 1.0f;
constexpr float kTCFavorableSharp = 1.5f;
constexpr float kTCCrossing = 2.0f;
constexpr float kTCUnfavorable = 2.5f;
constexpr float kTCUnfavorableSharp = 3.5f;
constexpr float kTCReverse = 9.5f;

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = {kTCStraight,       kTCSlight,  kTCFavorable,
                                         kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
                                         kTCUnfavorable,    kTCSlight};
constexpr float kLeftSideTurnCosts[] = {kTCStraight,         kTCSlight,  kTCUnfavorable,
                                        kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
                                        kTCFavorable,        kTCSlight};

// Valid ranges and defaults
constexpr ranged_default_t<float> kUseHillsRange{0, kDefaultUseHills, 1.0f};
constexpr ranged_default_t<float> kUsePrimaryRange{0, kDefaultUsePrimary, 1.0f};
constexpr ranged_default_t<uint32_t> kTopSpeedRange{kMinimumTopSpeed, kDefaultTopSpeed,
                                                    kMaximumTopSpeed};

// Additional penalty to avoid destination only
constexpr float kDestinationOnlyFactor = 0.2f;

// Weighting factor based on road class. These apply penalties to higher class
// roads. These penalties are modulated by the road factor - further
// avoiding higher class roads for those with low propensity for using
// primary roads.
constexpr float kRoadClassFactor[] = {
    1.0f,  // Motorway
    0.5f,  // Trunk
    0.2f,  // Primary
    0.1f,  // Secondary
    0.05f, // Tertiary
    0.05f, // Unclassified
    0.0f,  // Residential
    0.5f   // Service, other
};

constexpr uint32_t kMaxGradeFactor = 15;

// Avoid hills "strength". How much do we want to avoid a hill. Combines
// with the usehills factor (1.0 - usehills = avoidhills factor) to create
// a weighting penalty per weighted grade factor. This indicates how strongly
// edges with the specified grade are weighted. Note that speed also is
// influenced by grade, so these weights help further avoid hills.
constexpr float kAvoidHillsStrength[] = {
    1.0f,  // -10%  - Very steep downhill
    0.8f,  // -8%
    0.5f,  // -6.5%
    0.2f,  // -5%   - Moderately steep downhill
    0.1f,  // -3%
    0.0f,  // -1.5%
    0.05f, // 0%    - Flat
    0.1f,  // 1.5%
    0.3f,  // 3%
    0.8f,  // 5%
    2.0f,  // 6.5%
    3.0f,  // 8%    - Moderately steep uphill
    4.5f,  // 10%
    6.0f,  // 11.5%
    8.0f,  // 13%
    10.0f  // 15%   - Very steep uphill
};

constexpr float kGradeBasedSpeedFactor[] = {
    1.25f, // -10%  - 45
    1.2f,  // -8%   - 40.5
    1.15f, // -6.5% - 36
    1.1f,  // -5%   - 30.6
    1.05f, // -3%   - 25
    1.0f,  // -1.5% - 21.6
    1.0f,  // 0%    - 18
    1.0f,  // 1.5%  - 17
    0.95f, // 3%    - 15
    0.75f, // 5%    - 13.5
    0.6f,  // 6.5%  - 12
    0.5f,  // 8%    - 10
    0.45f, // 10%   - 9
    0.4f,  // 11.5% - 8
    0.35f, // 13%   - 7
    0.25f  // 15%   - 5.5
};

constexpr float kSurfaceSpeedFactors[] = {1.0f, 1.0f, 0.9f, 0.6f, 0.1f, 0.0f, 0.0f, 0.0f};

BaseCostingOptionsConfig GetBaseCostOptsConfig() {
  BaseCostingOptionsConfig cfg{};
  // override defaults
  cfg.dest_only_penalty_.def = kDefaultDestinationOnlyPenalty;
  cfg.disable_toll_booth_ = true;
  cfg.disable_rail_ferry_ = true;
  return cfg;
}

const BaseCostingOptionsConfig kBaseCostOptsConfig = GetBaseCostOptsConfig();

} // namespace

/**
 * Derived class providing dynamic edge costing for "direct" auto routes. This
 * is a route that is generally shortest time but uses route hierarchies that
 * can result in slightly longer routes that avoid shortcuts on residential
 * roads.
 */
class MotorScooterCost : public DynamicCost {
public:
  /**
   * Construct motor scooter costing. Pass in cost type and costing_options using protocol
   * buffer(pbf).
   * @param  costing specified costing type.
   * @param  costing_options pbf with request costing_options.
   */
  MotorScooterCost(const Costing& costing_options);

  // virtual destructor
  virtual ~MotorScooterCost() {
  }

  /**
   * Does the costing method allow multiple passes (with relaxed hierarchy
   * limits).
   * @return  Returns true if the costing model allows multiple passes.
   */
  virtual bool AllowMultiPass() const override {
    return true;
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
    throw std::runtime_error("MotorScooterCost::EdgeCost does not support transit edges");
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  time_info Time info about edge passing.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const graph_tile_ptr& tile,
                        const baldr::TimeInfo& time_info,
                        uint8_t& flow_sources) const override;

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
                                     const bool has_measured_speed,
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
    return speedfactor_[top_speed_];
  }

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const override {
    return static_cast<uint8_t>(VehicleType::kMotorScooter);
  }
  /**
   * Function to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. It's also used to filter
   * edges not usable / inaccessible by automobile.
   */
  bool Allowed(const baldr::DirectedEdge* edge,
               const graph_tile_ptr& tile,
               uint16_t disallow_mask = kDisallowNone) const override {
    bool allow_closures = (!filter_closures_ && !(disallow_mask & kDisallowClosure)) ||
                          !(flow_mask_ & kCurrentFlowMask);
    return DynamicCost::Allowed(edge, tile, disallow_mask) && !edge->bss_connection() &&
           (allow_closures || !tile->IsClosed(edge));
  }
  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes
public:
  std::vector<float> speedfactor_;
  float density_factor_[16]; // Density factor

  // Density factor used in edge transition costing
  std::vector<float> trans_density_factor_;
  float road_factor_; // Road factor based on use_primary

  // Elevation/grade penalty (weighting applied based on the edge's weighted
  // grade (relative value from 0-15)
  float grade_penalty_[16];
};

// Constructor
MotorScooterCost::MotorScooterCost(const Costing& costing)
    : DynamicCost(costing, TravelMode::kDrive, kMopedAccess),
      trans_density_factor_{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.1f, 1.2f, 1.3f,
                            1.4f, 1.6f, 1.9f, 2.2f, 2.5f, 2.8f, 3.1f, 3.5f} {
  const auto& costing_options = costing.options();

  // Get the base costs
  get_base_costs(costing);

  // Create speed cost table
  speedfactor_.resize(kMaxSpeedKph + 1, 0);
  speedfactor_[0] = kSecPerHour; // TODO - what to make speed=0?
  for (uint32_t s = 1; s <= kMaxSpeedKph; s++) {
    speedfactor_[s] = (kSecPerHour * 0.001f) / static_cast<float>(s);
  }

  // Set density factors - used to penalize edges in dense, urban areas
  for (uint32_t d = 0; d < 16; d++) {
    density_factor_[d] = 0.85f + (d * 0.018f);
  }

  // Set grade penalties based on use_hills option.
  // Scale from 0 (avoid hills) to 1 (don't avoid hills)
  float use_hills = costing_options.use_hills();
  float avoid_hills = (1.0f - use_hills);
  for (uint32_t i = 0; i <= kMaxGradeFactor; ++i) {
    grade_penalty_[i] = avoid_hills * kAvoidHillsStrength[i];
  }

  // Set the road classification factor based on use_primary option - scales from
  // 0 (avoid primary roads) to 1 (don't avoid primary roads). Above 0.5 start to
  // reduce the weight difference between road classes while factors below 0.5
  // start to increase the differences.
  float use_primary = costing_options.use_primary();
  road_factor_ = (use_primary >= 0.5f) ? 1.5f - use_primary : 3.0f - use_primary * 5.0f;
}

// Check if access is allowed on the specified edge.
bool MotorScooterCost::Allowed(const baldr::DirectedEdge* edge,
                               const bool is_dest,
                               const EdgeLabel& pred,
                               const graph_tile_ptr& tile,
                               const baldr::GraphId& edgeid,
                               const uint64_t current_time,
                               const uint32_t tz_index,
                               uint8_t& restriction_idx) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!IsAccessible(edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((pred.restrictions() & (1 << edge->localedgeidx())) && !ignore_restrictions_) ||
      (edge->surface() > kMinimumScooterSurface) || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(edge, tile))) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, is_dest, tile, edgeid, current_time,
                                           tz_index, restriction_idx);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool MotorScooterCost::AllowedReverse(const baldr::DirectedEdge* edge,
                                      const EdgeLabel& pred,
                                      const baldr::DirectedEdge* opp_edge,
                                      const graph_tile_ptr& tile,
                                      const baldr::GraphId& opp_edgeid,
                                      const uint64_t current_time,
                                      const uint32_t tz_index,
                                      uint8_t& restriction_idx) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!IsAccessible(opp_edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((opp_edge->restrictions() & (1 << pred.opp_local_idx())) && !ignore_restrictions_) ||
      (opp_edge->surface() > kMinimumScooterSurface) || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(opp_edge, tile))) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, false, tile, opp_edgeid, current_time,
                                           tz_index, restriction_idx);
}

Cost MotorScooterCost::EdgeCost(const baldr::DirectedEdge* edge,
                                const graph_tile_ptr& tile,
                                const baldr::TimeInfo& time_info,
                                uint8_t& flow_sources) const {
  auto speed = fixed_speed_ == baldr::kDisableFixedSpeed
                   ? tile->GetSpeed(edge, flow_mask_, time_info.second_of_week, false, &flow_sources,
                                    time_info.seconds_from_now)
                   : fixed_speed_;

  if (edge->use() == Use::kFerry) {
    assert(speed < speedfactor_.size());
    float sec = (edge->length() * speedfactor_[speed]);
    return {sec * ferry_factor_, sec};
  }

  uint32_t scooter_speed =
      (std::min(top_speed_, speed) * kSurfaceSpeedFactors[static_cast<uint32_t>(edge->surface())] *
       kGradeBasedSpeedFactor[static_cast<uint32_t>(edge->weighted_grade())]);

  assert(scooter_speed < speedfactor_.size());
  float sec = (edge->length() * speedfactor_[scooter_speed]);

  if (shortest_) {
    return Cost(edge->length(), sec);
  }

  float factor = 1.0f + (density_factor_[edge->density()] - 0.85f) +
                 (road_factor_ * kRoadClassFactor[static_cast<uint32_t>(edge->classification())]) +
                 grade_penalty_[static_cast<uint32_t>(edge->weighted_grade())] +
                 SpeedPenalty(edge, tile, time_info, flow_sources, speed);

  if (edge->destonly()) {
    factor += kDestinationOnlyFactor;
  }

  if (edge->use() == Use::kTrack) {
    factor *= track_factor_;
  } else if (edge->use() == Use::kLivingStreet) {
    factor *= living_street_factor_;
  } else if (edge->use() == Use::kServiceRoad) {
    factor *= service_factor_;
  }
  if (IsClosed(edge, tile)) {
    // Add a penalty for traversing a closed edge
    factor *= closure_factor_;
  }

  return {sec * factor, sec};
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost MotorScooterCost::TransitionCost(const baldr::DirectedEdge* edge,
                                      const baldr::NodeInfo* node,
                                      const EdgeLabel& pred) const {
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = base_transition_cost(node, edge, &pred, idx);
  c.secs = OSRMCarTurnDuration(edge, node, idx);

  // Transition time = turncost * stopimpact * densityfactor
  if (edge->stopimpact(idx) > 0 && !shortest_) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (node->drive_on_right())
                      ? kRightSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))]
                      : kLeftSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))];
    }

    if ((edge->use() != Use::kRamp && pred.use() == Use::kRamp) ||
        (edge->use() == Use::kRamp && pred.use() != Use::kRamp)) {
      turn_cost += 1.5f;
      if (edge->roundabout())
        turn_cost += 0.5f;
    }

    float seconds = turn_cost;
    bool is_turn = false;
    bool has_left = (edge->turntype(idx) == baldr::Turn::Type::kLeft ||
                     edge->turntype(idx) == baldr::Turn::Type::kSharpLeft);
    bool has_right = (edge->turntype(idx) == baldr::Turn::Type::kRight ||
                      edge->turntype(idx) == baldr::Turn::Type::kSharpRight);
    bool has_reverse = edge->turntype(idx) == baldr::Turn::Type::kReverse;

    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    if (has_left || has_right || has_reverse) {
      seconds *= edge->stopimpact(idx);
      is_turn = true;
    }

    AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, false, InternalTurn::kNoTurn,
                    seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!pred.has_measured_speed()) {
      if (!is_turn)
        seconds *= edge->stopimpact(idx);
      seconds *= trans_density_factor_[node->density()];
    }
    c.cost += seconds;
  }
  return c;
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// pred is the opposing current edge in the reverse tree
// edge is the opposing predecessor in the reverse tree
Cost MotorScooterCost::TransitionCostReverse(const uint32_t idx,
                                             const baldr::NodeInfo* node,
                                             const baldr::DirectedEdge* pred,
                                             const baldr::DirectedEdge* edge,
                                             const bool has_measured_speed,
                                             const InternalTurn /*internal_turn*/) const {

  // MotorScooters should be able to make uturns on short internal edges; therefore, InternalTurn
  // is ignored for now.
  // TODO: do we want to update the cost if we have flow or speed from traffic.

  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  Cost c = base_transition_cost(node, edge, pred, idx);
  c.secs = OSRMCarTurnDuration(edge, node, pred->opp_local_idx());

  // Transition time = turncost * stopimpact * densityfactor
  if (edge->stopimpact(idx) > 0 && !shortest_) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (node->drive_on_right())
                      ? kRightSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))]
                      : kLeftSideTurnCosts[static_cast<uint32_t>(edge->turntype(idx))];
    }

    if ((edge->use() != Use::kRamp && pred->use() == Use::kRamp) ||
        (edge->use() == Use::kRamp && pred->use() != Use::kRamp)) {
      turn_cost += 1.5f;
      if (edge->roundabout())
        turn_cost += 0.5f;
    }

    float seconds = turn_cost;
    bool is_turn = false;
    bool has_left = (edge->turntype(idx) == baldr::Turn::Type::kLeft ||
                     edge->turntype(idx) == baldr::Turn::Type::kSharpLeft);
    bool has_right = (edge->turntype(idx) == baldr::Turn::Type::kRight ||
                      edge->turntype(idx) == baldr::Turn::Type::kSharpRight);
    bool has_reverse = edge->turntype(idx) == baldr::Turn::Type::kReverse;

    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    if (has_left || has_right || has_reverse) {
      seconds *= edge->stopimpact(idx);
      is_turn = true;
    }

    AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, false, InternalTurn::kNoTurn,
                    seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!has_measured_speed) {
      if (!is_turn)
        seconds *= edge->stopimpact(idx);
      seconds *= trans_density_factor_[node->density()];
    }
    c.cost += seconds;
  }
  return c;
}

void ParseMotorScooterCostOptions(const rapidjson::Document& doc,
                                  const std::string& costing_options_key,
                                  Costing* c) {
  c->set_type(Costing::motor_scooter);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  ParseBaseCostOptions(json, c, kBaseCostOptsConfig);
  JSON_PBF_RANGED_DEFAULT(co, kTopSpeedRange, json, "/top_speed", top_speed);
  JSON_PBF_RANGED_DEFAULT(co, kUseHillsRange, json, "/use_hills", use_hills);
  JSON_PBF_RANGED_DEFAULT(co, kUsePrimaryRange, json, "/use_primary", use_primary);
}

cost_ptr_t CreateMotorScooterCost(const Costing& costing_options) {
  return std::make_shared<MotorScooterCost>(costing_options);
}

} // namespace sif
} // namespace valhalla

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

class TestMotorScooterCost : public MotorScooterCost {
public:
  TestMotorScooterCost(const Costing& costing_options) : MotorScooterCost(costing_options){};

  using MotorScooterCost::alley_penalty_;
  using MotorScooterCost::country_crossing_cost_;
  using MotorScooterCost::destination_only_penalty_;
  using MotorScooterCost::ferry_transition_cost_;
  using MotorScooterCost::gate_cost_;
  using MotorScooterCost::maneuver_penalty_;
  using MotorScooterCost::service_factor_;
  using MotorScooterCost::service_penalty_;
  using MotorScooterCost::top_speed_;
};

TestMotorScooterCost* make_motorscootercost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({"costing_options":{"motor_scooter":{")" << property << R"(":)" << testVal << "}}}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return new TestMotorScooterCost(request.options().costings().find(Costing::motor_scooter)->second);
}

template <typename T>
std::uniform_real_distribution<T>*
make_real_distributor_from_range(const ranged_default_t<T>& range) {
  T rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<T>(range.min - rangeLength, range.max + rangeLength);
}

template <typename T>
std::uniform_int_distribution<T>* make_int_distributor_from_range(const ranged_default_t<T>& range) {
  T rangeLength = range.max - range.min;
  return new std::uniform_int_distribution<T>(range.min - rangeLength, range.max + rangeLength);
}

TEST(MotorscooterCost, testMotorScooterCostParams) {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> fDistributor;
  std::shared_ptr<std::uniform_int_distribution<uint32_t>> iDistributor;
  std::shared_ptr<TestMotorScooterCost> ctorTester;

  const auto& defaults = kBaseCostOptsConfig;

  // maneuver_penalty_
  fDistributor.reset(make_real_distributor_from_range(defaults.maneuver_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("maneuver_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->maneuver_penalty_,
                test::IsBetween(defaults.maneuver_penalty_.min, defaults.maneuver_penalty_.max));
  }

  // alley_penalty_
  fDistributor.reset(make_real_distributor_from_range(defaults.alley_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("alley_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->alley_penalty_,
                test::IsBetween(defaults.alley_penalty_.min, defaults.alley_penalty_.max));
  }

  // destination_only_penalty_
  fDistributor.reset(make_real_distributor_from_range(defaults.dest_only_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorscootercost_from_json("destination_only_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->destination_only_penalty_,
                test::IsBetween(defaults.dest_only_penalty_.min, defaults.dest_only_penalty_.max));
  }

  // gate_cost_ (Cost.secs)
  fDistributor.reset(make_real_distributor_from_range(defaults.gate_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("gate_cost", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.secs,
                test::IsBetween(defaults.gate_cost_.min, defaults.gate_cost_.max));
  }

  // gate_penalty_ (Cost.cost)
  fDistributor.reset(make_real_distributor_from_range(defaults.gate_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("gate_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.cost,
                test::IsBetween(defaults.gate_penalty_.min, defaults.gate_penalty_.max));
  }

  // country_crossing_cost_ (Cost.secs)
  fDistributor.reset(make_real_distributor_from_range(defaults.country_crossing_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorscootercost_from_json("country_crossing_cost", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.secs,
                test::IsBetween(defaults.country_crossing_cost_.min,
                                defaults.country_crossing_cost_.max));
  }

  // country_crossing_penalty_ (Cost.cost)
  fDistributor.reset(make_real_distributor_from_range(defaults.country_crossing_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorscootercost_from_json("country_crossing_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.cost,
                test::IsBetween(defaults.country_crossing_penalty_.min,
                                defaults.country_crossing_penalty_.max +
                                    defaults.country_crossing_cost_.def));
  }

  // ferry_cost_ (Cost.secs)
  fDistributor.reset(make_real_distributor_from_range(defaults.ferry_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("ferry_cost", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->ferry_transition_cost_.secs,
                test::IsBetween(defaults.ferry_cost_.min, defaults.ferry_cost_.max));
  }

  // top_speed_
  iDistributor.reset(make_int_distributor_from_range(kTopSpeedRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("top_speed", (*iDistributor)(generator)));
    EXPECT_THAT(ctorTester->top_speed_, test::IsBetween(kTopSpeedRange.min, kTopSpeedRange.max));
  }

  // service_penalty_
  fDistributor.reset(make_real_distributor_from_range(defaults.service_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("service_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->service_penalty_,
                test::IsBetween(defaults.service_penalty_.min, defaults.service_penalty_.max));
  }

  // service_factor_
  fDistributor.reset(make_real_distributor_from_range(defaults.service_factor_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("service_factor", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->service_factor_,
                test::IsBetween(defaults.service_factor_.min, defaults.service_factor_.max));
  }

  /**
  // use_ferry
  fDistributor.reset(make_real_distributor_from_range(defaults.use_ferry_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("use_ferry", (*fDistributor)(generator)));
EXPECT_THAT(ctorTester->use_ferry , test::IsBetween(defaults.use_ferry_.min,
defaults.use_ferry_.max));
  }

  // use_hills - used in the constructor to create grade penalties
  fDistributor.reset(make_real_distributor_from_range(kUseHillsRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("use_hills", (*fDistributor)(generator)));
EXPECT_THAT(ctorTester->use_hills , test::IsBetween( kUseHillsRange.min ,kUseHillsRange.max));
  }

  // use_primary - used in the constructor to create road factors.
  fDistributor.reset(make_real_distributor_from_range(kUsePrimaryRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("use_primary", (*fDistributor)(generator)));
EXPECT_THAT(ctorTester->use_primary , test::IsBetween( kUsePrimaryRange.min ,kUsePrimaryRange.max));
  }
**/
}
} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
