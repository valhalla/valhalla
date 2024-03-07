#include "sif/lowspeedvehiclecost.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "midgard/constants.h"
#include "midgard/util.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
#include "sif/osrm_car_duration.h"
#include <cassert>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {

// Base transition costs
constexpr float kDefaultUseLivingStreets = 0.5f;  // Factor between 0 and 1
constexpr float kParkingAislePenalty = 30.0f;     // Seconds

// Minimum acceptable surface class
constexpr Surface kMinimumAcceptableSurface = Surface::kCompacted;

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

// Allowed vehicle speed ranges (for input validation).
constexpr uint32_t kMinimumTopSpeed = 20; // KPH
constexpr uint32_t kDefaultTopSpeed = 35; // KPH (21.75mph)
constexpr uint32_t kMaximumTopSpeed = 60; // KPH

// Valid ranges and defaults
constexpr ranged_default_t<uint32_t> kTopSpeedRange{kMinimumTopSpeed, kDefaultTopSpeed,
                                                    kMaximumTopSpeed};
// Bounds on max allowed speed limits (to validate input).
// In the presence of a maxspeed tag, it will be compared against this value. If the maxspeed
// value is greater than this tunable, the road will be inaccessible.
// 57kph is 35.42mph. In the US, LSV regulations typically state that LSVs may drive
// on roads having speed limits up to 35mph.
// We block any roads having an *explicitly tagged* higher speed limit
// (higher inferred speeds are merely penalized).
constexpr uint32_t kMinimumAllowedSpeedLimit = 20; // KPH (12.43mph)
constexpr uint32_t kDefaultAllowedSpeedLimit = 57; // KPH (35.42mph)
constexpr uint32_t kMaximumAllowedSpeedLimit = 80; // KPH
constexpr ranged_default_t<uint32_t> kMaxAllowedSpeedLimit{kMinimumAllowedSpeedLimit, kDefaultAllowedSpeedLimit,
                                                           kMaximumAllowedSpeedLimit};

// Weighting factor based on road class. See the EdgeCost method for details.
// TODO: As a future improvement, we can tune willingness to use higher class roads.
// LSVs are not entirely homogenous, and current weights are tuned for golf carts.
// We may introduce a second table of weights / different EdgeCost logic for LSVs later.
constexpr float kRoadClassFactor[] = {
    3.0f,  // Motorway
    2.0f,  // Trunk
    0.75f, // Primary
    0.5f, // Secondary
    0.3f,  // Tertiary
    0.3f,  // Unclassified
    0.25f,  // Residential
    0.0f  // Service, other
};

BaseCostingOptionsConfig GetBaseCostOptsConfig() {
  BaseCostingOptionsConfig cfg{};
  // override defaults
  cfg.disable_toll_booth_ = true;
  cfg.disable_rail_ferry_ = true;
  cfg.service_penalty_.def = 0;
  cfg.use_ferry_.def = 0;
  cfg.use_living_streets_.def = kDefaultUseLivingStreets;
  return cfg;
}

const std::string kDefaultVehicleType = "low_speed_vehicle";

const BaseCostingOptionsConfig kBaseCostOptsConfig = GetBaseCostOptsConfig();

} // namespace

/**
 * Derived class providing dynamic edge costing for street-legal low-speed vehicles (LSVs)
 * such as golf carts and neighborhood electric vehicles.
 */
class LowSpeedVehicleCost : public DynamicCost {
public:
  VehicleType type_;

/**
   * Construct LSV costing. Pass in cost type and costing_options using protocol buffer(pbf).
   * @param  costing_options pbf with request costing_options.
   */
  LowSpeedVehicleCost(const Costing& costing_options);

  // virtual destructor
  virtual ~LowSpeedVehicleCost() {
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
    throw std::runtime_error("LowSpeedVehicle::EdgeCost does not support transit edges");
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
                                     const InternalTurn internal_turn) const override;

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
    return static_cast<uint8_t>(type_);
  }

  /**
   * Function to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. It's also used to filter
   * edges not usable / inaccessible by the vehicle type.
   */
  bool Allowed(const baldr::DirectedEdge* edge,
               const graph_tile_ptr& tile,
               uint16_t disallow_mask = kDisallowNone) const override {
    bool allow_closures = (!filter_closures_ && !(disallow_mask & kDisallowClosure)) ||
                          !(flow_mask_ & kCurrentFlowMask);
    return DynamicCost::Allowed(edge, tile, disallow_mask) && !edge->bss_connection() &&
           // NB: SpeedType is either classified (determined by characteristics) or tagged.
           // Writing the logic this way is shorter and allows for easy short-circuiting.
           (edge->speed_type() == SpeedType::kClassified || edge->speed() <= max_allowed_speed_limit_) &&
           (allow_closures || !tile->IsClosed(edge));
  }

  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes

  float speedfactor_[kMaxSpeedKph + 1];

  // Road speed penalty factor. Penalties apply above a threshold
  float speedpenalty_[kMaxSpeedKph + 1];
  float trans_density_factor_[16]; // Density factor used in edge transition costing
  uint32_t max_allowed_speed_limit_;
};

LowSpeedVehicleCost::LowSpeedVehicleCost(const Costing& costing)
    : DynamicCost(costing, TravelMode::kDrive, kAutoAccess),
      trans_density_factor_{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.1f, 1.2f, 1.3f,
                            1.4f, 1.6f, 1.9f, 2.2f, 2.5f, 2.8f, 3.1f, 3.5f} {
  const auto& costing_options = costing.options();
  max_allowed_speed_limit_ = costing_options.max_allowed_speed_limit();

  // Set hierarchy to allow unlimited transitions
  for (auto& h : hierarchy_limits_) {
    h.max_up_transitions = kUnlimitedTransitions;
  }

  // Get the base costs
  get_base_costs(costing);

  // Get the vehicle type
  const std::string& type = costing_options.transport_type();
  if (type == "golf_cart") {
    type_ = VehicleType::kGolfCart;
    access_mask_ = kGolfCartAccess;
  } else {
    type_ = VehicleType::kLowSpeedVehicle;
    access_mask_ = kAutoAccess;
  }

  // Create speed cost table
  speedfactor_[0] = kSecPerHour; // TODO - what to make speed=0?
  for (uint32_t s = 1; s <= kMaxSpeedKph; s++) {
    speedfactor_[s] = (kSecPerHour * 0.001f) / static_cast<float>(s);

    // Severely penalize anything above the max speed, but don't block it since the
    // maxspeed hasn't been explicitly tagged if this is applied (it was allowed via access)
    speedpenalty_[s] = s <= max_allowed_speed_limit_ ? 1 : 3;
  }
}

// Check if access is allowed on the specified edge.
bool LowSpeedVehicleCost::Allowed(const baldr::DirectedEdge* edge,
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
      (edge->surface() > kMinimumAcceptableSurface) || IsUserAvoidEdge(edgeid) ||
      // NOTE: Parking aisles are baked as destination-only, but many LSVs/NEVs are explicitly allowed
      // to travel through these as a shortcut (per the GIS department of Peachtree City, GA;).
      // TODO: Probably should make this configurable as local laws may vary
      (!allow_destination_only_ && !pred.destonly() && edge->destonly() && edge->use() != Use::kParkingAisle) ||
      (pred.closure_pruning() && IsClosed(edge, tile)) ||
      (edge->speed_type() == SpeedType::kTagged && edge->speed() > max_allowed_speed_limit_)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, is_dest, tile, edgeid, current_time,
                                           tz_index, restriction_idx);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool LowSpeedVehicleCost::AllowedReverse(const baldr::DirectedEdge* edge,
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
      (opp_edge->surface() > kMinimumAcceptableSurface) || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly() && opp_edge->use() != Use::kParkingAisle) ||
      (pred.closure_pruning() && IsClosed(opp_edge, tile)) ||
      (opp_edge->speed_type() == SpeedType::kTagged && edge->speed() > max_allowed_speed_limit_)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, false, tile, opp_edgeid, current_time,
                                           tz_index, restriction_idx);
}

Cost LowSpeedVehicleCost::EdgeCost(const baldr::DirectedEdge* edge,
                            const graph_tile_ptr& tile,
                            const baldr::TimeInfo& time_info,
                            uint8_t& flow_sources) const {
  auto speed = fixed_speed_ == baldr::kDisableFixedSpeed
                   ? tile->GetSpeed(edge, flow_mask_, time_info.second_of_week, false, &flow_sources,
                                    time_info.seconds_from_now)
                   : fixed_speed_;

  uint32_t final_speed = std::min(top_speed_, speed);

  assert(final_speed < kMaxSpeedKph);
  float sec = (edge->length() * speedfactor_[final_speed]);

  if (shortest_) {
    return Cost(edge->length(), sec);
  }

  // Factor which accumulates for negative traits and is added in later.
  // It starts with the road class, since we want to prefer lower class roads in many scenarios.
  float avoidance_factor = kRoadClassFactor[static_cast<uint32_t>(edge->classification())];

  // Roads are more severely punished for having traffic faster than the vehicle's top speed.
  // Use the speed assigned to the directed edge. Even if we had traffic information we shouldn't use it here.
  avoidance_factor *= speedpenalty_[edge->speed()];

  float factor = 1.0f +
                 avoidance_factor +
                 SpeedPenalty(edge, tile, time_info, flow_sources, speed);

  if (IsClosed(edge, tile)) {
    // Add a penalty for traversing a closed edge
    factor *= closure_factor_;
  }

  return {sec * factor, sec};
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost LowSpeedVehicleCost::TransitionCost(const baldr::DirectedEdge* edge,
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

    if (edge->destonly() && !pred.destonly() && edge->use() == Use::kParkingAisle) {
      // Replace the usual destination only penalty with a lower parking aisle penalty
      c.cost += -destination_only_penalty_ + kParkingAislePenalty;
    }

    c.cost += seconds;
  }
  return c;
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// pred is the opposing current edge in the reverse tree
// edge is the opposing predecessor in the reverse tree
Cost LowSpeedVehicleCost::TransitionCostReverse(const uint32_t idx,
                                         const baldr::NodeInfo* node,
                                         const baldr::DirectedEdge* pred,
                                         const baldr::DirectedEdge* edge,
                                         const bool has_measured_speed,
                                         const InternalTurn internal_turn) const {
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

    AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, true, internal_turn,seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!has_measured_speed) {
      if (!is_turn)
        seconds *= edge->stopimpact(idx);
      seconds *= trans_density_factor_[node->density()];
    }

    if (edge->destonly() && !pred->destonly() && edge->use() == Use::kParkingAisle) {
      // Replace the usual destination only penalty with a lower parking aisle penalty
      c.cost += -destination_only_penalty_ + kParkingAislePenalty;
    }

    c.cost += seconds;
  }
  return c;
}

void ParseLowSpeedVehicleCostOptions(const rapidjson::Document& doc,
                              const std::string& costing_options_key,
                              Costing* c) {
  c->set_type(Costing::low_speed_vehicle);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  ParseBaseCostOptions(json, c, kBaseCostOptsConfig);
  JSON_PBF_RANGED_DEFAULT(co, kTopSpeedRange, json, "/top_speed", top_speed);
  JSON_PBF_RANGED_DEFAULT(co, kMaxAllowedSpeedLimit, json, "/max_allowed_speed_limit", max_allowed_speed_limit);
  JSON_PBF_DEFAULT(co, kDefaultVehicleType, json, "/vehicle_type", transport_type);
}

cost_ptr_t CreateLowSpeedVehicleCost(const Costing& costing_options) {
  return std::make_shared<LowSpeedVehicleCost>(costing_options);
}

} // namespace sif
} // namespace valhalla
