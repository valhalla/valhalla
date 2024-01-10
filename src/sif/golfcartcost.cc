#include "sif/golfcartcost.h"
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
constexpr Surface kMinimumGolfCartSurface = Surface::kCompacted;

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

// Minimum and maximum golf cart top speed (to validate input).
constexpr uint32_t kMinimumTopSpeed = 20; // KPH
constexpr uint32_t kDefaultTopSpeed = 35; // KPH (21.75mph)
constexpr uint32_t kMaximumTopSpeed = 60; // KPH

// Valid ranges and defaults
constexpr ranged_default_t<uint32_t> kTopSpeedRange{kMinimumTopSpeed, kDefaultTopSpeed,
                                                    kMaximumTopSpeed};
// Bounds on max allowed speed limits (to validate input).
// In the presence of a maxspeed tag, it will be compared against this value. If the maxspeed
// value is greater than this tunable, the road will be inaccessible.
// 57kph is 35.42mph. In the US, LSV regulations typically state that golf carts may drive
// on roads having speed limits up to 35mph. Block any road having an
// *explicitly tagged* higher speed limit (implicit ones just get a penalty).
constexpr uint32_t kMinimumAllowedSpeedLimit = 20; // KPH (12.43mph)
constexpr uint32_t kDefaultAllowedSpeedLimit = 57; // KPH (35.42mph)
constexpr uint32_t kMaximumAllowedSpeedLimit = 80; // KPH
constexpr ranged_default_t<uint32_t> kMaxAllowedSpeedLimit{kMinimumAllowedSpeedLimit, kDefaultAllowedSpeedLimit,
                                                           kMaximumAllowedSpeedLimit};

// Weighting factor based on road class. These apply penalties to higher class
// roads. These penalties are additive based on other penalties.
constexpr float kRoadClassPenaltyFactor[] = {
    1.0f,  // Motorway
    1.0f,  // Trunk
    1.0f, // Primary
    1.0f, // Secondary
    0.75f,  // Tertiary
    0.5f,  // Unclassified
    0.25f,  // Residential
    0.05f  // Service, other
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

const BaseCostingOptionsConfig kBaseCostOptsConfig = GetBaseCostOptsConfig();

} // namespace

/**
 * Derived class providing dynamic edge costing for street-legal golf carts.
 */
class GolfCartCost : public DynamicCost {
public:
/**
   * Construct golf cart costing. Pass in cost type and costing_options using protocol buffer(pbf).
   * @param  costing_options pbf with request costing_options.
   */
  GolfCartCost(const Costing& costing_options);

  // virtual destructor
  virtual ~GolfCartCost() {
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
    throw std::runtime_error("GolfCartCost::EdgeCost does not support transit edges");
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
    return static_cast<uint8_t>(VehicleType::kGolfCart);
  }

  /**
   * Function to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. It's also used to filter
   * edges not usable / inaccessible by golf cart.
   */
  bool Allowed(const baldr::DirectedEdge* edge,
               const graph_tile_ptr& tile,
               uint16_t disallow_mask = kDisallowNone) const override {
    bool allow_closures = (!filter_closures_ && !(disallow_mask & kDisallowClosure)) ||
                          !(flow_mask_ & kCurrentFlowMask);
    return DynamicCost::Allowed(edge, tile, disallow_mask) && !edge->bss_connection() &&
           (edge->speed_type() == SpeedType::kClassified || tile->edgeinfo(edge).speed_limit() <= max_allowed_speed_limit_) &&
           (allow_closures || !tile->IsClosed(edge));
  }

  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes

  std::vector<float> speedfactor_;

  // Road speed penalty factor. Penalties apply above a threshold
  float speedpenalty_[kMaxSpeedKph + 1];
  std::vector<float> trans_density_factor_; // Density factor used in edge transition costing
  uint32_t max_allowed_speed_limit_;
};

GolfCartCost::GolfCartCost(const Costing& costing)
    : DynamicCost(costing, TravelMode::kDrive, kGolfCartAccess),
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

  // Create speed cost table
  speedfactor_.resize(kMaxSpeedKph + 1, 0);
  speedfactor_[0] = kSecPerHour; // TODO - what to make speed=0?
  for (uint32_t s = 1; s <= kMaxSpeedKph; s++) {
    speedfactor_[s] = (kSecPerHour * 0.001f) / static_cast<float>(s);

    // Severely penalize anything above the max speed, but don't block it since the
    // maxspeed hasn't been explicitly tagged if this is applied (it was allowed via access)
    speedpenalty_[s] = s <= max_allowed_speed_limit_ ? 1 : 3;
  }
}

// Check if access is allowed on the specified edge.
bool GolfCartCost::Allowed(const baldr::DirectedEdge* edge,
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
      (edge->surface() > kMinimumGolfCartSurface) || IsUserAvoidEdge(edgeid) ||
      // NOTE: Parking aisles are baked as destination-only, but for golf carts we actually need
      // to ignore this (per the GIS department of Peachtree City, GA).
      (!allow_destination_only_ && !pred.destonly() && edge->destonly() && edge->use() != Use::kParkingAisle) ||
      (pred.closure_pruning() && IsClosed(edge, tile)) ||
      (edge->speed_type() == SpeedType::kTagged && tile->edgeinfo(edge).speed_limit() > max_allowed_speed_limit_)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, is_dest, tile, edgeid, current_time,
                                           tz_index, restriction_idx);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool GolfCartCost::AllowedReverse(const baldr::DirectedEdge* edge,
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
      (opp_edge->surface() > kMinimumGolfCartSurface) || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly() && opp_edge->use() != Use::kParkingAisle) ||
      (pred.closure_pruning() && IsClosed(opp_edge, tile)) ||
      (opp_edge->speed_type() == SpeedType::kTagged && tile->edgeinfo(opp_edge).speed_limit() > max_allowed_speed_limit_)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, false, tile, opp_edgeid, current_time,
                                           tz_index, restriction_idx);
}

Cost GolfCartCost::EdgeCost(const baldr::DirectedEdge* edge,
                            const graph_tile_ptr& tile,
                            const baldr::TimeInfo& time_info,
                            uint8_t& flow_sources) const {
  auto speed = fixed_speed_ == baldr::kDisableFixedSpeed
                   ? tile->GetSpeed(edge, flow_mask_, time_info.second_of_week, false, &flow_sources,
                                    time_info.seconds_from_now)
                   : fixed_speed_;

  uint32_t final_speed = std::min(top_speed_, speed);

  assert(final_speed < speedfactor_.size());
  float sec = (edge->length() * speedfactor_[final_speed]);

  if (shortest_) {
    return Cost(edge->length(), sec);
  }

  // Represents negative traits without looking at special accommodations for golf carts
  float avoidance_factor = 1.0f;

  // Parking aisles get special treatment in golf cart costing.
  // Confirming with several sources including the municipality of Peachtree City, GA,
  // these should be routable without much penalty.
  bool is_parking_aisle = edge->use() == Use::kParkingAisle;

  // Add penalties based on road class.
  // The dest only penalty (for private roads) is normally just a one-time fixed penalty on transitions.
  // However, in the case of golf cart routing, this is usually not sufficient to avoid large areas such as private golf courses which are not routable except as a destination.
  avoidance_factor += kRoadClassPenaltyFactor[static_cast<uint32_t>(edge->classification())] + (edge->destonly() * !is_parking_aisle * 10);

  // Multiply by speed so that roads are more severely punished for having traffic faster than the golf cart's speed.
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
Cost GolfCartCost::TransitionCost(const baldr::DirectedEdge* edge,
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
Cost GolfCartCost::TransitionCostReverse(const uint32_t idx,
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

void ParseGolfCartCostOptions(const rapidjson::Document& doc,
                              const std::string& costing_options_key,
                              Costing* c) {
  c->set_type(Costing::golf_cart);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  ParseBaseCostOptions(json, c, kBaseCostOptsConfig);
  JSON_PBF_RANGED_DEFAULT(co, kTopSpeedRange, json, "/top_speed", top_speed);
  JSON_PBF_RANGED_DEFAULT(co, kMaxAllowedSpeedLimit, json, "/max_allowed_speed_limit", max_allowed_speed_limit);
}

cost_ptr_t CreateGolfCartCost(const Costing& costing_options) {
  return std::make_shared<GolfCartCost>(costing_options);
}

} // namespace sif
} // namespace valhalla
