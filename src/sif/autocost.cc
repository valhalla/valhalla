#include "sif/autocost.h"
#include "baldr/accessrestriction.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "midgard/constants.h"
#include "midgard/util.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"
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

// Base transition costs
constexpr float kDefaultServicePenalty = 75.0f; // Seconds

// Other options
constexpr float kDefaultUseHighways = 0.5f; // Default preference of using a motorway or trunk 0-1
constexpr float kDefaultUseTolls = 0.5f;    // Default preference of using toll roads 0-1
constexpr float kDefaultUseTracks = 0.f;    // Default preference of using tracks 0-1
constexpr float kDefaultUseDistance = 0.f;  // Default preference of using distance vs time 0-1

// Default turn costs
constexpr float kTCStraight = 0.5f;
constexpr float kTCSlight = 0.75f;
constexpr float kTCFavorable = 1.0f;
constexpr float kTCFavorableSharp = 1.5f;
constexpr float kTCCrossing = 2.0f;
constexpr float kTCUnfavorable = 2.5f;
constexpr float kTCUnfavorableSharp = 3.5f;
constexpr float kTCReverse = 9.5f;

// How much to favor taxi roads.
constexpr float kTaxiFactor = 0.85f;

// Do not avoid alleys by default
constexpr float kDefaultAlleyFactor = 1.0f;

// How much to favor turn channels
constexpr float kTurnChannelFactor = 0.6f;

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = {kTCStraight,       kTCSlight,  kTCFavorable,
                                         kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
                                         kTCUnfavorable,    kTCSlight};
constexpr float kLeftSideTurnCosts[] = {kTCStraight,         kTCSlight,  kTCUnfavorable,
                                        kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
                                        kTCFavorable,        kTCSlight};

constexpr float kMinFactor = 0.1f;
constexpr float kMaxFactor = 100000.0f;

// Default auto attributes
constexpr float kDefaultAutoHeight = 1.6f; // Meters (62.9921 inches)
constexpr float kDefaultAutoWidth = 1.9f;  // Meters (74.8031 inches)

// Valid ranges and defaults
constexpr ranged_default_t<float> kAlleyFactorRange{kMinFactor, kDefaultAlleyFactor, kMaxFactor};
constexpr ranged_default_t<float> kUseHighwaysRange{0, kDefaultUseHighways, 1.0f};
constexpr ranged_default_t<float> kUseTollsRange{0, kDefaultUseTolls, 1.0f};
constexpr ranged_default_t<float> kUseDistanceRange{0, kDefaultUseDistance, 1.0f};
constexpr ranged_default_t<float> kAutoHeightRange{0, kDefaultAutoHeight, 10.0f};
constexpr ranged_default_t<float> kAutoWidthRange{0, kDefaultAutoWidth, 10.0f};

// Maximum highway avoidance bias (modulates the highway factors based on road class)
constexpr float kMaxHighwayBiasFactor = 8.0f;

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

// The basic costing for an edge is a trade off between time and distance. We allow the user to
// specify which one is more important to them and then we use a linear combination to combine the two
// into a final metric. The problem is that time in seconds and length in meters have two wildly
// different ranges, so the linear combination always favors length vs time. What we do to combat this
// is to change length units into time units by multiplying by the reciprocal of a constant speed.
// This means basically changes the units of distance to be more in the same ballpark as the units of
// time and makes the linear combination make more sense.
constexpr float kInvMedianSpeed = 1.f / 16.f; // about 37mph

BaseCostingOptionsConfig GetBaseCostOptsConfig() {
  BaseCostingOptionsConfig cfg{};
  // override defaults
  cfg.service_penalty_.def = kDefaultServicePenalty;
  cfg.use_tracks_.def = kDefaultUseTracks;
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
class AutoCost : public DynamicCost {
public:
  /**
   * Construct auto costing. Pass in cost type and costing_options using protocol buffer(pbf).
   * @param  costing_options pbf with request costing_options.
   */
  AutoCost(const CostingOptions& costing_options, uint32_t access_mask = (kAutoAccess | kHOVAccess));

  virtual ~AutoCost() {
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
   * Callback for Allowed doing mode  specific restriction checks
   */
  virtual bool ModeSpecificAllowed(const baldr::AccessRestriction& restriction) const override;

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
    throw std::runtime_error("AutoCost::EdgeCost does not support transit edges");
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge    Pointer to a directed edge.
   * @param   tile    Graph tile.
   * @param   seconds Time of week in seconds.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const graph_tile_ptr& tile,
                        const uint32_t seconds,
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

  bool IsHOVAllowed(const baldr::DirectedEdge* edge) const {
    // A non-hov edge means hov is allowed.
    if (!edge->is_hov_only())
      return true;

    // The edge is either HOV-2 or HOV-3 from this point forward.

    // If include_hov3 is set we can route onto both HOV-2 and HOV-3 edges
    if (include_hov3_)
      return true;

    // If include_hov2 is set we can route onto HOV-2 edges.
    if (include_hov2_ && (edge->hov_type() == baldr::HOVEdgeType::kHOV2))
      return true;

    // If include_hot is set we can route onto HOT edges (HOV and tolled).
    if (include_hot_ && edge->toll())
      return true;

    return false;
  }

  /**
   * Function to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. It's also used to filter
   * edges not usable / inaccessible by automobile.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const graph_tile_ptr& tile,
                       uint16_t disallow_mask = kDisallowNone) const override {
    bool allow_closures = (!filter_closures_ && !(disallow_mask & kDisallowClosure)) ||
                          !(flow_mask_ & kCurrentFlowMask);
    return DynamicCost::Allowed(edge, tile, disallow_mask) && !edge->bss_connection() &&
           (allow_closures || !tile->IsClosed(edge)) && IsHOVAllowed(edge);
  }

  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes
public:
  VehicleType type_; // Vehicle type: car (default), motorcycle, etc
  std::vector<float> speedfactor_;
  float density_factor_[16];  // Density factor
  float highway_factor_;      // Factor applied when road is a motorway or trunk
  float alley_factor_;        // Avoid alleys factor.
  float toll_factor_;         // Factor applied when road has a toll
  float surface_factor_;      // How much the surface factors are applied.
  float distance_factor_;     // How much distance factors in overall favorability
  float inv_distance_factor_; // How much time factors in overall favorability

  // Vehicle attributes (used for special restrictions and costing)
  float height_; // Vehicle height in meters
  float width_;  // Vehicle width in meters

  // Density factor used in edge transition costing
  std::vector<float> trans_density_factor_;
};

// Constructor
AutoCost::AutoCost(const CostingOptions& costing_options, uint32_t access_mask)
    : DynamicCost(costing_options, TravelMode::kDrive, access_mask, true),
      trans_density_factor_{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.1f, 1.2f, 1.3f,
                            1.4f, 1.6f, 1.9f, 2.2f, 2.5f, 2.8f, 3.1f, 3.5f} {

  // Get the vehicle type - enter as string and convert to enum.
  // Used to set the surface factor - penalize some roads based on surface type.
  surface_factor_ = 0.5f;
  const std::string& type = costing_options.transport_type();
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

  // Get the base transition costs
  get_base_costs(costing_options);

  // Get alley factor from costing options.
  alley_factor_ = costing_options.alley_factor();

  // Preference to use highways. Is a value from 0 to 1
  // Factor for highway use - use a non-linear factor with values at 0.5 being neutral (factor
  // of 0). Values between 0.5 and 1 slowly decrease to a maximum of -0.125 (to slightly prefer
  // highways) while values between 0.5 to 0 slowly increase to a maximum of kMaxHighwayBiasFactor
  // to avoid/penalize highways.
  float use_highways = costing_options.use_highways();
  if (use_highways >= 0.5f) {
    float f = (0.5f - use_highways);
    highway_factor_ = f * f * f;
  } else {
    float f = 1.0f - (use_highways * 2.0f);
    highway_factor_ = kMaxHighwayBiasFactor * (f * f);
  }

  // Preference for distance vs time
  distance_factor_ = costing_options.use_distance() * kInvMedianSpeed;
  inv_distance_factor_ = 1.f - costing_options.use_distance();

  // Preference to use toll roads (separate from toll booth penalty). Sets a toll
  // factor. A toll factor of 0 would indicate no adjustment to weighting for toll roads.
  // use_tolls = 1 would reduce weighting slightly (a negative delta) while
  // use_tolls = 0 would penalize (positive delta to weighting factor).
  float use_tolls = costing_options.use_tolls();
  toll_factor_ = use_tolls < 0.5f ? (4.0f - 8 * use_tolls) : // ranges from 4 to 0
                     (0.5f - use_tolls) * 0.03f;             // ranges from 0 to -0.15

  include_hot_ = costing_options.include_hot();
  include_hov2_ = costing_options.include_hov2();
  include_hov3_ = costing_options.include_hov3();

  // Get the vehicle attributes
  height_ = costing_options.height();
  width_ = costing_options.width();

  // Create speed cost table
  speedfactor_.resize(kMaxSpeedKph + 1, 0);
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
                       const bool is_dest,
                       const EdgeLabel& pred,
                       const graph_tile_ptr& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       uint8_t& restriction_idx) const {

  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes in case the origin is inside
  // a not thru region and a heading selected an edge entering the
  // region.
  if (!IsAccessible(edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((pred.restrictions() & (1 << edge->localedgeidx())) && !ignore_restrictions_) ||
      edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(edge, tile)) ||
      (exclude_unpaved_ && !pred.unpaved() && edge->unpaved()) || !IsHOVAllowed(edge)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, is_dest, tile, edgeid, current_time,
                                           tz_index, restriction_idx);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool AutoCost::AllowedReverse(const baldr::DirectedEdge* edge,
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
      opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(opp_edge, tile)) ||
      (exclude_unpaved_ && !pred.unpaved() && opp_edge->unpaved()) || !IsHOVAllowed(opp_edge)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, false, tile, opp_edgeid, current_time,
                                           tz_index, restriction_idx);
}

bool AutoCost::ModeSpecificAllowed(const baldr::AccessRestriction& restriction) const {
  switch (restriction.type()) {
    case AccessType::kMaxHeight:
      return height_ <= static_cast<float>(restriction.value() * 0.01);
    case AccessType::kMaxWidth:
      return width_ <= static_cast<float>(restriction.value() * 0.01);
    default:
      return true;
  };

  // Unreachable line
  return true;
}

// Get the cost to traverse the edge in seconds
Cost AutoCost::EdgeCost(const baldr::DirectedEdge* edge,
                        const graph_tile_ptr& tile,
                        const uint32_t seconds,
                        uint8_t& flow_sources) const {
  // either the computed edge speed or optional top_speed
  auto edge_speed = tile->GetSpeed(edge, flow_mask_, seconds, false, &flow_sources);
  auto final_speed = std::min(edge_speed, top_speed_);
  float sec = edge->length() * speedfactor_[final_speed];

  if (shortest_) {
    return Cost(edge->length(), sec);
  }

  // base factor is either ferry, rail ferry or density based
  float factor = 1;
  switch (edge->use()) {
    case Use::kFerry:
      factor = ferry_factor_;
      break;
    case Use::kRailFerry:
      factor = rail_ferry_factor_;
      break;
    default:
      factor = density_factor_[edge->density()];
      break;
  }

  // TODO: speed_penality hasn't been extensively tested, might alter this in future
  float speed_penalty = (edge_speed > top_speed_) ? (edge_speed - top_speed_) * 0.05f : 0.0f;
  factor += highway_factor_ * kHighwayFactor[static_cast<uint32_t>(edge->classification())] +
            surface_factor_ * kSurfaceFactor[static_cast<uint32_t>(edge->surface())] + speed_penalty +
            edge->toll() * toll_factor_;

  switch (edge->use()) {
    case Use::kAlley:
      factor *= alley_factor_;
      break;
    case Use::kTrack:
      factor *= track_factor_;
      break;
    case Use::kLivingStreet:
      factor *= living_street_factor_;
      break;
    case Use::kServiceRoad:
      factor *= service_factor_;
      break;
    case Use::kTurnChannel:
      if (flow_sources & kDefaultFlowMask) {
        // boost only historic & live speeds
        factor *= kTurnChannelFactor;
      }
      break;
    default:
      break;
  }

  if (IsClosed(edge, tile)) {
    // Add a penalty for traversing a closed edge
    factor *= closure_factor_;
  }
  // base cost before the factor is a linear combination of time vs distance, depending on which
  // one the user thinks is more important to them
  return Cost((sec * inv_distance_factor_ + edge->length() * distance_factor_) * factor, sec);
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost AutoCost::TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred) const {
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = base_transition_cost(node, edge, &pred, idx);
  c.secs = OSRMCarTurnDuration(edge, node, pred.opp_local_idx());

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

    AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, true, pred.internal_turn(),
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

  // Account for the user preferring distance
  c.cost *= inv_distance_factor_;

  return c;
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// pred is the opposing current edge in the reverse tree
// edge is the opposing predecessor in the reverse tree
Cost AutoCost::TransitionCostReverse(const uint32_t idx,
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

    AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, true, internal_turn, seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!has_measured_speed) {
      if (!is_turn)
        seconds *= edge->stopimpact(idx);
      seconds *= trans_density_factor_[node->density()];
    }
    c.cost += seconds;
  }

  // Account for the user preferring distance
  c.cost *= inv_distance_factor_;

  return c;
}

void ParseAutoCostOptions(const rapidjson::Document& doc,
                          const std::string& costing_options_key,
                          CostingOptions* pbf_costing_options) {
  pbf_costing_options->set_costing(Costing::auto_);
  pbf_costing_options->set_name(Costing_Enum_Name(pbf_costing_options->costing()));
  auto json_costing_options = rapidjson::get_child_optional(doc, costing_options_key.c_str());

  if (json_costing_options) {
    ParseSharedCostOptions(*json_costing_options, pbf_costing_options);
    ParseBaseCostOptions(*json_costing_options, pbf_costing_options, kBaseCostOptsConfig);

    // If specified, parse json and set pbf values

    // type (transport_type)
    pbf_costing_options->set_transport_type(
        rapidjson::get_optional<std::string>(*json_costing_options, "/type").get_value_or("car"));

    // alley_factor
    pbf_costing_options->set_alley_factor(
        kAlleyFactorRange(rapidjson::get_optional<float>(*json_costing_options, "/alley_factor")
                              .get_value_or(kDefaultAlleyFactor)));

    // use_highways
    pbf_costing_options->set_use_highways(
        kUseHighwaysRange(rapidjson::get_optional<float>(*json_costing_options, "/use_highways")
                              .get_value_or(kDefaultUseHighways)));

    // use_tolls
    pbf_costing_options->set_use_tolls(
        kUseTollsRange(rapidjson::get_optional<float>(*json_costing_options, "/use_tolls")
                           .get_value_or(kDefaultUseTolls)));

    // use distance
    pbf_costing_options->set_use_distance(
        kUseDistanceRange(rapidjson::get_optional<float>(*json_costing_options, "/use_distance")
                              .get_value_or(kDefaultUseDistance)));

    // height
    pbf_costing_options->set_height(
        kAutoHeightRange(rapidjson::get_optional<float>(*json_costing_options, "/height")
                             .get_value_or(kDefaultAutoHeight)));

    // width
    pbf_costing_options->set_width(
        kAutoWidthRange(rapidjson::get_optional<float>(*json_costing_options, "/width")
                            .get_value_or(kDefaultAutoWidth)));

    // HOT/HOV-use
    pbf_costing_options->set_include_hot(
        rapidjson::get_optional<bool>(*json_costing_options, "/include_hot").get_value_or(false));
    pbf_costing_options->set_include_hov2(
        rapidjson::get_optional<bool>(*json_costing_options, "/include_hov2").get_value_or(false));
    pbf_costing_options->set_include_hov3(
        rapidjson::get_optional<bool>(*json_costing_options, "/include_hov3").get_value_or(false));
  } else {
    SetDefaultBaseCostOptions(pbf_costing_options, kBaseCostOptsConfig);
    // Set pbf values to defaults
    pbf_costing_options->set_transport_type("car");
    pbf_costing_options->set_alley_factor(kDefaultAlleyFactor);
    pbf_costing_options->set_use_highways(kDefaultUseHighways);
    pbf_costing_options->set_use_tolls(kDefaultUseTolls);
    pbf_costing_options->set_flow_mask(kDefaultFlowMask);
    pbf_costing_options->set_top_speed(kMaxAssumedSpeed);
    pbf_costing_options->set_use_distance(kDefaultUseDistance);
    pbf_costing_options->set_height(kDefaultAutoHeight);
    pbf_costing_options->set_width(kDefaultAutoWidth);
  }
}

cost_ptr_t CreateAutoCost(const CostingOptions& costing_options) {
  return std::make_shared<AutoCost>(costing_options);
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
  BusCost(const CostingOptions& costing_options) : AutoCost(costing_options, kBusAccess) {
    type_ = VehicleType::kBus;
  }

  /// virtual destructor
  virtual ~BusCost() {
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
};

// Check if access is allowed on the specified edge.
bool BusCost::Allowed(const baldr::DirectedEdge* edge,
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
      edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(edge, tile)) ||
      (exclude_unpaved_ && !pred.unpaved() && edge->unpaved())) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, is_dest, tile, edgeid, current_time,
                                           tz_index, restriction_idx);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool BusCost::AllowedReverse(const baldr::DirectedEdge* edge,
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
      opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(opp_edge, tile)) ||
      (exclude_unpaved_ && !pred.unpaved() && opp_edge->unpaved())) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, false, tile, opp_edgeid, current_time,
                                           tz_index, restriction_idx);
}

void ParseBusCostOptions(const rapidjson::Document& doc,
                         const std::string& costing_options_key,
                         CostingOptions* pbf_costing_options) {
  ParseAutoCostOptions(doc, costing_options_key, pbf_costing_options);
  pbf_costing_options->set_costing(Costing::bus);
  pbf_costing_options->set_name(Costing_Enum_Name(pbf_costing_options->costing()));
}

cost_ptr_t CreateBusCost(const CostingOptions& costing_options) {
  return std::make_shared<BusCost>(costing_options);
}

/**
 * Derived class providing an alternate costing for driving that is intended
 * to favor Taxi roads.
 */
class TaxiCost : public AutoCost {
public:
  /**
   * Construct taxi costing.
   * Pass in costing_options using protocol buffer(pbf).
   * @param  costing_options  pbf with costing_options.
   */
  TaxiCost(const CostingOptions& costing_options) : AutoCost(costing_options, kTaxiAccess) {
  }

  virtual ~TaxiCost() {
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
   * Returns the cost to traverse the edge and an estimate of the actual time
   * (in seconds) to traverse the edge.
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  seconds   Time of week in seconds.
   * @return  Returns the cost to traverse the edge.
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const graph_tile_ptr& tile,
                        const uint32_t seconds,
                        uint8_t& flow_sources) const override {
    auto edge_speed = tile->GetSpeed(edge, flow_mask_, seconds, false, &flow_sources);
    auto final_speed = std::min(edge_speed, top_speed_);

    float sec = (edge->length() * speedfactor_[final_speed]);

    if (shortest_) {
      return Cost(edge->length(), sec);
    }

    float factor = (edge->use() == Use::kFerry) ? ferry_factor_ : density_factor_[edge->density()];
    float speed_penalty = (edge_speed > top_speed_) ? (edge_speed - top_speed_) * 0.05f : 0.0f;
    factor += speed_penalty;
    if ((edge->forwardaccess() & kTaxiAccess) && !(edge->forwardaccess() & kAutoAccess)) {
      factor *= kTaxiFactor;
    }

    if (edge->use() == Use::kAlley) {
      factor *= alley_factor_;
    } else if (edge->use() == Use::kTrack) {
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

    return Cost(sec * factor, sec);
  }
};

// Check if access is allowed on the specified edge.
bool TaxiCost::Allowed(const baldr::DirectedEdge* edge,
                       const bool is_dest,
                       const EdgeLabel& pred,
                       const graph_tile_ptr& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       uint8_t& restriction_idx) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes in case the origin is inside
  // a not thru region and a heading selected an edge entering the
  // region.
  if (!IsAccessible(edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((pred.restrictions() & (1 << edge->localedgeidx())) && !ignore_restrictions_) ||
      edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(edge, tile)) ||
      (exclude_unpaved_ && !pred.unpaved() && edge->unpaved())) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, is_dest, tile, edgeid, current_time,
                                           tz_index, restriction_idx);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool TaxiCost::AllowedReverse(const baldr::DirectedEdge* edge,
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
      opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(opp_edge, tile)) ||
      (exclude_unpaved_ && !pred.unpaved() && opp_edge->unpaved())) {
    return false;
  }
  return DynamicCost::EvaluateRestrictions(access_mask_, edge, false, tile, opp_edgeid, current_time,
                                           tz_index, restriction_idx);
}

void ParseTaxiCostOptions(const rapidjson::Document& doc,
                          const std::string& costing_options_key,
                          CostingOptions* pbf_costing_options) {
  ParseAutoCostOptions(doc, costing_options_key, pbf_costing_options);
  pbf_costing_options->set_costing(Costing::taxi);
  pbf_costing_options->set_name(Costing_Enum_Name(pbf_costing_options->costing()));
}

cost_ptr_t CreateTaxiCost(const CostingOptions& costing_options) {
  return std::make_shared<TaxiCost>(costing_options);
}

} // namespace sif
} // namespace valhalla

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

class TestAutoCost : public AutoCost {
public:
  TestAutoCost(const CostingOptions& costing_options) : AutoCost(costing_options){};

  using AutoCost::alley_penalty_;
  using AutoCost::country_crossing_cost_;
  using AutoCost::destination_only_penalty_;
  using AutoCost::ferry_transition_cost_;
  using AutoCost::flow_mask_;
  using AutoCost::gate_cost_;
  using AutoCost::height_;
  using AutoCost::maneuver_penalty_;
  using AutoCost::service_factor_;
  using AutoCost::service_penalty_;
  using AutoCost::toll_booth_cost_;
  using AutoCost::width_;
};

template <class T>
std::shared_ptr<TestAutoCost>
make_autocost_from_json(const std::string& property, T testVal, const std::string& extra_json = "") {
  std::stringstream ss;
  ss << R"({"costing_options":{"auto":{")" << property << R"(":)" << testVal << "}}" << extra_json
     << "}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return std::make_shared<TestAutoCost>(
      request.options().costing_options(static_cast<int>(Costing::auto_)));
}

std::uniform_real_distribution<float>
make_distributor_from_range(const ranged_default_t<float>& range) {
  float rangeLength = range.max - range.min;
  return std::uniform_real_distribution<float>(range.min - rangeLength, range.max + rangeLength);
}

TEST(AutoCost, testAutoCostParams) {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::uniform_real_distribution<float> distributor;
  std::shared_ptr<TestAutoCost> tester;

  const auto& defaults = kBaseCostOptsConfig;

  // maneuver_penalty_
  distributor = make_distributor_from_range(defaults.maneuver_penalty_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("maneuver_penalty", distributor(generator));
    EXPECT_THAT(tester->maneuver_penalty_,
                test::IsBetween(defaults.maneuver_penalty_.min, defaults.maneuver_penalty_.max));
  }

  // alley_penalty_
  distributor = make_distributor_from_range(defaults.alley_penalty_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("alley_penalty", distributor(generator));
    EXPECT_THAT(tester->alley_penalty_,
                test::IsBetween(defaults.alley_penalty_.min, defaults.alley_penalty_.max));
  }

  // alley_factor_
  distributor = make_distributor_from_range(kAlleyFactorRange);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("alley_factor", distributor(generator));
    EXPECT_THAT(tester->alley_factor_, test::IsBetween(kAlleyFactorRange.min, kAlleyFactorRange.max));
  }

  // destination_only_penalty_
  distributor = make_distributor_from_range(defaults.dest_only_penalty_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("destination_only_penalty", distributor(generator));
    EXPECT_THAT(tester->destination_only_penalty_,
                test::IsBetween(defaults.dest_only_penalty_.min, defaults.dest_only_penalty_.max));
  }

  // gate_cost_ (Cost.secs)
  distributor = make_distributor_from_range(defaults.gate_cost_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("gate_cost", distributor(generator));
    EXPECT_THAT(tester->gate_cost_.secs,
                test::IsBetween(defaults.gate_cost_.min, defaults.gate_cost_.max));
  }

  // gate_penalty_ (Cost.cost)
  distributor = make_distributor_from_range(defaults.gate_penalty_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("gate_penalty", distributor(generator));
    EXPECT_THAT(tester->gate_cost_.cost,
                test::IsBetween(defaults.gate_penalty_.min, defaults.gate_penalty_.max));
  }

  // tollbooth_cost_ (Cost.secs)
  distributor = make_distributor_from_range(defaults.toll_booth_cost_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("toll_booth_cost", distributor(generator));
    EXPECT_THAT(tester->toll_booth_cost_.secs,
                test::IsBetween(defaults.toll_booth_cost_.min, defaults.toll_booth_cost_.max));
  }

  // tollbooth_penalty_ (Cost.cost)
  distributor = make_distributor_from_range(defaults.toll_booth_penalty_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("toll_booth_penalty", distributor(generator));
    EXPECT_THAT(tester->toll_booth_cost_.cost,
                test::IsBetween(defaults.toll_booth_penalty_.min,
                                defaults.toll_booth_penalty_.max + defaults.toll_booth_cost_.def));
  }

  // country_crossing_cost_ (Cost.secs)
  distributor = make_distributor_from_range(defaults.country_crossing_cost_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("country_crossing_cost", distributor(generator));
    EXPECT_THAT(tester->country_crossing_cost_.secs,
                test::IsBetween(defaults.country_crossing_cost_.min,
                                defaults.country_crossing_cost_.max));
  }

  // country_crossing_penalty_ (Cost.cost)
  distributor = make_distributor_from_range(defaults.country_crossing_penalty_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("country_crossing_penalty", distributor(generator));
    EXPECT_THAT(tester->country_crossing_cost_.cost,
                test::IsBetween(defaults.country_crossing_penalty_.min,
                                defaults.country_crossing_penalty_.max +
                                    defaults.country_crossing_cost_.def));
  }

  // ferry_cost_ (Cost.secs)
  distributor = make_distributor_from_range(defaults.ferry_cost_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("ferry_cost", distributor(generator));
    EXPECT_THAT(tester->ferry_transition_cost_.secs,
                test::IsBetween(defaults.ferry_cost_.min, defaults.ferry_cost_.max));
  }

  /**
   // use_ferry
   distributor = make_distributor_from_range(defaults.use_ferry_);
   for (unsigned i = 0; i < testIterations; ++i) {
     tester = make_autocost_from_json("use_ferry", distributor(generator));
     EXPECT_THAT(tester->use_ferry, test::IsBetween(
       defaults.use_ferry_.min, defaults.use_ferry_.max));
   }
 **/

  // service_penalty_
  distributor = make_distributor_from_range(defaults.service_penalty_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("service_penalty", distributor(generator));
    EXPECT_THAT(tester->service_penalty_,
                test::IsBetween(defaults.service_penalty_.min, defaults.service_penalty_.max));
  }

  // service_factor_
  distributor = make_distributor_from_range(defaults.service_factor_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("service_factor", distributor(generator));
    EXPECT_THAT(tester->service_factor_,
                test::IsBetween(defaults.service_factor_.min, defaults.service_factor_.max));
  }

  // height_
  distributor = make_distributor_from_range(defaults.height_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("height", distributor(generator));
    EXPECT_THAT(tester->height_, test::IsBetween(defaults.height_.min, defaults.height_.max));
  }

  // width_
  distributor = make_distributor_from_range(defaults.width_);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("width", distributor(generator));
    EXPECT_THAT(tester->width_, test::IsBetween(defaults.width_.min, defaults.width_.max));
  }

  // flow_mask_
  using tc = std::tuple<std::string, std::string, uint8_t>;
  std::vector<tc>
      speed_type_test_cases{tc{"", R"(,"date_time":{"type":0})", kDefaultFlowMask},
                            tc{R"("")", R"(,"date_time":{"type":0})", kDefaultFlowMask},
                            tc{"[]", R"(,"date_time":{"type":0})", 0},
                            tc{R"(["foo"])", R"(,"date_time":{"type":0})", 0},
                            tc{R"(["freeflow"])", R"(,"date_time":{"type":0})", kFreeFlowMask},
                            tc{R"(["constrained"])", R"(,"date_time":{"type":0})",
                               kConstrainedFlowMask},
                            tc{R"(["predicted"])", R"(,"date_time":{"type":0})", kPredictedFlowMask},
                            tc{R"(["current"])", R"(,"date_time":{"type":0})", kCurrentFlowMask},
                            tc{R"(["freeflow","current","predicted"])", R"(,"date_time":{"type":0})",
                               kFreeFlowMask | kCurrentFlowMask | kPredictedFlowMask},
                            tc{R"(["freeflow","constrained","predicted","current"])",
                               R"(,"date_time":{"type":0})", kDefaultFlowMask},
                            tc{R"(["constrained","foo","predicted","freeflow"])",
                               R"(,"date_time":{"type":0})",
                               kFreeFlowMask | kConstrainedFlowMask | kPredictedFlowMask},

                            tc{"", "", kFreeFlowMask | kConstrainedFlowMask},
                            tc{R"("")", "", kFreeFlowMask | kConstrainedFlowMask},
                            tc{"[]", "", 0},
                            tc{R"(["foo"])", "", 0},
                            tc{R"(["freeflow"])", "", kFreeFlowMask},
                            tc{R"(["constrained"])", "", kConstrainedFlowMask},
                            tc{R"(["predicted"])", "", 0},
                            tc{R"(["current"])", "", 0},
                            tc{R"(["freeflow","current","predicted"])", "", kFreeFlowMask},
                            tc{R"(["constrained","current","predicted"])", "", kConstrainedFlowMask},
                            tc{R"(["freeflow","constrained","predicted","current"])", "",
                               kFreeFlowMask | kConstrainedFlowMask},
                            tc{R"(["constrained","foo","predicted","freeflow"])", "",
                               kFreeFlowMask | kConstrainedFlowMask}};

  for (size_t i = 0; i < speed_type_test_cases.size(); ++i) {
    auto value = std::get<0>(speed_type_test_cases[i]);
    auto extra_json = std::get<1>(speed_type_test_cases[i]);
    auto expected = std::get<2>(speed_type_test_cases[i]);

    std::string key("speed_types");
    if (value.empty()) {
      key = "foo";
      value = R"("bar")";
    }

    tester = make_autocost_from_json(key, value, extra_json);
    EXPECT_EQ(tester->flow_mask_, expected);
  }
}
} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
