#include "sif/autocost.h"
#include "sif/costconstants.h"

#include "baldr/accessrestriction.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "midgard/constants.h"
#include "midgard/util.h"
#include "sif/dynamiccost.h"

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
constexpr float kDefaultAlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultGateCost = 30.0f;                // Seconds
constexpr float kDefaultGatePenalty = 300.0f;            // Seconds
constexpr float kDefaultTollBoothCost = 15.0f;           // Seconds
constexpr float kDefaultTollBoothPenalty = 0.0f;         // Seconds
constexpr float kDefaultFerryCost = 300.0f;              // Seconds
constexpr float kDefaultCountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultCountryCrossingPenalty = 0.0f;   // Seconds

// Other options
constexpr float kDefaultUseFerry = 0.5f;    // Factor between 0 and 1
constexpr float kDefaultUseHighways = 1.0f; // Factor between 0 and 1
constexpr float kDefaultUseTolls = 0.5f;    // Factor between 0 and 1

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

// How much to favor taxi roads.
constexpr float kTaxiFactor = 0.85f;

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = {kTCStraight,       kTCSlight,  kTCFavorable,
                                         kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
                                         kTCUnfavorable,    kTCSlight};
constexpr float kLeftSideTurnCosts[] = {kTCStraight,         kTCSlight,  kTCUnfavorable,
                                        kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
                                        kTCFavorable,        kTCSlight};

// Valid ranges and defaults
constexpr ranged_default_t<float> kManeuverPenaltyRange{0, kDefaultManeuverPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kDestinationOnlyPenaltyRange{0, kDefaultDestinationOnlyPenalty,
                                                               kMaxPenalty};
constexpr ranged_default_t<float> kAlleyPenaltyRange{0, kDefaultAlleyPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kGateCostRange{0, kDefaultGateCost, kMaxPenalty};
constexpr ranged_default_t<float> kGatePenaltyRange{0, kDefaultGatePenalty, kMaxPenalty};
constexpr ranged_default_t<float> kTollBoothCostRange{0, kDefaultTollBoothCost, kMaxPenalty};
constexpr ranged_default_t<float> kTollBoothPenaltyRange{0, kDefaultTollBoothPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kFerryCostRange{0, kDefaultFerryCost, kMaxPenalty};
constexpr ranged_default_t<float> kCountryCrossingCostRange{0, kDefaultCountryCrossingCost,
                                                            kMaxPenalty};
constexpr ranged_default_t<float> kCountryCrossingPenaltyRange{0, kDefaultCountryCrossingPenalty,
                                                               kMaxPenalty};
constexpr ranged_default_t<float> kUseFerryRange{0, kDefaultUseFerry, 1.0f};
constexpr ranged_default_t<float> kUseHighwaysRange{0, kDefaultUseHighways, 1.0f};
constexpr ranged_default_t<float> kUseTollsRange{0, kDefaultUseTolls, 1.0f};

constexpr float kHighwayFactor[] = {
    10.0f, // Motorway
    0.5f,  // Trunk
    0.0f,  // Primary
    0.0f,  // Secondary
    0.0f,  // Tertiary
    0.0f,  // Unclassified
    0.0f,  // Residential
    0.0f   // Service, other
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
   * Construct auto costing. Pass in cost type and options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  options pbf with request options.
   */
  AutoCost(const Costing costing, const Options& options);

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
                       const uint32_t tz_index,
                       bool& has_time_restrictions) const;

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
   * be restricted if bollards or gates are present.
   * @param  node  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const {
    return (node->access() & kAutoAccess);
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
      if (edge->is_shortcut() || !(edge->forwardaccess() & kAutoAccess)) {
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
  float density_factor_[16]; // Density factor
  float highway_factor_;     // Factor applied when road is a motorway or trunk
  float toll_factor_;        // Factor applied when road has a toll
  float surface_factor_;     // How much the surface factors are applied.

  // Density factor used in edge transition costing
  std::vector<float> trans_density_factor_;
};

// Constructor
AutoCost::AutoCost(const Costing costing, const Options& options)
    : DynamicCost(options, TravelMode::kDrive), trans_density_factor_{1.0f, 1.0f, 1.0f, 1.0f,
                                                                      1.0f, 1.1f, 1.2f, 1.3f,
                                                                      1.4f, 1.6f, 1.9f, 2.2f,
                                                                      2.5f, 2.8f, 3.1f, 3.5f} {

  // Grab the costing options based on the specified costing type
  const CostingOptions& costing_options = options.costing_options(static_cast<int>(costing));

  // Get the vehicle type - enter as string and convert to enum.
  // Used to set the surface factor - penalize some roads based on surface type.
  surface_factor_ = 0.5f;
  std::string type = costing_options.transport_type();
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

  // Preference to use highways. Is a value from 0 to 1
  float use_highways_ = costing_options.use_highways();
  highway_factor_ = 1.0f - use_highways_;

  // Preference to use toll roads (separate from toll booth penalty). Sets a toll
  // factor. A toll factor of 0 would indicate no adjustment to weighting for toll roads.
  // use_tolls = 1 would reduce weighting slightly (a negative delta) while
  // use_tolls = 0 would penalize (positive delta to weighting factor).
  float use_tolls = costing_options.use_tolls();
  toll_factor_ = use_tolls < 0.5f ? (4.0f - 8 * use_tolls) : // ranges from 4 to 0
                     (0.5f - use_tolls) * 0.03f;             // ranges from 0 to -0.15

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
                       const uint32_t tz_index,
                       bool& has_time_restrictions) const {
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

  return DynamicCost::EvaluateRestrictions(kAutoAccess, edge, tile, edgeid, current_time, tz_index,
                                           has_time_restrictions);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool AutoCost::AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const baldr::GraphTile*& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index,
                              bool& has_time_restrictions) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(opp_edge->forwardaccess() & kAutoAccess) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
      opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly())) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(kAutoAccess, edge, tile, opp_edgeid, current_time,
                                           tz_index, has_time_restrictions);
}

// Get the cost to traverse the edge in seconds
Cost AutoCost::EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphTile* tile,
                        const uint32_t seconds) const {
  auto speed = tile->GetSpeed(edge, flow_mask_, seconds);
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
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = base_transition_cost(node, edge, pred, idx);

  // Intersection transition time = factor * stopimpact * turncost. Factor depends
  // on density and whether traffic is available
  if (edge->stopimpact(idx) > 0) {
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

    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    float seconds = turn_cost * edge->stopimpact(idx);
    // Apply density factor penality if there isnt traffic on this edge or youre not using traffic
    if (!edge->has_flow_speed() || flow_mask_ == 0)
      seconds *= trans_density_factor_[node->density()];

    c.cost += seconds;
    c.secs += seconds;
  }
  return c;
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// pred is the opposing current edge in the reverse tree
// edge is the opposing predecessor in the reverse tree
Cost AutoCost::TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* pred,
                                     const baldr::DirectedEdge* edge) const {
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  Cost c = base_transition_cost(node, edge, pred, idx);

  // Transition time = densityfactor * stopimpact * turncost
  if (edge->stopimpact(idx) > 0) {
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

    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    float seconds = turn_cost * edge->stopimpact(idx);
    // Apply density factor penality if there isnt traffic on this edge or youre not using traffic
    if (!edge->has_flow_speed() || flow_mask_ == 0)
      seconds *= trans_density_factor_[node->density()];

    c.secs += seconds;
    c.cost += seconds;
  }
  return c;
}

void ParseAutoCostOptions(const rapidjson::Document& doc,
                          const std::string& costing_options_key,
                          CostingOptions* pbf_costing_options) {
  auto json_costing_options = rapidjson::get_child_optional(doc, costing_options_key.c_str());

  if (json_costing_options) {
    // TODO: farm more common stuff out to parent class
    ParseCostOptions(*json_costing_options, pbf_costing_options);

    // If specified, parse json and set pbf values

    // type (transport_type)
    pbf_costing_options->set_transport_type(
        rapidjson::get_optional<std::string>(*json_costing_options, "/type").get_value_or("car"));

    // maneuver_penalty
    pbf_costing_options->set_maneuver_penalty(kManeuverPenaltyRange(
        rapidjson::get_optional<float>(*json_costing_options, "/maneuver_penalty")
            .get_value_or(kDefaultManeuverPenalty)));

    // destination_only_penalty
    pbf_costing_options->set_destination_only_penalty(kDestinationOnlyPenaltyRange(
        rapidjson::get_optional<float>(*json_costing_options, "/destination_only_penalty")
            .get_value_or(kDefaultDestinationOnlyPenalty)));

    // gate_cost
    pbf_costing_options->set_gate_cost(
        kGateCostRange(rapidjson::get_optional<float>(*json_costing_options, "/gate_cost")
                           .get_value_or(kDefaultGateCost)));

    // gate_penalty
    pbf_costing_options->set_gate_penalty(
        kGatePenaltyRange(rapidjson::get_optional<float>(*json_costing_options, "/gate_penalty")
                              .get_value_or(kDefaultGatePenalty)));

    // toll_booth_cost
    pbf_costing_options->set_toll_booth_cost(
        kTollBoothCostRange(rapidjson::get_optional<float>(*json_costing_options, "/toll_booth_cost")
                                .get_value_or(kDefaultTollBoothCost)));

    // toll_booth_penalty
    pbf_costing_options->set_toll_booth_penalty(kTollBoothPenaltyRange(
        rapidjson::get_optional<float>(*json_costing_options, "/toll_booth_penalty")
            .get_value_or(kDefaultTollBoothPenalty)));

    // alley_penalty
    pbf_costing_options->set_alley_penalty(
        kAlleyPenaltyRange(rapidjson::get_optional<float>(*json_costing_options, "/alley_penalty")
                               .get_value_or(kDefaultAlleyPenalty)));

    // country_crossing_cost
    pbf_costing_options->set_country_crossing_cost(kCountryCrossingCostRange(
        rapidjson::get_optional<float>(*json_costing_options, "/country_crossing_cost")
            .get_value_or(kDefaultCountryCrossingCost)));

    // country_crossing_penalty
    pbf_costing_options->set_country_crossing_penalty(kCountryCrossingPenaltyRange(
        rapidjson::get_optional<float>(*json_costing_options, "/country_crossing_penalty")
            .get_value_or(kDefaultCountryCrossingPenalty)));

    // ferry_cost
    pbf_costing_options->set_ferry_cost(
        kFerryCostRange(rapidjson::get_optional<float>(*json_costing_options, "/ferry_cost")
                            .get_value_or(kDefaultFerryCost)));

    // use_ferry
    pbf_costing_options->set_use_ferry(
        kUseFerryRange(rapidjson::get_optional<float>(*json_costing_options, "/use_ferry")
                           .get_value_or(kDefaultUseFerry)));

    // use_highways
    pbf_costing_options->set_use_highways(
        kUseHighwaysRange(rapidjson::get_optional<float>(*json_costing_options, "/use_highways")
                              .get_value_or(kDefaultUseHighways)));

    // use_tolls
    pbf_costing_options->set_use_tolls(
        kUseTollsRange(rapidjson::get_optional<float>(*json_costing_options, "/use_tolls")
                           .get_value_or(kDefaultUseTolls)));
  } else {
    // Set pbf values to defaults
    pbf_costing_options->set_transport_type("car");
    pbf_costing_options->set_maneuver_penalty(kDefaultManeuverPenalty);
    pbf_costing_options->set_destination_only_penalty(kDefaultDestinationOnlyPenalty);
    pbf_costing_options->set_gate_cost(kDefaultGateCost);
    pbf_costing_options->set_gate_penalty(kDefaultGatePenalty);
    pbf_costing_options->set_toll_booth_cost(kDefaultTollBoothCost);
    pbf_costing_options->set_toll_booth_penalty(kDefaultTollBoothPenalty);
    pbf_costing_options->set_alley_penalty(kDefaultAlleyPenalty);
    pbf_costing_options->set_country_crossing_cost(kDefaultCountryCrossingCost);
    pbf_costing_options->set_country_crossing_penalty(kDefaultCountryCrossingPenalty);
    pbf_costing_options->set_ferry_cost(kDefaultFerryCost);
    pbf_costing_options->set_use_ferry(kDefaultUseFerry);
    pbf_costing_options->set_use_highways(kDefaultUseHighways);
    pbf_costing_options->set_use_tolls(kDefaultUseTolls);
    pbf_costing_options->set_flow_mask(kDefaultFlowMask);
  }
}

cost_ptr_t CreateAutoCost(const Costing costing, const Options& options) {
  return std::make_shared<AutoCost>(costing, options);
}

/**
 * Derived class providing an alternate costing for driving that is intended
 * to provide a short path.
 */
class AutoShorterCost : public AutoCost {
public:
  /**
   * Construct auto costing for shorter (not absolute shortest) path.
   * Pass in options with protocol buffer(pbf).
   * @param  options  pbf with options.
   */
  AutoShorterCost(const Costing costing, const Options& options) : AutoCost(costing, options) {
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
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  seconds   Time of week in seconds.
   * @return  Returns the cost to traverse the edge.
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphTile* tile,
                        const uint32_t seconds) const {
    auto speed = tile->GetSpeed(edge, flow_mask_, seconds);
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

void ParseAutoShorterCostOptions(const rapidjson::Document& doc,
                                 const std::string& costing_options_key,
                                 CostingOptions* pbf_costing_options) {
  ParseAutoCostOptions(doc, costing_options_key, pbf_costing_options);
}

cost_ptr_t CreateAutoShorterCost(const Costing costing, const Options& options) {
  return std::make_shared<AutoShorterCost>(costing, options);
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
  BusCost(const Costing costing, const Options& options) : AutoCost(costing, options) {
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
                       const uint32_t tz_index,
                       bool& has_time_restrictions) const;

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
      if (!(edge->forwardaccess() & kBusAccess)) {
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
                      const uint32_t tz_index,
                      bool& has_time_restrictions) const {
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

  return DynamicCost::EvaluateRestrictions(kBusAccess, edge, tile, edgeid, current_time, tz_index,
                                           has_time_restrictions);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool BusCost::AllowedReverse(const baldr::DirectedEdge* edge,
                             const EdgeLabel& pred,
                             const baldr::DirectedEdge* opp_edge,
                             const baldr::GraphTile*& tile,
                             const baldr::GraphId& opp_edgeid,
                             const uint64_t current_time,
                             const uint32_t tz_index,
                             bool& has_time_restrictions) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(opp_edge->forwardaccess() & kBusAccess) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
      opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly())) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(kBusAccess, edge, tile, opp_edgeid, current_time, tz_index,
                                           has_time_restrictions);
}

void ParseBusCostOptions(const rapidjson::Document& doc,
                         const std::string& costing_options_key,
                         CostingOptions* pbf_costing_options) {
  ParseAutoCostOptions(doc, costing_options_key, pbf_costing_options);
}

cost_ptr_t CreateBusCost(const Costing costing, const Options& options) {
  return std::make_shared<BusCost>(costing, options);
}

/**
 * Derived class providing an alternate costing for driving that is intended
 * to favor HOV roads.
 */
class HOVCost : public AutoCost {
public:
  /**
   * Construct hov costing.
   * Pass in options using protocol buffer(pbf).
   * @param  options  pbf with options.
   */
  HOVCost(const Costing costing, const Options& options) : AutoCost(costing, options) {
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
                       const uint32_t tz_index,
                       bool& has_time_restrictions) const;

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
                              const uint32_t tz_index,
                              bool& has_time_restrictions) const;

  /**
   * Returns the cost to traverse the edge and an estimate of the actual time
   * (in seconds) to traverse the edge.
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  seconds   Time of week in seconds.
   * @return  Returns the cost to traverse the edge.
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphTile* tile,
                        const uint32_t seconds) const {
    auto speed = tile->GetSpeed(edge, flow_mask_, seconds);
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
      if (!(edge->forwardaccess() & kHOVAccess)) {
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
                      const uint32_t tz_index,
                      bool& has_time_restrictions) const {
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

  return DynamicCost::EvaluateRestrictions(kHOVAccess, edge, tile, edgeid, current_time, tz_index,
                                           has_time_restrictions);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool HOVCost::AllowedReverse(const baldr::DirectedEdge* edge,
                             const EdgeLabel& pred,
                             const baldr::DirectedEdge* opp_edge,
                             const baldr::GraphTile*& tile,
                             const baldr::GraphId& opp_edgeid,
                             const uint64_t current_time,
                             const uint32_t tz_index,
                             bool& has_time_restrictions) const {
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

  return DynamicCost::EvaluateRestrictions(kHOVAccess, edge, tile, opp_edgeid, current_time, tz_index,
                                           has_time_restrictions);
}

void ParseHOVCostOptions(const rapidjson::Document& doc,
                         const std::string& costing_options_key,
                         CostingOptions* pbf_costing_options) {
  ParseAutoCostOptions(doc, costing_options_key, pbf_costing_options);
}

cost_ptr_t CreateHOVCost(const Costing costing, const Options& options) {
  return std::make_shared<HOVCost>(costing, options);
}

/**
 * Derived class providing an alternate costing for driving that is intended
 * to favor Taxi roads.
 */
class TaxiCost : public AutoCost {
public:
  /**
   * Construct taxi costing.
   * Pass in options using protocol buffer(pbf).
   * @param  options  pbf with options.
   */
  TaxiCost(const Costing costing, const Options& options) : AutoCost(costing, options) {
  }

  virtual ~TaxiCost() {
  }

  /**
   * Get the access mode used by this costing method.
   * @return  Returns access mode.
   */
  uint32_t access_mode() const {
    return kTaxiAccess;
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
                       bool& has_time_restrictions) const;

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
                              const uint32_t tz_index,
                              bool& has_time_restrictions) const;

  /**
   * Returns the cost to traverse the edge and an estimate of the actual time
   * (in seconds) to traverse the edge.
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  seconds   Time of week in seconds.
   * @return  Returns the cost to traverse the edge.
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphTile* tile,
                        const uint32_t seconds) const {
    auto speed = tile->GetSpeed(edge, flow_mask_, seconds);
    float factor = (edge->use() == Use::kFerry) ? ferry_factor_ : density_factor_[edge->density()];
    if ((edge->forwardaccess() & kTaxiAccess) && !(edge->forwardaccess() & kAutoAccess)) {
      factor *= kTaxiFactor;
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
    return (node->access() & kTaxiAccess);
  }

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. Function/functor is also used to filter
   * edges not usable / inaccessible by taxi.
   */
  virtual const EdgeFilter GetEdgeFilter() const {
    // Throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge) {
      if (!(edge->forwardaccess() & kTaxiAccess)) {
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
    return [](const baldr::NodeInfo* node) { return !(node->access() & kTaxiAccess); };
  }
};

// Check if access is allowed on the specified edge.
bool TaxiCost::Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       bool& has_time_restrictions) const {
  // TODO - obtain and check the access restrictions.

  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes in case the origin is inside
  // a not thru region and a heading selected an edge entering the
  // region.
  if (!(edge->forwardaccess() & kTaxiAccess) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
      edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly())) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(kTaxiAccess, edge, tile, edgeid, current_time, tz_index,
                                           has_time_restrictions);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool TaxiCost::AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const baldr::GraphTile*& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index,
                              bool& has_time_restrictions) const {
  // TODO - obtain and check the access restrictions.

  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!(opp_edge->forwardaccess() & kTaxiAccess) ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
      opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly())) {
    return false;
  }
  return DynamicCost::EvaluateRestrictions(kHOVAccess, edge, tile, opp_edgeid, current_time, tz_index,
                                           has_time_restrictions);
}

void ParseTaxiCostOptions(const rapidjson::Document& doc,
                          const std::string& costing_options_key,
                          CostingOptions* pbf_costing_options) {
  ParseAutoCostOptions(doc, costing_options_key, pbf_costing_options);
}

cost_ptr_t CreateTaxiCost(const Costing costing, const Options& options) {
  return std::make_shared<TaxiCost>(costing, options);
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
  AutoDataFix(const Costing costing, const Options& options) : AutoCost(costing, options) {
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
                       const uint32_t tz_index,
                       bool& has_time_restrictions) const {
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
      // Do not allow edges with no auto access in either direction
      if (!((edge->forwardaccess() & kAutoAccess) || (edge->reverseaccess() & kAutoAccess))) {
        return 0.0f;
      } else {
        return 1.0f;
      }
    };
  }
};

void ParseAutoDataFixCostOptions(const rapidjson::Document& doc,
                                 const std::string& costing_options_key,
                                 CostingOptions* pbf_costing_options) {
  ParseAutoCostOptions(doc, costing_options_key, pbf_costing_options);
}

cost_ptr_t CreateAutoDataFixCost(const Costing costing, const Options& options) {
  return std::make_shared<AutoDataFix>(costing, options);
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
  TestAutoCost(const Costing costing, const Options& options) : AutoCost(costing, options){};

  using AutoCost::alley_penalty_;
  using AutoCost::country_crossing_cost_;
  using AutoCost::destination_only_penalty_;
  using AutoCost::ferry_transition_cost_;
  using AutoCost::flow_mask_;
  using AutoCost::gate_cost_;
  using AutoCost::maneuver_penalty_;
  using AutoCost::toll_booth_cost_;
};

template <class T>
std::shared_ptr<TestAutoCost>
make_autocost_from_json(const std::string& property, T testVal, const std::string& extra_json = "") {
  std::stringstream ss;
  ss << R"({"costing_options":{"auto":{")" << property << R"(":)" << testVal << "}}" << extra_json
     << "}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return std::make_shared<TestAutoCost>(valhalla::Costing::auto_, request.options());
}

std::uniform_real_distribution<float>
make_distributor_from_range(const ranged_default_t<float>& range) {
  float rangeLength = range.max - range.min;
  return std::uniform_real_distribution<float>(range.min - rangeLength, range.max + rangeLength);
}

void testAutoCostParams() {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::uniform_real_distribution<float> distributor;
  std::shared_ptr<TestAutoCost> tester;

  // maneuver_penalty_
  distributor = make_distributor_from_range(kManeuverPenaltyRange);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("maneuver_penalty", distributor(generator));
    if (tester->maneuver_penalty_ < kManeuverPenaltyRange.min ||
        tester->maneuver_penalty_ > kManeuverPenaltyRange.max) {
      throw std::runtime_error("maneuver_penalty_ is not within it's range");
    }
  }

  // alley_penalty_
  distributor = make_distributor_from_range(kAlleyPenaltyRange);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("alley_penalty", distributor(generator));
    if (tester->alley_penalty_ < kAlleyPenaltyRange.min ||
        tester->alley_penalty_ > kAlleyPenaltyRange.max) {
      throw std::runtime_error("alley_penalty_ is not within it's range");
    }
  }

  // destination_only_penalty_
  distributor = make_distributor_from_range(kDestinationOnlyPenaltyRange);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("destination_only_penalty", distributor(generator));
    if (tester->destination_only_penalty_ < kDestinationOnlyPenaltyRange.min ||
        tester->destination_only_penalty_ > kDestinationOnlyPenaltyRange.max) {
      throw std::runtime_error("destination_only_penalty_ is not within it's range");
    }
  }

  // gate_cost_ (Cost.secs)
  distributor = make_distributor_from_range(kGateCostRange);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("gate_cost", distributor(generator));
    if (tester->gate_cost_.secs < kGateCostRange.min ||
        tester->gate_cost_.secs > kGateCostRange.max) {
      throw std::runtime_error("gate_cost_ is not within it's range");
    }
  }

  // gate_penalty_ (Cost.cost)
  distributor = make_distributor_from_range(kGatePenaltyRange);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("gate_penalty", distributor(generator));
    if (tester->gate_cost_.cost < kGatePenaltyRange.min ||
        tester->gate_cost_.cost > kGatePenaltyRange.max + kDefaultGateCost) {
      throw std::runtime_error("gate_penalty_ is not within it's range");
    }
  }

  // tollbooth_cost_ (Cost.secs)
  distributor = make_distributor_from_range(kTollBoothCostRange);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("toll_booth_cost", distributor(generator));
    if (tester->toll_booth_cost_.secs < kTollBoothCostRange.min ||
        tester->toll_booth_cost_.secs > kTollBoothCostRange.max) {
      throw std::runtime_error("tollbooth_cost_ is not within it's range");
    }
  }

  // tollbooth_penalty_ (Cost.cost)
  distributor = make_distributor_from_range(kTollBoothPenaltyRange);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("toll_booth_penalty", distributor(generator));
    if (tester->toll_booth_cost_.cost < kTollBoothPenaltyRange.min ||
        tester->toll_booth_cost_.cost > kTollBoothPenaltyRange.max + kDefaultTollBoothCost) {
      throw std::runtime_error("tollbooth_penalty_ is not within it's range");
    }
  }

  // country_crossing_cost_ (Cost.secs)
  distributor = make_distributor_from_range(kCountryCrossingCostRange);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("country_crossing_cost", distributor(generator));
    if (tester->country_crossing_cost_.secs < kCountryCrossingCostRange.min ||
        tester->country_crossing_cost_.secs > kCountryCrossingCostRange.max) {
      throw std::runtime_error("country_crossing_cost_ is not within it's range");
    }
  }

  // country_crossing_penalty_ (Cost.cost)
  distributor = make_distributor_from_range(kCountryCrossingPenaltyRange);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("country_crossing_penalty", distributor(generator));
    if (tester->country_crossing_cost_.cost < kCountryCrossingPenaltyRange.min ||
        tester->country_crossing_cost_.cost >
            kCountryCrossingPenaltyRange.max + kDefaultCountryCrossingCost) {
      throw std::runtime_error("country_crossing_penalty_ is not within it's range");
    }
  }

  // ferry_cost_ (Cost.secs)
  distributor = make_distributor_from_range(kFerryCostRange);
  for (unsigned i = 0; i < testIterations; ++i) {
    tester = make_autocost_from_json("ferry_cost", distributor(generator));
    if (tester->ferry_transition_cost_.secs < kFerryCostRange.min ||
        tester->ferry_transition_cost_.secs > kFerryCostRange.max) {
      throw std::runtime_error("ferry_cost_ is not within it's range");
    }
  }
  /**
   // use_ferry
   distributor = make_distributor_from_range(kUseFerryRange);
   for (unsigned i = 0; i < testIterations; ++i) {
     tester = make_autocost_from_json("use_ferry", distributor(generator));
     if (tester->use_ferry < kUseFerryRange.min || tester->use_ferry > kUseFerryRange.max) {
       throw std::runtime_error("use_ferry is not within it's range");
     }
   }
 **/

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
    auto actual = tester->flow_mask_;

    if (actual != expected)
      throw std::runtime_error("flow_mask_ is not the expected value in test case " +
                               std::to_string(i) + " | " + std::to_string(actual) +
                               "!=" + std::to_string(expected));
  }
}
} // namespace

int main() {
  test::suite suite("costing");

  suite.test(TEST_CASE(testAutoCostParams));

  return suite.tear_down();
}

#endif
