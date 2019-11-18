#include "sif/truckcost.h"

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
constexpr float kDefaultAlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultGateCost = 30.0f;                // Seconds
constexpr float kDefaultGatePenalty = 300.0f;            // Seconds
constexpr float kDefaultTollBoothCost = 15.0f;           // Seconds
constexpr float kDefaultTollBoothPenalty = 0.0f;         // Seconds
constexpr float kDefaultCountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultCountryCrossingPenalty = 0.0f;   // Seconds

// Other options
constexpr float kDefaultLowClassPenalty = 30.0f; // Seconds
constexpr float kDefaultUseTolls = 0.5f;         // Factor between 0 and 1

// Default turn costs
constexpr float kTCStraight = 0.5f;
constexpr float kTCSlight = 0.75f;
constexpr float kTCFavorable = 1.0f;
constexpr float kTCFavorableSharp = 1.5f;
constexpr float kTCCrossing = 2.0f;
constexpr float kTCUnfavorable = 2.5f;
constexpr float kTCUnfavorableSharp = 3.5f;
constexpr float kTCReverse = 5.0f;

// Default truck attributes
constexpr float kDefaultTruckWeight = 21.77f;  // Metric Tons (48,000 lbs)
constexpr float kDefaultTruckAxleLoad = 9.07f; // Metric Tons (20,000 lbs)
constexpr float kDefaultTruckHeight = 4.11f;   // Meters (13 feet 6 inches)
constexpr float kDefaultTruckWidth = 2.6f;     // Meters (102.36 inches)
constexpr float kDefaultTruckLength = 21.64f;  // Meters (71 feet)

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = {kTCStraight,       kTCSlight,  kTCFavorable,
                                         kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
                                         kTCUnfavorable,    kTCSlight};
constexpr float kLeftSideTurnCosts[] = {kTCStraight,         kTCSlight,  kTCUnfavorable,
                                        kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
                                        kTCFavorable,        kTCSlight};

// How much to favor truck routes.
constexpr float kTruckRouteFactor = 0.85f;

// Weighting factor based on road class. These apply penalties to lower class
// roads.
constexpr float kRoadClassFactor[] = {
    0.0f,  // Motorway
    0.05f, // Trunk
    0.1f,  // Primary
    0.25f, // Secondary
    0.35f, // Tertiary
    0.5f,  // Unclassified
    0.75f, // Residential
    0.1f   // Service, other
};

// Valid ranges and defaults
constexpr ranged_default_t<float> kManeuverPenaltyRange{0, kDefaultManeuverPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kDestinationOnlyPenaltyRange{0, kDefaultDestinationOnlyPenalty,
                                                               kMaxPenalty};
constexpr ranged_default_t<float> kAlleyPenaltyRange{0, kDefaultAlleyPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kGateCostRange{0, kDefaultGateCost, kMaxPenalty};
constexpr ranged_default_t<float> kGatePenaltyRange{0, kDefaultGatePenalty, kMaxPenalty};
constexpr ranged_default_t<float> kTollBoothCostRange{0, kDefaultTollBoothCost, kMaxPenalty};
constexpr ranged_default_t<float> kTollBoothPenaltyRange{0, kDefaultTollBoothPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kCountryCrossingCostRange{0, kDefaultCountryCrossingCost,
                                                            kMaxPenalty};
constexpr ranged_default_t<float> kCountryCrossingPenaltyRange{0, kDefaultCountryCrossingPenalty,
                                                               kMaxPenalty};
constexpr ranged_default_t<float> kLowClassPenaltyRange{0, kDefaultLowClassPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kTruckWeightRange{0, kDefaultTruckWeight, 100.0f};
constexpr ranged_default_t<float> kTruckAxleLoadRange{0, kDefaultTruckAxleLoad, 40.0f};
constexpr ranged_default_t<float> kTruckHeightRange{0, kDefaultTruckHeight, 10.0f};
constexpr ranged_default_t<float> kTruckWidthRange{0, kDefaultTruckWidth, 10.0f};
constexpr ranged_default_t<float> kTruckLengthRange{0, kDefaultTruckLength, 50.0f};
constexpr ranged_default_t<float> kUseTollsRange{0, kDefaultUseTolls, 1.0f};

} // namespace

/**
 * Derived class providing dynamic edge costing for truck routes.
 */
class TruckCost : public DynamicCost {
public:
  /**
   * Construct truck costing. Pass in cost type and options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  options pbf with request options.
   */
  TruckCost(const Costing costing, const Options& options);

  virtual ~TruckCost();

  /**
   * Does the costing allow hierarchy transitions. Truck costing will allow
   * transitions by default.
   * @return  Returns true if the costing model allows hierarchy transitions).
   */
  virtual bool AllowTransitions() const;

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
   * be restricted if bollards or gates are present.
   * @param  node  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const;

  /**
   * Callback for Allowed doing mode  specific restriction checks
   */
  virtual bool ModeSpecificAllowed(const baldr::AccessRestriction& restriction) const;

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
    throw std::runtime_error("TruckCost::EdgeCost does not support transit edges");
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  seconds   Time of week in seconds.
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
   * edges not usable / inaccessible by truck.
   */
  virtual const EdgeFilter GetEdgeFilter() const {
    // Throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge) {
      if (edge->is_shortcut() || !(edge->forwardaccess() & kTruckAccess)) {
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
    return [](const baldr::NodeInfo* node) { return !(node->access() & kTruckAccess); };
  }

public:
  VehicleType type_; // Vehicle type: tractor trailer
  float speedfactor_[kMaxSpeedKph + 1];
  float density_factor_[16]; // Density factor
  float toll_factor_;        // Factor applied when road has a toll
  float low_class_penalty_;  // Penalty (seconds) to go to residential or service road

  // Vehicle attributes (used for special restrictions and costing)
  bool hazmat_;     // Carrying hazardous materials
  float weight_;    // Vehicle weight in metric tons
  float axle_load_; // Axle load weight in metric tons
  float height_;    // Vehicle height in meters
  float width_;     // Vehicle width in meters
  float length_;    // Vehicle length in meters

  // Density factor used in edge transition costing
  std::vector<float> trans_density_factor_;
};

// Constructor
TruckCost::TruckCost(const Costing costing, const Options& options)
    : DynamicCost(options, TravelMode::kDrive), trans_density_factor_{1.0f, 1.0f, 1.0f, 1.0f,
                                                                      1.0f, 1.1f, 1.2f, 1.3f,
                                                                      1.4f, 1.6f, 1.9f, 2.2f,
                                                                      2.5f, 2.8f, 3.1f, 3.5f} {

  // Grab the costing options based on the specified costing type
  const CostingOptions& costing_options = options.costing_options(static_cast<int>(costing));

  type_ = VehicleType::kTractorTrailer;

  // Get the base costs
  get_base_costs(costing_options);

  low_class_penalty_ = costing_options.low_class_penalty();

  // Get the vehicle attributes
  hazmat_ = costing_options.hazmat();
  weight_ = costing_options.weight();
  axle_load_ = costing_options.axle_load();
  height_ = costing_options.height();
  width_ = costing_options.width();
  length_ = costing_options.length();

  // Create speed cost table
  speedfactor_[0] = kSecPerHour; // TODO - what to make speed=0?
  for (uint32_t s = 1; s <= kMaxSpeedKph; s++) {
    speedfactor_[s] = (kSecPerHour * 0.001f) / static_cast<float>(s);
  }

  // Preference to use toll roads (separate from toll booth penalty). Sets a toll
  // factor. A toll factor of 0 would indicate no adjustment to weighting for toll roads.
  // use_tolls = 1 would reduce weighting slightly (a negative delta) while
  // use_tolls = 0 would penalize (positive delta to weighting factor).
  float use_tolls = costing_options.use_tolls();
  toll_factor_ = use_tolls < 0.5f ? (2.0f - 4 * use_tolls) : // ranges from 2 to 0
                     (0.5f - use_tolls) * 0.03f;             // ranges from 0 to -0.15

  for (uint32_t d = 0; d < 16; d++) {
    density_factor_[d] = 0.85f + (d * 0.025f);
  }
}

// Destructor
TruckCost::~TruckCost() {
}

// Auto costing will allow hierarchy transitions by default.
bool TruckCost::AllowTransitions() const {
  return true;
}

// Does the costing method allow multiple passes (with relaxed hierarchy
// limits).
bool TruckCost::AllowMultiPass() const {
  return true;
}

// Get the access mode used by this costing method.
uint32_t TruckCost::access_mode() const {
  return kTruckAccess;
}

bool TruckCost::ModeSpecificAllowed(const baldr::AccessRestriction& restriction) const {
  switch (restriction.type()) {
    case AccessType::kHazmat:
      if (hazmat_ != restriction.value()) {
        return false;
      }
      break;
    case AccessType::kMaxAxleLoad:
      if (axle_load_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxHeight:
      if (height_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxLength:
      if (length_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxWeight:
      if (weight_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxWidth:
      if (width_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    default:
      return true;
  };
  return true;
}

// Check if access is allowed on the specified edge.
inline bool TruckCost::Allowed(const baldr::DirectedEdge* edge,
                               const EdgeLabel& pred,
                               const baldr::GraphTile*& tile,
                               const baldr::GraphId& edgeid,
                               const uint64_t current_time,
                               const uint32_t tz_index,
                               bool& has_time_restrictions) const {
  // Check access, U-turn, and simple turn restriction.
  // TODO - perhaps allow U-turns at dead-end nodes?
  if (!(edge->forwardaccess() & kTruckAccess) || (pred.opp_local_idx() == edge->localedgeidx()) ||
      (pred.restrictions() & (1 << edge->localedgeidx())) ||
      edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly())) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(kTruckAccess, edge, tile, edgeid, current_time, tz_index,
                                           has_time_restrictions);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool TruckCost::AllowedReverse(const baldr::DirectedEdge* edge,
                               const EdgeLabel& pred,
                               const baldr::DirectedEdge* opp_edge,
                               const baldr::GraphTile*& tile,
                               const baldr::GraphId& opp_edgeid,
                               const uint64_t current_time,
                               const uint32_t tz_index,
                               bool& has_time_restrictions) const {
  // Check access, U-turn, and simple turn restriction.
  // TODO - perhaps allow U-turns at dead-end nodes?
  if (!(opp_edge->forwardaccess() & kTruckAccess) || (pred.opp_local_idx() == edge->localedgeidx()) ||
      (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
      opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly())) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(kTruckAccess, edge, tile, opp_edgeid, current_time,
                                           tz_index, has_time_restrictions);
}

// Check if access is allowed at the specified node.
bool TruckCost::Allowed(const baldr::NodeInfo* node) const {
  return (node->access() & kTruckAccess);
}

// Get the cost to traverse the edge in seconds
Cost TruckCost::EdgeCost(const baldr::DirectedEdge* edge,
                         const baldr::GraphTile* tile,
                         const uint32_t seconds) const {
  auto speed = tile->GetSpeed(edge, flow_mask_, seconds);
  float factor = density_factor_[edge->density()];
  if (edge->truck_route() > 0) {
    factor *= kTruckRouteFactor;
  }

  if (edge->toll()) {
    factor += toll_factor_;
  }

  // Use the lower or truck speed (ir present) and speed
  uint32_t s = (edge->truck_speed() > 0) ? std::min(edge->truck_speed(), speed) : speed;
  float sec = edge->length() * speedfactor_[s];
  return {sec * factor, sec};
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost TruckCost::TransitionCost(const baldr::DirectedEdge* edge,
                               const baldr::NodeInfo* node,
                               const EdgeLabel& pred) const {
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = base_transition_cost(node, edge, pred, idx);

  // Penalty to transition onto low class roads.
  if (edge->classification() == RoadClass::kResidential ||
      edge->classification() == RoadClass::kServiceOther) {
    c.cost += low_class_penalty_;
  }

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
Cost TruckCost::TransitionCostReverse(const uint32_t idx,
                                      const baldr::NodeInfo* node,
                                      const baldr::DirectedEdge* pred,
                                      const baldr::DirectedEdge* edge) const {
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  Cost c = base_transition_cost(node, edge, pred, idx);

  // Penalty to transition onto low class roads.
  if (edge->classification() == RoadClass::kResidential ||
      edge->classification() == RoadClass::kServiceOther) {
    c.cost += low_class_penalty_;
  }

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

    c.cost += seconds;
    c.secs += seconds;
  }
  return c;
}

// Get the cost factor for A* heuristics. This factor is multiplied
// with the distance to the destination to produce an estimate of the
// minimum cost to the destination. The A* heuristic must underestimate the
// cost to the destination. So a time based estimate based on speed should
// assume the maximum speed is used to the destination such that the time
// estimate is less than the least possible time along roads.
float TruckCost::AStarCostFactor() const {
  return speedfactor_[kMaxSpeedKph];
}

// Returns the current travel type.
uint8_t TruckCost::travel_type() const {
  return static_cast<uint8_t>(type_);
}

void ParseTruckCostOptions(const rapidjson::Document& doc,
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

    // low_class_penalty
    pbf_costing_options->set_low_class_penalty(kLowClassPenaltyRange(
        rapidjson::get_optional<float>(*json_costing_options, "/low_class_penalty")
            .get_value_or(kDefaultLowClassPenalty)));

    // hazmat
    pbf_costing_options->set_hazmat(
        rapidjson::get_optional<bool>(*json_costing_options, "/hazmat").get_value_or(false));

    // weight
    pbf_costing_options->set_weight(
        kTruckWeightRange(rapidjson::get_optional<float>(*json_costing_options, "/weight")
                              .get_value_or(kDefaultTruckWeight)));

    // axle_load
    pbf_costing_options->set_axle_load(
        kTruckAxleLoadRange(rapidjson::get_optional<float>(*json_costing_options, "/axle_load")
                                .get_value_or(kDefaultTruckAxleLoad)));

    // height
    pbf_costing_options->set_height(
        kTruckHeightRange(rapidjson::get_optional<float>(*json_costing_options, "/height")
                              .get_value_or(kDefaultTruckHeight)));

    // width
    pbf_costing_options->set_width(
        kTruckWidthRange(rapidjson::get_optional<float>(*json_costing_options, "/width")
                             .get_value_or(kDefaultTruckWidth)));

    // length
    pbf_costing_options->set_length(
        kTruckLengthRange(rapidjson::get_optional<float>(*json_costing_options, "/length")
                              .get_value_or(kDefaultTruckLength)));

    // use_tolls
    pbf_costing_options->set_use_tolls(
        kUseTollsRange(rapidjson::get_optional<float>(*json_costing_options, "/use_tolls")
                           .get_value_or(kDefaultUseTolls)));
  } else {
    // Set pbf values to defaults
    pbf_costing_options->set_maneuver_penalty(kDefaultManeuverPenalty);
    pbf_costing_options->set_destination_only_penalty(kDefaultDestinationOnlyPenalty);
    pbf_costing_options->set_gate_cost(kDefaultGateCost);
    pbf_costing_options->set_gate_penalty(kDefaultGatePenalty);
    pbf_costing_options->set_toll_booth_cost(kDefaultTollBoothCost);
    pbf_costing_options->set_toll_booth_penalty(kDefaultTollBoothPenalty);
    pbf_costing_options->set_alley_penalty(kDefaultAlleyPenalty);
    pbf_costing_options->set_country_crossing_cost(kDefaultCountryCrossingCost);
    pbf_costing_options->set_country_crossing_penalty(kDefaultCountryCrossingPenalty);
    pbf_costing_options->set_low_class_penalty(kDefaultLowClassPenalty);
    pbf_costing_options->set_hazmat(false);
    pbf_costing_options->set_weight(kDefaultTruckWeight);
    pbf_costing_options->set_axle_load(kDefaultTruckAxleLoad);
    pbf_costing_options->set_height(kDefaultTruckHeight);
    pbf_costing_options->set_width(kDefaultTruckWidth);
    pbf_costing_options->set_length(kDefaultTruckLength);
    pbf_costing_options->set_use_tolls(kDefaultUseTolls);
    pbf_costing_options->set_flow_mask(kDefaultFlowMask);
  }
}

cost_ptr_t CreateTruckCost(const Costing costing, const Options& options) {
  return std::make_shared<TruckCost>(costing, options);
}

} // namespace sif
} // namespace valhalla

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

class TestTruckCost : public TruckCost {
public:
  TestTruckCost(const Costing costing, const Options& options) : TruckCost(costing, options){};

  using TruckCost::alley_penalty_;
  using TruckCost::country_crossing_cost_;
  using TruckCost::destination_only_penalty_;
  using TruckCost::ferry_transition_cost_;
  using TruckCost::gate_cost_;
  using TruckCost::maneuver_penalty_;
  using TruckCost::toll_booth_cost_;
};

TestTruckCost* make_truckcost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({"costing_options":{"truck":{")" << property << R"(":)" << testVal << "}}}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return new TestTruckCost(valhalla::Costing::truck, request.options());
}

std::uniform_real_distribution<float>*
make_distributor_from_range(const ranged_default_t<float>& range) {
  float rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<float>(range.min - rangeLength, range.max + rangeLength);
}

void testTruckCostParams() {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> distributor;
  std::shared_ptr<TestTruckCost> ctorTester;

  // maneuver_penalty_
  distributor.reset(make_distributor_from_range(kManeuverPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("maneuver_penalty", (*distributor)(generator)));
    if (ctorTester->maneuver_penalty_ < kManeuverPenaltyRange.min ||
        ctorTester->maneuver_penalty_ > kManeuverPenaltyRange.max) {
      throw std::runtime_error("maneuver_penalty_ is not within it's range");
    }
  }

  // alley_penalty_
  distributor.reset(make_distributor_from_range(kAlleyPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("alley_penalty", (*distributor)(generator)));
    if (ctorTester->alley_penalty_ < kAlleyPenaltyRange.min ||
        ctorTester->alley_penalty_ > kAlleyPenaltyRange.max) {
      throw std::runtime_error("alley_penalty_ is not within it's range");
    }
  }

  // destination_only_penalty_
  distributor.reset(make_distributor_from_range(kDestinationOnlyPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("destination_only_penalty", (*distributor)(generator)));
    if (ctorTester->destination_only_penalty_ < kDestinationOnlyPenaltyRange.min ||
        ctorTester->destination_only_penalty_ > kDestinationOnlyPenaltyRange.max) {
      throw std::runtime_error("destination_only_penalty_ is not within it's range");
    }
  }

  // gate_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(kGateCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("gate_cost", (*distributor)(generator)));
    if (ctorTester->gate_cost_.secs < kGateCostRange.min ||
        ctorTester->gate_cost_.secs > kGateCostRange.max) {
      throw std::runtime_error("gate_cost_ is not within it's range");
    }
  }

  // gate_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(kGatePenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("gate_penalty", (*distributor)(generator)));
    if (ctorTester->gate_cost_.cost < kGatePenaltyRange.min ||
        ctorTester->gate_cost_.cost > kGatePenaltyRange.max + kDefaultGateCost) {
      throw std::runtime_error("gate_penalty_ is not within it's range");
    }
  }

  // tollbooth_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(kTollBoothCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("toll_booth_cost", (*distributor)(generator)));
    if (ctorTester->toll_booth_cost_.secs < kTollBoothCostRange.min ||
        ctorTester->toll_booth_cost_.secs > kTollBoothCostRange.max) {
      throw std::runtime_error("tollbooth_cost_ is not within it's range");
    }
  }

  // tollbooth_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(kTollBoothPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("toll_booth_penalty", (*distributor)(generator)));
    if (ctorTester->toll_booth_cost_.cost < kTollBoothPenaltyRange.min ||
        ctorTester->toll_booth_cost_.cost > kTollBoothPenaltyRange.max + kDefaultTollBoothCost) {
      throw std::runtime_error("tollbooth_penalty_ is not within it's range");
    }
  }

  // country_crossing_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(kCountryCrossingCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("country_crossing_cost", (*distributor)(generator)));
    if (ctorTester->country_crossing_cost_.secs < kCountryCrossingCostRange.min ||
        ctorTester->country_crossing_cost_.secs > kCountryCrossingCostRange.max) {
      throw std::runtime_error("country_crossing_cost_ is not within it's range");
    }
  }

  // country_crossing_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(kCountryCrossingPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("country_crossing_penalty", (*distributor)(generator)));
    if (ctorTester->country_crossing_cost_.cost < kCountryCrossingPenaltyRange.min ||
        ctorTester->country_crossing_cost_.cost >
            kCountryCrossingPenaltyRange.max + kDefaultCountryCrossingCost) {
      throw std::runtime_error("country_crossing_penalty_ is not within it's range");
    }
  }

  // Ferry transition cost and ferry use not yet supported

  // low_class_penalty_
  distributor.reset(make_distributor_from_range(kLowClassPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("low_class_penalty", (*distributor)(generator)));
    if (ctorTester->low_class_penalty_ < kLowClassPenaltyRange.min ||
        ctorTester->low_class_penalty_ > kLowClassPenaltyRange.max) {
      throw std::runtime_error("low_class_penalty_ is not within it's range");
    }
  }

  // weight_
  distributor.reset(make_distributor_from_range(kTruckWeightRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("weight", (*distributor)(generator)));
    if (ctorTester->weight_ < kTruckWeightRange.min || ctorTester->weight_ > kTruckWeightRange.max) {
      throw std::runtime_error("weight_ is not within it's range");
    }
  }

  // axle_load_
  distributor.reset(make_distributor_from_range(kTruckAxleLoadRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("axle_load", (*distributor)(generator)));
    if (ctorTester->axle_load_ < kTruckAxleLoadRange.min ||
        ctorTester->axle_load_ > kTruckAxleLoadRange.max) {
      throw std::runtime_error("axle_load_ is not within it's range");
    }
  }

  // height_
  distributor.reset(make_distributor_from_range(kTruckHeightRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("height", (*distributor)(generator)));
    if (ctorTester->height_ < kTruckHeightRange.min || ctorTester->height_ > kTruckHeightRange.max) {
      throw std::runtime_error("height_ is not within it's range");
    }
  }

  // width_
  distributor.reset(make_distributor_from_range(kTruckWidthRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("width", (*distributor)(generator)));
    if (ctorTester->width_ < kTruckWidthRange.min || ctorTester->width_ > kTruckWidthRange.max) {
      throw std::runtime_error("width_ is not within it's range");
    }
  }

  // length_
  distributor.reset(make_distributor_from_range(kTruckLengthRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("length", (*distributor)(generator)));
    if (ctorTester->length_ < kTruckLengthRange.min || ctorTester->length_ > kTruckLengthRange.max) {
      throw std::runtime_error("length_ is not within it's range");
    }
  }
}
} // namespace

int main() {
  test::suite suite("costing");

  suite.test(TEST_CASE(testTruckCostParams));

  return suite.tear_down();
}

#endif
