#include "sif/dynamiccost.h"

#include "baldr/graphconstants.h"
#include "midgard/util.h"
#include "proto_conversions.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/motorcyclecost.h"
#include "sif/motorscootercost.h"
#include "sif/nocost.h"
#include "sif/pedestriancost.h"
#include "sif/transitcost.h"
#include "sif/truckcost.h"
#include "worker.h"
#include <utility>

using namespace valhalla::baldr;

namespace {

uint8_t SpeedMask_Parse(const boost::optional<const rapidjson::Value&>& speed_types) {
  static const std::unordered_map<std::string, uint8_t> types{
      {"freeflow", kFreeFlowMask},
      {"constrained", kConstrainedFlowMask},
      {"predicted", kPredictedFlowMask},
      {"current", kCurrentFlowMask},
  };

  if (!speed_types)
    return kDefaultFlowMask;

  bool had_value = false;
  uint8_t mask = 0;
  if (speed_types->IsArray()) {
    had_value = true;
    for (const auto& speed_type : speed_types->GetArray()) {
      if (speed_type.IsString()) {
        auto i = types.find(speed_type.GetString());
        if (i != types.cend()) {
          mask |= i->second;
        }
      }
    }
  }

  return had_value ? mask : kDefaultFlowMask;
}

} // namespace

namespace valhalla {
namespace sif {

// default options/parameters
namespace {

// max penalty to apply when use tracks
constexpr float kMaxTrackPenalty = 300.f; // 5 min

// min and max factors to apply when use tracks
constexpr float kMinTrackFactor = 0.8f;
constexpr float kMaxTrackFactor = 4.f;

// max penalty to apply when use living streets
constexpr float kMaxLivingStreetPenalty = 500.f;

// min and max factors to apply when use living streets
constexpr float kMinLivingStreetFactor = 0.8f;
constexpr float kMaxLivingStreetFactor = 3.f;

constexpr float kMinFactor = 0.1f;
constexpr float kMaxFactor = 100000.0f;

// Base transition costs
constexpr float kDefaultDestinationOnlyPenalty = 600.0f; // Seconds
constexpr float kDefaultManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultAlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultGateCost = 30.0f;                // Seconds
constexpr float kDefaultGatePenalty = 300.0f;            // Seconds
constexpr float kDefaultPrivateAccessPenalty = 450.0f;   // Seconds
constexpr float kDefaultTollBoothCost = 15.0f;           // Seconds
constexpr float kDefaultTollBoothPenalty = 0.0f;         // Seconds
constexpr float kDefaultFerryCost = 300.0f;              // Seconds
constexpr float kDefaultRailFerryCost = 300.0f;          // Seconds
constexpr float kDefaultCountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultCountryCrossingPenalty = 0.0f;   // Seconds
constexpr float kDefaultServicePenalty = 15.0f;          // Seconds

// Other options
constexpr float kDefaultUseFerry = 0.5f;         // Default preference of using a ferry 0-1
constexpr float kDefaultUseRailFerry = 0.4f;     // Default preference of using a rail ferry 0-1
constexpr float kDefaultUseTracks = 0.5f;        // Default preference of using tracks 0-1
constexpr float kDefaultUseLivingStreets = 0.1f; // Default preference of using living streets 0-1

// How much to avoid generic service roads.
constexpr float kDefaultServiceFactor = 1.0f;

// Default penalty factor for avoiding closures (increases the cost of an edge as if its being
// traversed at kMinSpeedKph)
constexpr float kDefaultClosureFactor = 9.0f;
// Default range of closure factor to use for closed edges. Min is set to 1.0, which means do not
// penalize closed edges. The max is set to 10.0 in order to limit how much expansion occurs from the
// non-closure end
constexpr ranged_default_t<float> kClosureFactorRange{1.0f, kDefaultClosureFactor, 10.0f};

constexpr ranged_default_t<uint32_t> kVehicleSpeedRange{10, baldr::kMaxAssumedSpeed,
                                                        baldr::kMaxSpeedKph};
} // namespace

/*
 * Assign default values for costing options in constructor. In case of different
 * default values they should be overrided in "<type>cost.cc" file.
 */
BaseCostingOptionsConfig::BaseCostingOptionsConfig()
    : dest_only_penalty_{0.f, kDefaultDestinationOnlyPenalty, kMaxPenalty},
      maneuver_penalty_{0.f, kDefaultManeuverPenalty, kMaxPenalty},
      alley_penalty_{0.f, kDefaultAlleyPenalty, kMaxPenalty},
      gate_cost_{0.f, kDefaultGateCost, kMaxPenalty}, gate_penalty_{0.f, kDefaultGatePenalty,
                                                                    kMaxPenalty},
      private_access_penalty_{0.f, kDefaultPrivateAccessPenalty, kMaxPenalty},
      country_crossing_cost_{0.f, kDefaultCountryCrossingCost, kMaxPenalty},
      country_crossing_penalty_{0.f, kDefaultCountryCrossingPenalty, kMaxPenalty},
      toll_booth_cost_{0.f, kDefaultTollBoothCost, kMaxPenalty},
      toll_booth_penalty_{0.f, kDefaultTollBoothPenalty, kMaxPenalty},
      ferry_cost_{0.f, kDefaultFerryCost, kMaxPenalty}, use_ferry_{0.f, kDefaultUseFerry, 1.f},
      rail_ferry_cost_{0.f, kDefaultRailFerryCost, kMaxPenalty},
      use_rail_ferry_{0.f, kDefaultUseRailFerry, 1.f}, service_penalty_{0.f, kDefaultServicePenalty,
                                                                        kMaxPenalty},
      service_factor_{kMinFactor, kDefaultServiceFactor, kMaxFactor}, use_tracks_{0.f,
                                                                                  kDefaultUseTracks,
                                                                                  1.f},
      use_living_streets_{0.f, kDefaultUseLivingStreets, 1.f}, closure_factor_{kClosureFactorRange},
      exclude_unpaved_(false),
      exclude_cash_only_tolls_(false), include_hot_{false}, include_hov2_{false}, include_hov3_{
                                                                                      false} {
}

DynamicCost::DynamicCost(const CostingOptions& options,
                         const TravelMode mode,
                         uint32_t access_mask,
                         bool penalize_uturns)
    : pass_(0), allow_transit_connections_(false), allow_destination_only_(true),
      allow_conditional_destination_(false), travel_mode_(mode), access_mask_(access_mask),
      closure_factor_(kDefaultClosureFactor), flow_mask_(kDefaultFlowMask),
      shortest_(options.shortest()), ignore_restrictions_(options.ignore_restrictions()),
      ignore_oneways_(options.ignore_oneways()), ignore_access_(options.ignore_access()),
      ignore_closures_(options.ignore_closures()), top_speed_(options.top_speed()),
      filter_closures_(ignore_closures_ ? false : options.filter_closures()),
      penalize_uturns_(penalize_uturns) {
  // Parse property tree to get hierarchy limits
  // TODO - get the number of levels
  uint32_t n_levels = sizeof(kDefaultMaxUpTransitions) / sizeof(kDefaultMaxUpTransitions[0]);
  for (uint32_t level = 0; level < n_levels; level++) {
    hierarchy_limits_.emplace_back(HierarchyLimits(level));
  }

  // Add avoid edges to internal set
  for (auto& edge : options.exclude_edges()) {
    user_exclude_edges_.insert({GraphId(edge.id()), edge.percent_along()});
  }
}

DynamicCost::~DynamicCost() {
}

// Does the costing method allow multiple passes (with relaxed hierarchy
// limits). Defaults to false. Costing methods that wish to allow multiple
// passes with relaxed hierarchy transitions must override this method.
bool DynamicCost::AllowMultiPass() const {
  return false;
}

// We provide a convenience method for those algorithms which dont have time components or aren't
// using them for the current route. Here we just call out to the derived classes costing function
// with a time that tells the function that we aren't using time. This avoids having to worry about
// default parameters and inheritance (which are a bad mix)
Cost DynamicCost::EdgeCost(const baldr::DirectedEdge* edge, const graph_tile_ptr& tile) const {
  uint8_t flow_sources;
  return EdgeCost(edge, tile, kConstrainedFlowSecondOfDay, flow_sources);
}

// Returns the cost to make the transition from the predecessor edge.
// Defaults to 0. Costing models that wish to include edge transition
// costs (i.e., intersection/turn costs) must override this method.
Cost DynamicCost::TransitionCost(const DirectedEdge*, const NodeInfo*, const EdgeLabel&) const {
  return {0.0f, 0.0f};
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// Defaults to 0. Costing models that wish to include edge transition
// costs (i.e., intersection/turn costs) must override this method.
Cost DynamicCost::TransitionCostReverse(const uint32_t,
                                        const baldr::NodeInfo*,
                                        const baldr::DirectedEdge*,
                                        const baldr::DirectedEdge*,
                                        const bool,
                                        const InternalTurn) const {
  return {0.0f, 0.0f};
}

// Returns the transfer cost between 2 transit stops.
Cost DynamicCost::TransferCost() const {
  return {0.0f, 0.0f};
}

// Returns the default transfer cost between 2 transit stops.
Cost DynamicCost::DefaultTransferCost() const {
  return {0.0f, 0.0f};
}

// Get the general unit size that can be considered as equal for sorting
// purposes. Defaults to 1 (second).
uint32_t DynamicCost::UnitSize() const {
  return kDefaultUnitSize;
}

// Set to allow use of transit connections.
void DynamicCost::SetAllowTransitConnections(const bool allow) {
  allow_transit_connections_ = allow;
}

// Sets the flag indicating whether destination only edges are allowed.
void DynamicCost::set_allow_destination_only(const bool allow) {
  allow_destination_only_ = allow;
}

// Sets the flag indicating whether edges with valid restriction conditional=destination are allowed.
void DynamicCost::set_allow_conditional_destination(const bool allow) {
  allow_conditional_destination_ = allow;
}

// Returns the maximum transfer distance between stops that you are willing
// to travel for this mode.  It is the max distance you are willing to
// travel between transfers.
uint32_t DynamicCost::GetMaxTransferDistanceMM() {
  return 0;
}

// This method overrides the factor for this mode.  The lower the value
// the more the mode is favored.
float DynamicCost::GetModeFactor() {
  return 1.0f;
}

// This method overrides the max_distance with the max_distance_mm per segment
// distance. An example is a pure walking route may have a max distance of
// 10000 meters (10km) but for a multi-modal route a lower limit of 5000
// meters per segment (e.g. from origin to a transit stop or from the last
// transit stop to the destination).
void DynamicCost::UseMaxMultiModalDistance() {
}

// Gets the hierarchy limits.
std::vector<HierarchyLimits>& DynamicCost::GetHierarchyLimits() {
  return hierarchy_limits_;
}

// Relax hierarchy limits.
void DynamicCost::RelaxHierarchyLimits(const bool using_bidirectional) {
  // since bidirectional A* does about half the expansion we can do half the relaxation here
  const float relax_factor = using_bidirectional ? 8.f : 16.f;
  const float expansion_within_factor = using_bidirectional ? 2.0f : 4.0f;

  for (auto& hierarchy : hierarchy_limits_) {
    hierarchy.Relax(relax_factor, expansion_within_factor);
  }
}

// Set the current travel mode.
void DynamicCost::set_travel_mode(const TravelMode mode) {
  travel_mode_ = mode;
}

// Get the current travel mode.
TravelMode DynamicCost::travel_mode() const {
  return travel_mode_;
}

// Get the current travel type.
uint8_t DynamicCost::travel_type() const {
  return 0;
}

// Get the wheelchair required flag.
bool DynamicCost::wheelchair() const {
  return false;
}

// Get the bicycle required flag.
bool DynamicCost::bicycle() const {
  return false;
}

// Add to the exclude list.
void DynamicCost::AddToExcludeList(const graph_tile_ptr&) {
}

// Checks if we should exclude or not.
bool DynamicCost::IsExcluded(const graph_tile_ptr&, const baldr::DirectedEdge*) {
  return false;
}

// Checks if we should exclude or not.
bool DynamicCost::IsExcluded(const graph_tile_ptr&, const baldr::NodeInfo*) {
  return false;
}

// Adds a list of edges (GraphIds) to the user specified avoid list.
void DynamicCost::AddUserAvoidEdges(const std::vector<AvoidEdge>& exclude_edges) {
  for (auto edge : exclude_edges) {
    user_exclude_edges_.insert({edge.id, edge.percent_along});
  }
}

Cost DynamicCost::BSSCost() const {
  return kNoCost;
}

void DynamicCost::set_use_tracks(float use_tracks) {
  // Calculate penalty value based on use preference. Return value
  // in range [kMaxTrackPenalty; 0], if use < 0.5; or
  // 0, if use > 0.5.
  track_penalty_ = use_tracks < 0.5f ? (kMaxTrackPenalty * (1.f - 2.f * use_tracks)) : 0.f;
  // Calculate factor value based on use preference. Return value
  // in range [kMaxTrackFactor; 1], if use < 0.5; or
  // in range [1; kMinTrackFactor], if use > 0.5
  track_factor_ = use_tracks < 0.5f
                      ? (kMaxTrackFactor - 2.f * use_tracks * (kMaxTrackFactor - 1.f))
                      : (kMinTrackFactor + 2.f * (1.f - use_tracks) * (1.f - kMinTrackFactor));
}

void DynamicCost::set_use_living_streets(float use_living_streets) {
  // Calculate penalty value based on use preference. Return value
  // in range [kMaxLivingStreetPenalty; 0], if use < 0.5; or
  // 0, if use > 0.5.
  living_street_penalty_ =
      use_living_streets < 0.5f ? (kMaxLivingStreetPenalty * (1.f - 2.f * use_living_streets)) : 0;

  // Calculate factor value based on use preference. Return value
  // in range [kMaxLivingStreetFactor; 1], if use < 0.5; or
  // in range [1; kMinLivingStreetFactor], if use > 0.5.
  // Thus living_street_factor_ is inversely proportional to use_living_streets.
  living_street_factor_ =
      use_living_streets < 0.5f
          ? (kMaxLivingStreetFactor - 2.f * use_living_streets * (kMaxLivingStreetFactor - 1.f))
          : (kMinLivingStreetFactor +
             2.f * (1.f - use_living_streets) * (1.f - kMinLivingStreetFactor));
}

void ParseSharedCostOptions(const rapidjson::Value& value, CostingOptions* pbf_costing_options) {
  auto speed_types = rapidjson::get_child_optional(value, "/speed_types");
  pbf_costing_options->set_flow_mask(SpeedMask_Parse(speed_types));

  pbf_costing_options->set_ignore_restrictions(
      rapidjson::get<bool>(value, "/ignore_restrictions", false));
  pbf_costing_options->set_ignore_oneways(rapidjson::get<bool>(value, "/ignore_oneways", false));
  pbf_costing_options->set_ignore_access(rapidjson::get<bool>(value, "/ignore_access", false));
  pbf_costing_options->set_ignore_closures(rapidjson::get<bool>(value, "/ignore_closures", false));
  auto name = rapidjson::get_optional<std::string>(value, "/name");
  if (name) {
    pbf_costing_options->set_name(*name);
  }
  pbf_costing_options->set_shortest(rapidjson::get<bool>(value, "/shortest", false));
  pbf_costing_options->set_top_speed(
      kVehicleSpeedRange(rapidjson::get<uint32_t>(value, "/top_speed", kMaxAssumedSpeed)));
}

void ParseBaseCostOptions(const rapidjson::Value& value,
                          CostingOptions* pbf_costing_options,
                          const BaseCostingOptionsConfig& base_cfg) {
  // destination only penalty
  pbf_costing_options->set_destination_only_penalty(base_cfg.dest_only_penalty_(
      rapidjson::get<float>(value, "/destination_only_penalty", base_cfg.dest_only_penalty_.def)));

  // maneuver_penalty
  pbf_costing_options->set_maneuver_penalty(base_cfg.maneuver_penalty_(
      rapidjson::get<float>(value, "/maneuver_penalty", base_cfg.maneuver_penalty_.def)));

  // alley_penalty
  pbf_costing_options->set_alley_penalty(base_cfg.alley_penalty_(
      rapidjson::get<float>(value, "/alley_penalty", base_cfg.alley_penalty_.def)));

  // gate_cost
  pbf_costing_options->set_gate_cost(
      base_cfg.gate_cost_(rapidjson::get<float>(value, "/gate_cost", base_cfg.gate_cost_.def)));

  // gate_penalty
  pbf_costing_options->set_gate_penalty(base_cfg.gate_penalty_(
      rapidjson::get<float>(value, "/gate_penalty", base_cfg.gate_penalty_.def)));

  // private_access_penalty
  pbf_costing_options->set_private_access_penalty(base_cfg.private_access_penalty_(
      rapidjson::get<float>(value, "/private_access_penalty", base_cfg.private_access_penalty_.def)));

  // country_crossing_cost
  pbf_costing_options->set_country_crossing_cost(base_cfg.country_crossing_cost_(
      rapidjson::get<float>(value, "/country_crossing_cost", base_cfg.country_crossing_cost_.def)));

  // country_crossing_penalty
  pbf_costing_options->set_country_crossing_penalty(base_cfg.country_crossing_penalty_(
      rapidjson::get<float>(value, "/country_crossing_penalty",
                            base_cfg.country_crossing_penalty_.def)));

  if (!base_cfg.disable_toll_booth_) {
    // toll_booth_cost
    pbf_costing_options->set_toll_booth_cost(base_cfg.toll_booth_cost_(
        rapidjson::get<float>(value, "/toll_booth_cost", base_cfg.toll_booth_cost_.def)));

    // toll_booth_penalty
    pbf_costing_options->set_toll_booth_penalty(base_cfg.toll_booth_penalty_(
        rapidjson::get<float>(value, "/toll_booth_penalty", base_cfg.toll_booth_penalty_.def)));
  }

  if (!base_cfg.disable_ferry_) {
    // ferry_cost
    pbf_costing_options->set_ferry_cost(
        base_cfg.ferry_cost_(rapidjson::get<float>(value, "/ferry_cost", base_cfg.ferry_cost_.def)));

    // use_ferry
    pbf_costing_options->set_use_ferry(
        base_cfg.use_ferry_(rapidjson::get<float>(value, "/use_ferry", base_cfg.use_ferry_.def)));
  }

  if (!base_cfg.disable_rail_ferry_) {
    // rail_ferry_cost
    pbf_costing_options->set_rail_ferry_cost(base_cfg.rail_ferry_cost_(
        rapidjson::get<float>(value, "/rail_ferry_cost", base_cfg.rail_ferry_cost_.def)));

    // use_rail_ferry
    pbf_costing_options->set_use_rail_ferry(base_cfg.use_rail_ferry_(
        rapidjson::get<float>(value, "/use_rail_ferry", base_cfg.use_rail_ferry_.def)));
  }

  pbf_costing_options->set_exclude_unpaved(
      rapidjson::get<bool>(value, "/exclude_unpaved", base_cfg.exclude_unpaved_));

  pbf_costing_options->set_exclude_cash_only_tolls(
      rapidjson::get<bool>(value, "/exclude_cash_only_tolls", base_cfg.exclude_cash_only_tolls_));

  // service_penalty
  pbf_costing_options->set_service_penalty(base_cfg.service_penalty_(
      rapidjson::get<float>(value, "/service_penalty", base_cfg.service_penalty_.def)));

  // service_factor
  pbf_costing_options->set_service_factor(base_cfg.service_factor_(
      rapidjson::get<float>(value, "/service_factor", base_cfg.service_factor_.def)));

  // use_tracks
  pbf_costing_options->set_use_tracks(
      base_cfg.use_tracks_(rapidjson::get<float>(value, "/use_tracks", base_cfg.use_tracks_.def)));

  // use_living_streets
  pbf_costing_options->set_use_living_streets(base_cfg.use_living_streets_(
      rapidjson::get<float>(value, "/use_living_streets", base_cfg.use_living_streets_.def)));

  // closure_factor
  pbf_costing_options->set_closure_factor(base_cfg.closure_factor_(
      rapidjson::get<float>(value, "/closure_factor", base_cfg.closure_factor_.def)));

  // HOT/HOV
  pbf_costing_options->set_include_hot(
      rapidjson::get<bool>(value, "/include_hot", base_cfg.include_hot_));
  pbf_costing_options->set_include_hov2(
      rapidjson::get<bool>(value, "/include_hov2", base_cfg.include_hov2_));
  pbf_costing_options->set_include_hov3(
      rapidjson::get<bool>(value, "/include_hov3", base_cfg.include_hov3_));
}

void SetDefaultBaseCostOptions(CostingOptions* pbf_costing_options,
                               const BaseCostingOptionsConfig& shared_opts) {
  pbf_costing_options->set_destination_only_penalty(shared_opts.dest_only_penalty_.def);
  pbf_costing_options->set_maneuver_penalty(shared_opts.maneuver_penalty_.def);
  pbf_costing_options->set_alley_penalty(shared_opts.alley_penalty_.def);
  pbf_costing_options->set_gate_cost(shared_opts.gate_cost_.def);
  pbf_costing_options->set_gate_penalty(shared_opts.gate_penalty_.def);
  pbf_costing_options->set_private_access_penalty(shared_opts.private_access_penalty_.def);
  pbf_costing_options->set_country_crossing_cost(shared_opts.country_crossing_cost_.def);
  pbf_costing_options->set_country_crossing_penalty(shared_opts.country_crossing_penalty_.def);

  if (!shared_opts.disable_toll_booth_) {
    pbf_costing_options->set_toll_booth_cost(shared_opts.toll_booth_cost_.def);
    pbf_costing_options->set_toll_booth_penalty(shared_opts.toll_booth_penalty_.def);
  }

  if (!shared_opts.disable_ferry_) {
    pbf_costing_options->set_ferry_cost(shared_opts.ferry_cost_.def);
    pbf_costing_options->set_use_ferry(shared_opts.use_ferry_.def);
  }

  if (!shared_opts.disable_rail_ferry_) {
    pbf_costing_options->set_rail_ferry_cost(shared_opts.rail_ferry_cost_.def);
    pbf_costing_options->set_use_rail_ferry(shared_opts.use_rail_ferry_.def);
  }

  pbf_costing_options->set_service_penalty(shared_opts.service_penalty_.def);
  pbf_costing_options->set_service_factor(shared_opts.service_factor_.def);

  pbf_costing_options->set_use_tracks(shared_opts.use_tracks_.def);
  pbf_costing_options->set_use_living_streets(shared_opts.use_living_streets_.def);

  pbf_costing_options->set_closure_factor(shared_opts.closure_factor_.def);

  pbf_costing_options->set_exclude_unpaved(shared_opts.exclude_unpaved_);
  pbf_costing_options->set_exclude_cash_only_tolls(shared_opts.exclude_cash_only_tolls_);

  pbf_costing_options->set_include_hot(shared_opts.include_hot_);
  pbf_costing_options->set_include_hov2(shared_opts.include_hov2_);
  pbf_costing_options->set_include_hov3(shared_opts.include_hov3_);
}

void ParseCostingOptions(const rapidjson::Document& doc,
                         const std::string& costing_options_key,
                         Options& options) {
  // if specified, get the costing options in there
  for (int i = 0; i < Costing_ARRAYSIZE; ++i) {
    Costing costing = static_cast<Costing>(i);
    // Create the costing string
    const auto& costing_str = valhalla::Costing_Enum_Name(costing);
    // Deprecated costings still need their place in the array
    if (costing_str.empty()) {
      options.add_costing_options();
      continue;
    }
    // Create the costing options key
    const auto key = costing_options_key + "/" + costing_str;
    // Parse the costing options
    ParseCostingOptions(doc, key, options.add_costing_options(), costing);
  }
}

void ParseCostingOptions(const rapidjson::Document& doc,
                         const std::string& key,
                         CostingOptions* costing_options,
                         Costing costing) {
  // if the costing wasnt specified we have to find it nested in the json object
  if (costing == Costing_ARRAYSIZE) {
    // it has to have a costing object, it has to be an object, it has to have a member
    // named costing and the value of that member has to be a string type
    auto json = rapidjson::get_child_optional(doc, key.c_str());
    decltype(json->MemberBegin()) costing_itr;
    if (!json || !json->IsObject() ||
        (costing_itr = json->FindMember("costing")) == json->MemberEnd() ||
        !costing_itr->value.IsString()) {
      throw valhalla_exception_t{127};
    }
    // then we can try to parse the string and if its invalid we barf
    std::string costing_str = costing_itr->value.GetString();
    if (!Costing_Enum_Parse(costing_str, &costing)) {
      throw valhalla_exception_t{125, "'" + costing_str + "'"};
    }
  }
  // finally we can parse the costing
  switch (costing) {
    case auto_: {
      sif::ParseAutoCostOptions(doc, key, costing_options);
      break;
    }
    case bicycle: {
      sif::ParseBicycleCostOptions(doc, key, costing_options);
      break;
    }
    case bus: {
      sif::ParseBusCostOptions(doc, key, costing_options);
      break;
    }
    case taxi: {
      sif::ParseTaxiCostOptions(doc, key, costing_options);
      break;
    }
    case motor_scooter: {
      sif::ParseMotorScooterCostOptions(doc, key, costing_options);
      break;
    }
    case multimodal: {
      costing_options->set_costing(Costing::multimodal); // Nothing to parse for this one
      break;
    }
    case pedestrian: {
      sif::ParsePedestrianCostOptions(doc, key, costing_options);
      break;
    }
    case bikeshare: {
      costing_options->set_costing(Costing::bikeshare); // Nothing to parse for this one
      break;
    }
    case transit: {
      sif::ParseTransitCostOptions(doc, key, costing_options);
      break;
    }
    case truck: {
      sif::ParseTruckCostOptions(doc, key, costing_options);
      break;
    }
    case motorcycle: {
      sif::ParseMotorcycleCostOptions(doc, key, costing_options);
      break;
    }
    case none_: {
      sif::ParseNoCostOptions(doc, key, costing_options);
      break;
    }
  }
  costing_options->set_costing(costing);
}

} // namespace sif
} // namespace valhalla
