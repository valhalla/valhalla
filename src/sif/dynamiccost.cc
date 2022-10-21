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
constexpr ranged_default_t<uint32_t> kFixedSpeedRange{0, baldr::kDisableFixedSpeed,
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

DynamicCost::DynamicCost(const Costing& costing,
                         const TravelMode mode,
                         uint32_t access_mask,
                         bool penalize_uturns)
    : pass_(0), allow_transit_connections_(false), allow_destination_only_(true),
      allow_conditional_destination_(false), travel_mode_(mode), access_mask_(access_mask),
      closure_factor_(kDefaultClosureFactor), flow_mask_(kDefaultFlowMask),
      shortest_(costing.options().shortest()),
      ignore_restrictions_(costing.options().ignore_restrictions()),
      ignore_oneways_(costing.options().ignore_oneways()),
      ignore_access_(costing.options().ignore_access()),
      ignore_closures_(costing.options().ignore_closures()),
      top_speed_(costing.options().top_speed()), fixed_speed_(costing.options().fixed_speed()),
      filter_closures_(ignore_closures_ ? false : costing.filter_closures()),
      penalize_uturns_(penalize_uturns) {
  // Parse property tree to get hierarchy limits
  // TODO - get the number of levels
  uint32_t n_levels = sizeof(kDefaultMaxUpTransitions) / sizeof(kDefaultMaxUpTransitions[0]);
  for (uint32_t level = 0; level < n_levels; level++) {
    hierarchy_limits_.emplace_back(HierarchyLimits(level));
  }

  // Add avoid edges to internal set
  for (auto& edge : costing.options().exclude_edges()) {
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
  return EdgeCost(edge, tile, TimeInfo::invalid(), flow_sources);
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

void ParseBaseCostOptions(const rapidjson::Value& json,
                          Costing* c,
                          const BaseCostingOptionsConfig& cfg) {
  auto* co = c->mutable_options();

  // ignore bogus input
  if (co->has_flow_mask_case() && co->flow_mask() > kDefaultFlowMask)
    co->clear_flow_mask();

  // defer to json or defaults if no pbf is present
  auto speed_types = rapidjson::get_child_optional(json, "/speed_types");
  if (speed_types || !co->has_flow_mask_case())
    co->set_flow_mask(SpeedMask_Parse(speed_types));

  // named costing
  auto name = rapidjson::get_optional<std::string>(json, "/name");
  if (name) {
    c->set_name(*name);
  }

  // various traversability flags
  JSON_PBF_DEFAULT(co, false, json, "/ignore_restrictions", ignore_restrictions);
  JSON_PBF_DEFAULT(co, false, json, "/ignore_oneways", ignore_oneways);
  JSON_PBF_DEFAULT(co, false, json, "/ignore_access", ignore_access);
  JSON_PBF_DEFAULT(co, false, json, "/ignore_closures", ignore_closures);

  // shortest
  JSON_PBF_DEFAULT(co, false, json, "/shortest", shortest);

  // top speed
  JSON_PBF_RANGED_DEFAULT(co, kVehicleSpeedRange, json, "/top_speed", top_speed);

  // destination only penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.dest_only_penalty_, json, "/destination_only_penalty",
                          destination_only_penalty);

  // maneuver_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.maneuver_penalty_, json, "/maneuver_penalty", maneuver_penalty);

  // alley_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.alley_penalty_, json, "/alley_penalty", alley_penalty);

  // gate_cost
  JSON_PBF_RANGED_DEFAULT(co, cfg.gate_cost_, json, "/gate_cost", gate_cost);

  // gate_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.gate_penalty_, json, "/gate_penalty", gate_penalty);

  // private_access_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.private_access_penalty_, json, "/private_access_penalty",
                          private_access_penalty);

  // country_crossing_cost
  JSON_PBF_RANGED_DEFAULT(co, cfg.country_crossing_cost_, json, "/country_crossing_cost",
                          country_crossing_cost);

  // country_crossing_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.country_crossing_penalty_, json, "/country_crossing_penalty",
                          country_crossing_penalty);

  if (!cfg.disable_toll_booth_) {
    // toll_booth_cost
    JSON_PBF_RANGED_DEFAULT(co, cfg.toll_booth_cost_, json, "/toll_booth_cost", toll_booth_cost);

    // toll_booth_penalty
    JSON_PBF_RANGED_DEFAULT(co, cfg.toll_booth_penalty_, json, "/toll_booth_penalty",
                            toll_booth_penalty);
  }

  if (!cfg.disable_ferry_) {
    // ferry_cost
    JSON_PBF_RANGED_DEFAULT(co, cfg.ferry_cost_, json, "/ferry_cost", ferry_cost);

    // use_ferry
    JSON_PBF_RANGED_DEFAULT(co, cfg.use_ferry_, json, "/use_ferry", use_ferry);
  }

  if (!cfg.disable_rail_ferry_) {
    // rail_ferry_cost
    JSON_PBF_RANGED_DEFAULT(co, cfg.rail_ferry_cost_, json, "/rail_ferry_cost", rail_ferry_cost);

    // use_rail_ferry
    JSON_PBF_RANGED_DEFAULT(co, cfg.use_rail_ferry_, json, "/use_rail_ferry", use_rail_ferry);
  }

  JSON_PBF_DEFAULT(co, cfg.exclude_unpaved_, json, "/exclude_unpaved", exclude_unpaved);

  JSON_PBF_DEFAULT(co, cfg.exclude_cash_only_tolls_, json, "/exclude_cash_only_tolls",
                   exclude_cash_only_tolls);

  // service_penalty
  JSON_PBF_RANGED_DEFAULT(co, cfg.service_penalty_, json, "/service_penalty", service_penalty);

  // service_factor
  JSON_PBF_RANGED_DEFAULT(co, cfg.service_factor_, json, "/service_factor", service_factor);

  // use_tracks
  JSON_PBF_RANGED_DEFAULT(co, cfg.use_tracks_, json, "/use_tracks", use_tracks);

  // use_living_streets
  JSON_PBF_RANGED_DEFAULT(co, cfg.use_living_streets_, json, "/use_living_streets",
                          use_living_streets);

  // closure_factor
  JSON_PBF_RANGED_DEFAULT(co, cfg.closure_factor_, json, "/closure_factor", closure_factor);

  // HOT/HOV
  JSON_PBF_DEFAULT(co, cfg.include_hot_, json, "/include_hot", include_hot);
  JSON_PBF_DEFAULT(co, cfg.include_hov2_, json, "/include_hov2", include_hov2);
  JSON_PBF_DEFAULT(co, cfg.include_hov3_, json, "/include_hov3", include_hov3);

  co->set_fixed_speed(
      kFixedSpeedRange(rapidjson::get<uint32_t>(json, "/fixed_speed", co->fixed_speed())));
}

void ParseCosting(const rapidjson::Document& doc,
                  const std::string& costing_options_key,
                  Options& options) {
  // if specified, get the costing options in there
  for (auto i = Costing::Type_MIN; i <= Costing::Type_MAX; i = Costing::Type(i + 1)) {
    // Create the costing options key
    const auto& costing_str = valhalla::Costing_Enum_Name(i);
    if (costing_str.empty())
      continue;
    const auto key = costing_options_key + "/" + costing_str;
    // Parse the costing options
    auto& costing = (*options.mutable_costings())[i];
    ParseCosting(doc, key, &costing, i);
  }
}

void ParseCosting(const rapidjson::Document& doc,
                  const std::string& key,
                  Costing* costing,
                  Costing::Type costing_type) {
  // if the costing wasnt specified we have to find it nested in the json object
  if (costing_type == Costing::Type_ARRAYSIZE) {
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
    if (!Costing_Enum_Parse(costing_str, &costing_type)) {
      throw valhalla_exception_t{125, "'" + costing_str + "'"};
    }
  }
  // finally we can parse the costing
  switch (costing_type) {
    case Costing::auto_: {
      sif::ParseAutoCostOptions(doc, key, costing);
      break;
    }
    case Costing::bicycle: {
      sif::ParseBicycleCostOptions(doc, key, costing);
      break;
    }
    case Costing::bus: {
      sif::ParseBusCostOptions(doc, key, costing);
      break;
    }
    case Costing::taxi: {
      sif::ParseTaxiCostOptions(doc, key, costing);
      break;
    }
    case Costing::motor_scooter: {
      sif::ParseMotorScooterCostOptions(doc, key, costing);
      break;
    }
    case Costing::multimodal: {
      costing->set_type(Costing::multimodal); // Nothing to parse for this one
      break;
    }
    case Costing::pedestrian: {
      sif::ParsePedestrianCostOptions(doc, key, costing);
      break;
    }
    case Costing::bikeshare: {
      costing->set_type(Costing::bikeshare); // Nothing to parse for this one
      break;
    }
    case Costing::transit: {
      sif::ParseTransitCostOptions(doc, key, costing);
      break;
    }
    case Costing::truck: {
      sif::ParseTruckCostOptions(doc, key, costing);
      break;
    }
    case Costing::motorcycle: {
      sif::ParseMotorcycleCostOptions(doc, key, costing);
      break;
    }
    case Costing::none_: {
      sif::ParseNoCostOptions(doc, key, costing);
      break;
    }
    default: {
      throw std::logic_error("Unknown costing");
    }
  }
  costing->set_type(costing_type);
}

} // namespace sif
} // namespace valhalla
