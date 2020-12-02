#include "sif/dynamiccost.h"

#include "baldr/graphconstants.h"
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

DynamicCost::DynamicCost(const CostingOptions& options, const TravelMode mode, uint32_t access_mask)
    : pass_(0), allow_transit_connections_(false), allow_destination_only_(true), travel_mode_(mode),
      access_mask_(access_mask), flow_mask_(kDefaultFlowMask), shortest_(options.shortest()),
      ignore_restrictions_(options.ignore_restrictions()), ignore_oneways_(options.ignore_oneways()),
      ignore_access_(options.ignore_access()), ignore_closures_(options.ignore_closures()),
      top_speed_(options.top_speed()) {
  // Parse property tree to get hierarchy limits
  // TODO - get the number of levels
  uint32_t n_levels = sizeof(kDefaultMaxUpTransitions) / sizeof(kDefaultMaxUpTransitions[0]);
  for (uint32_t level = 0; level < n_levels; level++) {
    hierarchy_limits_.emplace_back(HierarchyLimits(level));
  }

  // Add avoid edges to internal set
  for (auto& edge : options.avoid_edges()) {
    user_avoid_edges_.insert({GraphId(edge.id()), edge.percent_along()});
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
Cost DynamicCost::EdgeCost(const baldr::DirectedEdge* edge,
                           const boost::intrusive_ptr<const baldr::GraphTile>& tile) const {
  return EdgeCost(edge, std::move(tile), kConstrainedFlowSecondOfDay);
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
                                        const baldr::DirectedEdge*) const {
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
void DynamicCost::RelaxHierarchyLimits(const float factor, const float expansion_within_factor) {
  for (auto& hierarchy : hierarchy_limits_) {
    hierarchy.Relax(factor, expansion_within_factor);
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
void DynamicCost::AddToExcludeList(const boost::intrusive_ptr<const baldr::GraphTile>&) {
}

// Checks if we should exclude or not.
bool DynamicCost::IsExcluded(const boost::intrusive_ptr<const baldr::GraphTile>&,
                             const baldr::DirectedEdge*) {
  return false;
}

// Checks if we should exclude or not.
bool DynamicCost::IsExcluded(const boost::intrusive_ptr<const baldr::GraphTile>&,
                             const baldr::NodeInfo*) {
  return false;
}

// Adds a list of edges (GraphIds) to the user specified avoid list.
void DynamicCost::AddUserAvoidEdges(const std::vector<AvoidEdge>& avoid_edges) {
  for (auto edge : avoid_edges) {
    user_avoid_edges_.insert({edge.id, edge.percent_along});
  }
}

Cost DynamicCost::BSSCost() const {
  return kNoCost;
};

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
    case hov: {
      sif::ParseHOVCostOptions(doc, key, costing_options);
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
