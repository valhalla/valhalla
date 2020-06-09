#include "sif/dynamiccost.h"
#include "baldr/graphconstants.h"

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

DynamicCost::DynamicCost(const Options& options, const TravelMode mode)
    : pass_(0), allow_transit_connections_(false), allow_destination_only_(true), travel_mode_(mode),
      flow_mask_(kDefaultFlowMask) {
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
Cost DynamicCost::EdgeCost(const baldr::DirectedEdge* edge, const baldr::GraphTile* tile) const {
  return EdgeCost(edge, tile, kInvalidSecondsOfWeek);
}

// Returns the cost to make the transition from the predecessor edge.
// Defaults to 0. Costing models that wish to include edge transition
// costs (i.e., intersection/turn costs) must override this method.
Cost DynamicCost::TransitionCost(const DirectedEdge* edge,
                                 const NodeInfo* node,
                                 const EdgeLabel& pred) const {
  return {0.0f, 0.0f};
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// Defaults to 0. Costing models that wish to include edge transition
// costs (i.e., intersection/turn costs) must override this method.
Cost DynamicCost::TransitionCostReverse(const uint32_t idx,
                                        const baldr::NodeInfo* node,
                                        const baldr::DirectedEdge* opp_edge,
                                        const baldr::DirectedEdge* opp_pred_edge) const {
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
void DynamicCost::AddToExcludeList(const baldr::GraphTile*& tile) {
  ;
}

// Checks if we should exclude or not.
bool DynamicCost::IsExcluded(const baldr::GraphTile*& tile, const baldr::DirectedEdge* edge) {
  return false;
}

// Checks if we should exclude or not.
bool DynamicCost::IsExcluded(const baldr::GraphTile*& tile, const baldr::NodeInfo* node) {
  return false;
}

// Adds a list of edges (GraphIds) to the user specified avoid list.
void DynamicCost::AddUserAvoidEdges(const std::vector<AvoidEdge>& avoid_edges) {
  for (auto edge : avoid_edges) {
    user_avoid_edges_.insert({edge.id, edge.percent_along});
  }
}

void ParseCostOptions(const rapidjson::Value& value, CostingOptions* pbf_costing_options) {
  auto speed_types = rapidjson::get_child_optional(value, "/speed_types");
  pbf_costing_options->set_flow_mask(SpeedMask_Parse(speed_types));
}

} // namespace sif
} // namespace valhalla
