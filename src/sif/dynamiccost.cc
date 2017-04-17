#include "sif/dynamiccost.h"
#include "baldr/double_bucket_queue.h" // For kInvalidLabel

using namespace valhalla::baldr;

namespace {

using namespace valhalla::sif;

// Check for complex restriction
bool IsRestricted(const EdgeLabel& pred, const std::vector<EdgeLabel>& edge_labels,
                  const std::vector<ComplexRestriction>& restrictions,
                  const bool forward) {
  // Lambda to get the next predecessor EdgeLabel (that is not a transition)
  auto next_predecessor = [&edge_labels](const EdgeLabel* label) {
    // Get the next predecessor - make sure it is valid. Continue to get
    // the next predecessor if the edge is a transition edge.
    const EdgeLabel* next_pred = (label->predecessor() == kInvalidLabel) ?
                    label : &edge_labels[label->predecessor()];
    while (next_pred->use() == Use::kTransitionUp &&
           next_pred->predecessor() != kInvalidLabel) {
      next_pred = &edge_labels[next_pred->predecessor()];
    }
    return next_pred;
  };

  // Get the first predecessor edge (that is not a transition)
  const EdgeLabel* first_pred = &pred;
  if (first_pred->use() == Use::kTransitionUp) {
    first_pred = next_predecessor(first_pred);
  }

  // Iterate through the restrictions
  for (const auto& cr : restrictions) {
    // Walk the via list, break if the via edge Ids do not match the path
    const EdgeLabel* next_pred = first_pred;
    for (const auto& via_id : cr.GetVias()) {
      if (via_id != next_pred->edgeid()) {
        return false;
      }
      next_pred = next_predecessor(next_pred);
    }

    // Check against the start/end of the complex restriction
    if (( forward && next_pred->edgeid() == cr.from_id()) ||
        (!forward && next_pred->edgeid() == cr.to_id())) {
      return true;
    }
  }
  return false;
}

}

namespace valhalla{
namespace sif {

DynamicCost::DynamicCost(const boost::property_tree::ptree& pt,
                         const TravelMode mode)
    : allow_transit_connections_(false),
      disable_destination_only_(false),
      travel_mode_(mode) {
  // Parse property tree to get hierarchy limits
  // TODO - get the number of levels
  uint32_t n_levels = sizeof(kDefaultMaxUpTransitions) /
      sizeof(kDefaultMaxUpTransitions[0]);
  for (uint32_t level = 0; level < n_levels; level++) {
    hierarchy_limits_.emplace_back(HierarchyLimits(pt, level));
  }

  // Parse property tree to get avoid edges
  auto avoid_edges = pt.get_child_optional("avoid_edges");
  if (avoid_edges) {
    for (auto& edgeid : *avoid_edges) {
      user_avoid_edges_.insert(GraphId(edgeid.second.get_value<uint64_t>()));
    }
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

// Get the cost to traverse the specified directed edge using a transit
// departure (schedule based edge traversal). Cost includes
// the time (seconds) to traverse the edge. Only transit cost models override
// this method.
Cost DynamicCost::EdgeCost(const baldr::DirectedEdge* edge,
              const baldr::TransitDeparture* departure,
              const uint32_t curr_time) const {
  return { 0.0f, 0.0f };
}

// Returns the cost to make the transition from the predecessor edge.
// Defaults to 0. Costing models that wish to include edge transition
// costs (i.e., intersection/turn costs) must override this method.
Cost DynamicCost::TransitionCost(const DirectedEdge* edge,
                                 const NodeInfo* node,
                                 const EdgeLabel& pred) const {
  return { 0.0f, 0.0f };
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// Defaults to 0. Costing models that wish to include edge transition
// costs (i.e., intersection/turn costs) must override this method.
Cost DynamicCost::TransitionCostReverse(
    const uint32_t idx, const baldr::NodeInfo* node,
    const baldr::DirectedEdge* opp_edge,
    const baldr::DirectedEdge* opp_pred_edge) const {
  return { 0.0f, 0.0f };
}

/**
 * Test if an edge should be restricted due to a complex restriction.
 */
bool DynamicCost::Restricted(const DirectedEdge* edge,
                             const EdgeLabel& pred,
                             const std::vector<EdgeLabel>& edgelabels,
                             const baldr::GraphTile*& tile,
                             const baldr::GraphId& edgeid,
                             const bool forward) const {
  // If forward, check if the edge marks the end of a restriction, else check
  // if the edge marks the start of a complex restriction.
  bool has_restriction = (forward) ?
      edge->end_restriction()   & access_mode() :
      edge->start_restriction() & access_mode();
  if (has_restriction) {
    // Get complex restrictions. Return false if no restrictions are found
    auto restrictions = tile->GetRestrictions(forward, edgeid, access_mode());
    if (restrictions.size() == 0) {
      return false;
    }
    return IsRestricted(pred, edgelabels, restrictions, forward);
  } else {
    return false;
  }
}

// Returns the transfer cost between 2 transit stops.
Cost DynamicCost::TransferCost() const {
  return { 0.0f, 0.0f };
}

// Returns the default transfer cost between 2 transit stops.
Cost DynamicCost::DefaultTransferCost() const {
  return { 0.0f, 0.0f };
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

// Set to disable use of destination-only segments.
void DynamicCost::DisableDestinationOnly() {
  disable_destination_only_ = true;
}

// Set to allow use of destination-only segments.
void DynamicCost::EnableDestinationOnly() {
  disable_destination_only_ = false;
}

// Returns the maximum transfer distance between stops that you are willing
// to travel for this mode.  It is the max distance you are willing to
// travel between transfers.
uint32_t DynamicCost::GetMaxTransferDistanceMM() {
  return 0;
}

// This method overrides the weight for this mode.  The higher the value
// the more the mode is favored.
float DynamicCost::GetModeWeight() {
  return 1.0f;
}

// This method overrides the max_distance with the max_distance_mm per segment
// distance. An example is a pure walking route may have a max distance of
// 10000 meters (10km) but for a multi-modal route a lower limit of 5000
// meters per segment (e.g. from origin to a transit stop or from the last
// transit stop to the destination).
void DynamicCost::UseMaxMultiModalDistance() {
  ;
}

// Gets the hierarchy limits.
std::vector<HierarchyLimits>& DynamicCost::GetHierarchyLimits() {
  return hierarchy_limits_;
}

// Relax hierarchy limits.
void DynamicCost::RelaxHierarchyLimits(const float factor,
                                       const float expansion_within_factor) {
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
bool DynamicCost::IsExcluded(const baldr::GraphTile*& tile,
                             const baldr::DirectedEdge* edge) {
  return false;
}

// Checks if we should exclude or not.
bool DynamicCost::IsExcluded(const baldr::GraphTile*& tile,
                             const baldr::NodeInfo* node) {
  return false;
}

// Adds a list of edges (GraphIds) to the user specified avoid list.
void DynamicCost::AddUserAvoidEdges(const std::vector<GraphId>& avoid_edges) {
  for (auto edgeid : avoid_edges) {
    user_avoid_edges_.insert(edgeid);
  }
}

}
}
