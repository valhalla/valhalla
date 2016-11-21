#include <string.h>
#include "sif/edgelabel.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default constructor
EdgeLabel::EdgeLabel() {
  memset(this, 0, sizeof(EdgeLabel));
}

// Constructor with values.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
                     const DirectedEdge* edge, const Cost& cost,
                     const float sortcost, const float dist,
                     const TravelMode mode, const uint32_t path_distance)
    : predecessor_(predecessor),
      edgeid_(edgeid),
      opp_edgeid_ {},
      endnode_(edge->endnode()),
      cost_(cost),
      sortcost_(sortcost),
      distance_(dist),
      path_distance_(path_distance),
      use_(static_cast<uint32_t>(edge->use())),
      opp_index_(edge->opp_index()),
      opp_local_idx_(edge->opp_local_idx()),
      restrictions_(edge->restrictions()),
      shortcut_(edge->shortcut()),
      mode_(static_cast<uint32_t>(mode)),
      dest_only_(edge->destonly()),
      has_transit_(0),
      origin_(0),
      toll_(edge->toll()),
      classification_(static_cast<uint32_t>(edge->classification())),
      tripid_(0),
      blockid_(0),
      transit_operator_(0),
      transition_cost_(0),
      transition_secs_(0),
      on_complex_rest_(edge->part_of_complex_restriction()),
      not_thru_(edge->not_thru()),
      not_thru_pruning_(false),
      deadend_(edge->deadend()) {
}

// Constructor with values - used in bidirectional A*
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
                     const GraphId& oppedgeid, const DirectedEdge* edge,
                     const Cost& cost, const float sortcost, const float dist,
                     const TravelMode mode, const Cost& tc,
                     bool not_thru_pruning)
    : predecessor_(predecessor),
      edgeid_(edgeid),
      opp_edgeid_(oppedgeid),
      endnode_(edge->endnode()),
      cost_(cost),
      sortcost_(sortcost),
      distance_(dist),
      path_distance_(0),
      use_(static_cast<uint32_t>(edge->use())),
      opp_index_(edge->opp_index()),
      opp_local_idx_(edge->opp_local_idx()),
      restrictions_(edge->restrictions()),
      shortcut_(edge->shortcut()),
      mode_(static_cast<uint32_t>(mode)),
      dest_only_(edge->destonly()),
      has_transit_(0),
      origin_(0),
      toll_(edge->toll()),
      classification_(static_cast<uint32_t>(edge->classification())),
      tripid_(0),
      blockid_(0),
      transit_operator_(0),
      transition_cost_(tc.cost),
      transition_secs_(tc.secs),
      on_complex_rest_(edge->part_of_complex_restriction()),
      not_thru_(edge->not_thru()),
      not_thru_pruning_(not_thru_pruning),
      deadend_(edge->deadend()) {
}

// Constructor with values.  Used for multi-modal path.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
          const DirectedEdge* edge, const Cost& cost,
          const float sortcost, const float dist,
          const TravelMode mode, const uint32_t path_distance,
          const uint32_t tripid, const GraphId& prior_stopid,
          const uint32_t blockid, const uint32_t transit_operator,
          const bool has_transit)
    : predecessor_(predecessor),
      edgeid_(edgeid),
      opp_edgeid_(prior_stopid),
      endnode_(edge->endnode()),
      cost_(cost),
      sortcost_(sortcost),
      distance_(dist),
      path_distance_(path_distance),
      use_(static_cast<uint32_t>(edge->use())),
      opp_index_(edge->opp_index()),
      opp_local_idx_(edge->opp_local_idx()),
      restrictions_(edge->restrictions()),
      shortcut_(edge->shortcut()),
      mode_(static_cast<uint32_t>(mode)),
      dest_only_(edge->destonly()),
      has_transit_(has_transit),
      origin_(0),
      toll_(edge->toll()),
      classification_(static_cast<uint32_t>(edge->classification())),
      tripid_(tripid),
      blockid_(blockid),
      transit_operator_(transit_operator),
      transition_cost_(0),
      transition_secs_(0),
      on_complex_rest_(edge->part_of_complex_restriction()),
      not_thru_(edge->not_thru()),
      not_thru_pruning_(false),
      deadend_(edge->deadend()) {
}

// Constructor with values - used in time distance matrix (needs the
// accumulated distance as well as opposing edge information). Sets
// sortcost to the true cost.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
                const GraphId& oppedgeid, const DirectedEdge* edge,
                const Cost& cost, const TravelMode mode,
                const Cost& tc, const uint32_t path_distance,
                bool not_thru_pruning)
    :  predecessor_(predecessor),
       edgeid_(edgeid),
       opp_edgeid_(oppedgeid),
       endnode_(edge->endnode()),
       cost_(cost),
       sortcost_(cost.cost),
       distance_(0.0f),
       path_distance_(path_distance),
       use_(static_cast<uint32_t>(edge->use())),
       opp_index_(edge->opp_index()),
       opp_local_idx_(edge->opp_local_idx()),
       restrictions_(edge->restrictions()),
       shortcut_(edge->shortcut()),
       mode_(static_cast<uint32_t>(mode)),
       dest_only_(edge->destonly()),
       has_transit_(0),
       origin_(0),
       toll_(edge->toll()),
       classification_(static_cast<uint32_t>(edge->classification())),
       tripid_(0),
       blockid_(0),
       transit_operator_(0),
       transition_cost_(tc.cost),
       transition_secs_(tc.secs),
       on_complex_rest_(edge->part_of_complex_restriction()),
       not_thru_(edge->not_thru()),
       not_thru_pruning_(not_thru_pruning),
       deadend_(edge->deadend()) {
}

// Constructor given a predecessor edge label. This is used for hierarchy
// transitions where the attributes at the predecessor are needed (rather
// than attributes from the directed edge.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
                     const GraphId& endnode, const EdgeLabel& pred) {
  *this        = pred;
  predecessor_ = predecessor;
  edgeid_      = edgeid;
  endnode_     = endnode;
  origin_      = 0;

  // Set the use so we know this is a transition edge. For now we only need to
  // know it is a transition edge so we can skip it in complex restrictions.
  use_ = static_cast<uint32_t>(Use::kTransitionUp);
}

// Update predecessor and cost values in the label.
void EdgeLabel::Update(const uint32_t predecessor, const Cost& cost,
                       const float sortcost) {
  predecessor_ = predecessor;
  cost_ = cost;
  sortcost_ = sortcost;
}

// Update an existing edge label with new predecessor and cost information.
// Update distance as well (used in time distance matrix)
void EdgeLabel::Update(const uint32_t predecessor, const Cost& cost,
                       const float sortcost, const Cost& tc,
                       const uint32_t distance) {
  predecessor_ = predecessor;
  cost_ = cost;
  sortcost_ = sortcost;
  transition_cost_  = tc.cost;
  transition_secs_  = tc.secs;
  path_distance_ = distance;
}

// Update an existing edge label with new predecessor and cost information.
// Update transit information: prior stop Id will stay the same but trip Id
// and block Id may change (a new trip at an earlier departure time).
// The mode, edge Id, and end node remain the same.
void EdgeLabel::Update(const uint32_t predecessor, const Cost& cost,
          const float sortcost, const uint32_t path_distance,
          const uint32_t tripid,  const uint32_t blockid) {
  predecessor_ = predecessor;
  cost_ = cost;
  sortcost_ = sortcost;
  path_distance_ = path_distance;
  tripid_ = tripid;
  blockid_ = blockid;
}

// Get the predecessor edge label index.
uint32_t EdgeLabel::predecessor() const {
  return predecessor_;
}

// Get the GraphId of this edge.
const GraphId& EdgeLabel::edgeid() const {
  return edgeid_;
}

// Get the prior transit stop Id.
const GraphId& EdgeLabel::prior_stopid() const {
  return opp_edgeid_;
}

// Get the GraphId of the opposing directed edge.
const GraphId& EdgeLabel::opp_edgeid() const {
  return opp_edgeid_;
}

// Get the end node of the predecessor edge.
const GraphId& EdgeLabel::endnode() const {
  return endnode_;
}

// Get the cost from the origin to this directed edge.
const Cost& EdgeLabel::cost() const {
  return cost_;
}

// Get the sort cost.
float EdgeLabel::sortcost() const {
  return sortcost_;
}

// Set the sort cost
void EdgeLabel::SetSortCost(float sortcost) {
  sortcost_ = sortcost;
}

// Get the distance to the destination.
float EdgeLabel::distance() const {
  return distance_;
}

// Get the use of the directed edge.
Use EdgeLabel::use() const {
  return static_cast<Use>(use_);
}

// Get the opposing local index. This is the index of the incoming edge
// (on the local hierarchy) at the end node of the predecessor directed
// edge. This is used for edge transition costs and Uturn detection.
uint32_t EdgeLabel::opp_local_idx() const {
  return opp_local_idx_;
}

// Get the restriction mask at the end node. Each bit set to 1 indicates a
// turn restriction onto the directed edge with matching local edge index.
uint32_t EdgeLabel::restrictions() const {
  return restrictions_;
}

// Get the shortcut flag.
bool EdgeLabel::shortcut() const {
  return shortcut_;
}

// Get the travel mode along this edge.
TravelMode EdgeLabel::mode() const {
  return static_cast<TravelMode>(mode_);
}

// Get the destination only flag.
bool EdgeLabel::destonly() const {
  return dest_only_;
}

// Has any transit been taken up to this point on the path.
bool EdgeLabel::has_transit() const {
  return has_transit_;
}

// Is this edge an origin edge?
bool EdgeLabel::origin() const {
  return origin_;
}

// Sets this edge as an origin.
void EdgeLabel::set_origin() {
  origin_ = true;
}

// Does this edge have a toll.
bool EdgeLabel::toll() const {
  return toll_;
}

// Get the opposing index - for bidirectional A*.
uint32_t EdgeLabel::opp_index() const {
  return opp_index_;
}

// Get the current accumulated path distance in meters.
uint32_t EdgeLabel::path_distance() const {
  return path_distance_;
}

// Get the predecessor road classification.
RoadClass EdgeLabel::classification() const {
  return static_cast<RoadClass>(classification_);
}

// Should not thru pruning be enabled on this path?
bool EdgeLabel::not_thru_pruning() const {
  return not_thru_pruning_;
}

// Get the transit trip Id.
uint32_t EdgeLabel::tripid() const {
  return tripid_;
}

// Return the transit block Id of the prior trip.
uint32_t EdgeLabel::blockid() const {
  return blockid_;
}

// Get the index of the transit operator.
uint32_t EdgeLabel::transit_operator() const {
  return transit_operator_;
}

// Get the transition cost in seconds. This is used in the bidirectional A*
// to determine the cost at the connection.
uint32_t EdgeLabel::transition_cost() const {
  return transition_cost_;
}

// Get the transition cost in seconds. This is used in the bidirectional A*
// reverse path search to allow the recovery of the true elapsed time along
// the path. This is needed since the transition cost is applied at a different
// node than the forward search.
uint32_t EdgeLabel::transition_secs() const {
  return transition_secs_;
}

// Set the transition cost.
void EdgeLabel::set_transition_cost(const Cost& tc) {
  transition_cost_ = tc.cost;
  transition_secs_ = tc.secs;
}

// Is this edge part of a complex restriction.
bool EdgeLabel::on_complex_rest() const {
  return on_complex_rest_;
}

// Is this edge not-through
bool EdgeLabel::not_thru() const {
  return not_thru_;
}

// Set the not-through flag for this edge.
void EdgeLabel::set_not_thru(const bool not_thru) {
  not_thru_ = not_thru;
}

// Is this edge a dead end.
bool EdgeLabel::deadend() const {
  return deadend_;
}

// Operator for sorting.
bool EdgeLabel::operator < (const EdgeLabel& other) const {
  return sortcost() < other.sortcost();
}

}
}
