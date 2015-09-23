#include "sif/edgelabel.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default constructor
EdgeLabel::EdgeLabel()
    : predecessor_(kInvalidLabel),
      edgeid_(GraphId()),
      cost_{0.0f, 0.0f},
      sortcost_(0.0f),
      distance_(0.0f),
      attributes_{},
      walking_distance_(0.0f),
      tripid_(0),
      prior_stopid_(0),
      blockid_(0) {
}

// Constructor with values.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
                     const DirectedEdge* edge, const Cost& cost,
                     const float sortcost, const float dist,
                     const uint32_t restrictions,
                     const uint32_t opp_local_idx,
                     const TravelMode mode)
    : predecessor_(predecessor),
      edgeid_(edgeid),
      endnode_(edge->endnode()),
      cost_(cost),
      sortcost_(sortcost),
      distance_(dist),
      walking_distance_(0),
      tripid_(0),
      prior_stopid_(0),
      blockid_(0) {
  attributes_.use           = static_cast<uint32_t>(edge->use());
  attributes_.opp_local_idx = opp_local_idx;
  attributes_.restrictions  = restrictions;
  attributes_.trans_up      = edge->trans_up();
  attributes_.trans_down    = edge->trans_down();
  attributes_.shortcut      = edge->shortcut();
  attributes_.mode          = static_cast<uint32_t>(mode);
  attributes_.dest_only     = edge->destonly();
}

// Constructor with values.  Used for multi-modal path.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const baldr::GraphId& edgeid,
          const baldr::DirectedEdge* edge, const Cost& cost,
          const float sortcost, const float dist,
          const uint32_t restrictions, const uint32_t opp_local_idx,
          const TravelMode mode, const uint32_t walking_distance,
          const uint32_t tripid, const uint32_t prior_stopid,
          const uint32_t blockid, const bool has_transit)
    : predecessor_(predecessor),
      edgeid_(edgeid),
      endnode_(edge->endnode()),
      cost_(cost),
      sortcost_(sortcost),
      distance_(dist),
      walking_distance_(walking_distance),
      tripid_(tripid),
      prior_stopid_(prior_stopid),
      blockid_(blockid) {
  attributes_.use           = static_cast<uint32_t>(edge->use());
  attributes_.opp_local_idx = opp_local_idx;
  attributes_.restrictions  = restrictions;
  attributes_.trans_up      = edge->trans_up();
  attributes_.trans_down    = edge->trans_down();
  attributes_.shortcut      = edge->shortcut();
  attributes_.mode          = static_cast<uint32_t>(mode);
  attributes_.dest_only     = edge->destonly();
  attributes_.has_transit   = has_transit;
}

// Destructor
EdgeLabel::~EdgeLabel() {
}

// Update predecessor and cost values in the label.
void EdgeLabel::Update(const uint32_t predecessor, const Cost& cost,
                       const float sortcost) {
  predecessor_ = predecessor;
  cost_ = cost;
  sortcost_ = sortcost;
}

// Update an existing edge label with new predecessor and cost information.
// Update transit information: prior stop Id will stay the same but trip Id
// and block Id may change (a new trip at an earlier departure time).
// The mode, edge Id, and end node remain the same.
void EdgeLabel::Update(const uint32_t predecessor, const Cost& cost,
          const float sortcost, const uint32_t walking_distance,
          const uint32_t tripid,  const uint32_t blockid) {
  predecessor_ = predecessor;
  cost_ = cost;
  sortcost_ = sortcost;
  walking_distance_ = walking_distance;
  tripid_ = tripid;
  blockid_ = blockid;
}

// Get the predecessor edge label index.
uint32_t EdgeLabel::predecessor() const {
  return predecessor_;
}

// Get the GraphId of this edge.
const baldr::GraphId& EdgeLabel::edgeid() const {
  return edgeid_;
}

// Get the end node of the predecessor edge.
const baldr::GraphId& EdgeLabel::endnode() const {
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
  return static_cast<Use>(attributes_.use);
}

// Get the opposing local index. This is the index of the incoming edge
// (on the local hierarchy) at the end node of the predecessor directed
// edge. This is used for edge transition costs and Uturn detection.
uint32_t EdgeLabel::opp_local_idx() const {
  return attributes_.opp_local_idx;
}

// Get the restriction mask at the end node. Each bit set to 1 indicates a
// turn restriction onto the directed edge with matching local edge index.
uint32_t EdgeLabel::restrictions() const {
  return attributes_.restrictions;
}

// Get the transition up flag.
bool EdgeLabel::trans_up() const {
  return attributes_.trans_up;
}

// Get the transition down flag.
bool EdgeLabel::trans_down() const {
  return attributes_.trans_down;
}

// Get the shortcut flag.
bool EdgeLabel::shortcut() const {
  return attributes_.shortcut;
}

// Get the travel mode along this edge.
TravelMode EdgeLabel::mode() const {
  return static_cast<TravelMode>(attributes_.mode);
}

// Get the destination only flag.
bool EdgeLabel::destonly() const {
  return attributes_.dest_only;
}

// Has any transit been taken up to this point on the path.
bool EdgeLabel::has_transit() const {
  return attributes_.has_transit;
}

// Is this edge an origin edge?
bool EdgeLabel::origin() const {
  return attributes_.origin;
}

// Sets this edge as an origin.
void EdgeLabel::set_origin() {
  attributes_.origin = true;
}

// Get the current walking distance in meters.
uint32_t EdgeLabel::walking_distance() const {
  return walking_distance_;
}

// Get the transit trip Id.
uint32_t EdgeLabel::tripid() const {
  return tripid_;
}

// Get the prior transit stop Id.
uint32_t EdgeLabel::prior_stopid() const {
  return prior_stopid_;
}

// Return the transit block Id of the prior trip.
uint32_t EdgeLabel::blockid() const {
  return blockid_;
}

// Operator for sorting.
bool EdgeLabel::operator < (const EdgeLabel& other) const {
  return sortcost() < other.sortcost();
}

}
}
