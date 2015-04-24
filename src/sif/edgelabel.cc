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
      attributes_{} {
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
      distance_(dist) {
  attributes_.opp_local_idx = opp_local_idx;
  attributes_.restrictions  = restrictions;
  attributes_.trans_up      = edge->trans_up();
  attributes_.trans_down    = edge->trans_down();
  attributes_.shortcut      = edge->shortcut();
  attributes_.mode          = static_cast<uint32_t>(mode);
}

// Destructor
EdgeLabel::~EdgeLabel() {
}

// Update predecessor and cost values in the label.
void EdgeLabel::Update(const uint32_t predecessor, const Cost& cost,
                       const float sortcost, const TravelMode mode) {
  predecessor_ = predecessor;
  cost_ = cost;
  sortcost_ = sortcost;
  attributes_.mode = static_cast<uint32_t>(mode);
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

// Operator for sorting.
bool EdgeLabel::operator < (const EdgeLabel& other) const {
  return sortcost() < other.sortcost();
}

}
}
