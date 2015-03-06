#include "thor/edgelabel.h"
#include "thor/adjacencylist.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// Default constructor
EdgeLabel::EdgeLabel()
    : predecessor_(kInvalidLabel),
      edgeid_(GraphId()),
      truecost_(0.0f),
      sortcost_(0.0f),
      distance_(0.0f),
      attributes_{} {
}

// Constructor with values.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
                     const DirectedEdge* edge, const float cost,
                     const float sortcost, const float dist,
                     const uint32_t restrictions)
    : predecessor_(predecessor),
      edgeid_(edgeid),
      truecost_(cost),
      sortcost_(sortcost),
      distance_(dist) {
  endnode_                  = edge->endnode();
  attributes_.uturn_index   = edge->opp_index();
  attributes_.trans_up      = edge->trans_up();
  attributes_.trans_down    = edge->trans_down();
  attributes_.restrictions  = restrictions;
  attributes_.opp_local_idx = edge->opp_local_idx();
}

// Destructor
EdgeLabel::~EdgeLabel() {
}

// Update predecessor and cost values in the label.
void EdgeLabel::Update(const uint32_t predecessor, const float cost,
                       const float sortcost) {
  predecessor_ = predecessor;
  truecost_ = cost;
  sortcost_ = sortcost;
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

// Get the sort cost.
float EdgeLabel::sortcost() const {
  return sortcost_;
}

// Set the sort cost
void EdgeLabel::SetSortCost(float sortcost) {
sortcost_ = sortcost;
}

// Get the true cost.
float EdgeLabel::truecost() const {
  return truecost_;
}

// Get the distance to the destination.
float EdgeLabel::distance() const {
  return distance_;
}

// Get the Uturn index
uint32_t EdgeLabel::uturn_index() const {
  return attributes_.uturn_index;
}

// Get the transition up flag.
bool EdgeLabel::trans_up() const {
  return attributes_.trans_up;
}

// Get the transition down flag.
bool EdgeLabel::trans_down() const {
  return attributes_.trans_down;
}

// Get the restriction mask at the end node. Each bit set to 1 indicates a
// turn restriction onto the directed edge with matching local edge index.
uint32_t EdgeLabel::restrictions() const {
  return attributes_.restrictions;
}

// Get the opposing local index. This is the index of the incoming edge
// (on the local hierarchy) at the end node of the predecessor directed
// edge. This is used for edge transition costs.
uint32_t EdgeLabel::opp_local_idx() const {
  return attributes_.opp_local_idx;
}

// Operator for sorting.
bool EdgeLabel::operator < (const EdgeLabel& other) const {
  return sortcost() < other.sortcost();
}

}
}
