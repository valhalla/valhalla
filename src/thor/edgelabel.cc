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
      uturn_index_(0) {
}

// Constructor with values.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
                     const GraphId& endnode, const float cost,
                     const float sortcost, const float dist,
                     const uint32_t uturn)
    : predecessor_(predecessor),
      edgeid_(edgeid),
      endnode_(endnode),
      truecost_(cost),
      sortcost_(sortcost),
      distance_(dist),
      uturn_index_(uturn) {
}

// Destructor
EdgeLabel::~EdgeLabel() {
}

// Set edge label values.
void EdgeLabel::Set(const uint32_t predecessor, const GraphId& edgeid,
                     const GraphId& endnode, const float cost,
                     const float sortcost, const float dist,
                     const uint32_t uturn) {
  predecessor_ = predecessor;
  edgeid_   = edgeid;
  endnode_  = endnode;
  truecost_ = cost;
  sortcost_ = sortcost;
  distance_ = dist;
  uturn_index_ = uturn;
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

// Set the predecessor edge label index.
void EdgeLabel::SetPredecessor(const uint32_t predecessor) {
  predecessor_ = predecessor;
}

// Get the GraphId of this edge.
const baldr::GraphId& EdgeLabel::edgeid() const {
  return edgeid_;
}

// Set the GraphId of this edge
void EdgeLabel::SetEdgeId(const baldr::GraphId& edgeid) {
  edgeid_ = edgeid;
}

// Get the end node of the predecessor edge.
const baldr::GraphId& EdgeLabel::endnode() const {
  return endnode_;
}

// Set the end node of the predecessor edge.
void EdgeLabel::SetEndNode(const baldr::GraphId& endnode) {
  endnode_ = endnode;
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

// Set the true cost.
void EdgeLabel::SetTrueCost(float truecost) {
  truecost_ = truecost;
}

// Get the distance to the destination.
float EdgeLabel::distance() const {
  return distance_;
}

// Set the distance to the destination.
void EdgeLabel::SetDistance(const float d) {
  distance_ = d;
}

uint32_t EdgeLabel::uturn_index() const {
  return uturn_index_;
}

void EdgeLabel::SetUturnIndex(const uint32_t idx) {
  uturn_index_ = idx;
}

// Operator for sorting.
bool EdgeLabel::operator < (const EdgeLabel& other) const {
  return sortcost() < other.sortcost();
}

}
}
