#include "thor/edgelabel.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// Default constructor
EdgeLabel::EdgeLabel()
    : predecessor_(nullptr),
      edgeid_(GraphId()),
      truecost_(0.0f),
      sortcost_(0.0f) {
}

// Constructor with values.
EdgeLabel::EdgeLabel(const EdgeLabel* predecessor,
                     const GraphId& edgeid, const GraphId& endnode,
                     const float cost, const float sortcost)
    : predecessor_(predecessor),
      edgeid_(edgeid),
      endnode_(endnode),
      truecost_(cost),
      sortcost_(sortcost) {
}

// Destructor
EdgeLabel::~EdgeLabel() {
}

// Constructor with values.
void EdgeLabel::Update(const EdgeLabel* predecessor, const float cost,
                       const float sortcost) {
  predecessor_ = predecessor;
  truecost_ = cost;
  sortcost_ = sortcost;
}

// Get the predecessor edge label.
const EdgeLabel* EdgeLabel::predecessor() const {
  return predecessor_;
}

void EdgeLabel::SetPredecessor(const EdgeLabel* predecessor) {
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

const baldr::GraphId& EdgeLabel::endnode() const {
  return endnode_;
}

void EdgeLabel::SetEndNode(const baldr::GraphId& endnode) {
  endnode_ = endnode;
}

float EdgeLabel::sortcost() const {
  return sortcost_;
}

void EdgeLabel::SetSortCost(float sortcost) {
  sortcost_ = sortcost;
}

float EdgeLabel::truecost() const {
  return truecost_;
}

void EdgeLabel::SetTrueCost(float truecost) {
  truecost_ = truecost;
}

bool EdgeLabel::operator < (const EdgeLabel& other) const {
  return sortcost() < other.sortcost();
}

}
}
