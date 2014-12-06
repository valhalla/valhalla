#include "thor/edgelabel.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

EdgeLabel::EdgeLabel() {

}

const baldr::GraphId& EdgeLabel::edgeid() const {
  return edgeid_;
}

void EdgeLabel::SetEdgeId(const baldr::GraphId& edgeid) {
  edgeid_ = edgeid;
}

const baldr::GraphId& EdgeLabel::endnode() const {
  return endnode_;
}

void EdgeLabel::SetEndNode(const baldr::GraphId& endnode) {
  endnode_ = endnode;
}

/**
const EdgeLabel* EdgeLabel::predecessor() const {
  return predecessor_;
}

void EdgeLabel::SetPredecessor(const EdgeLabel*& predecessor) {
  predecessor_ = predecessor;
}
**/
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

}
}
