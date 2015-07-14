#include "thor/edgestatus.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// Constructor
EdgeStatus::EdgeStatus() {
}

// Clear current edge status (all become unreached)
void EdgeStatus::Init() {
  edgestatus_.clear();
}

// Set the edge status of a GraphId
void EdgeStatus::Set(const baldr::GraphId& edgeid,
                     const EdgeSet set, const uint32_t index) {
  edgestatus_[edgeid] = { set, index };
}

// Update the edge status of a GraphId
void EdgeStatus::Update(const baldr::GraphId& edgeid, const EdgeSet set) {
  edgestatus_[edgeid].status.set = set;
}

// Get the edge status of a GraphId. If not found in the map the
// edge is considered unreached.
EdgeStatusInfo EdgeStatus::Get(const baldr::GraphId& edgeid) const {
  auto p = edgestatus_.find(edgeid);
  return (p == edgestatus_.end()) ? EdgeStatusInfo() : p->second;
}

}
}
