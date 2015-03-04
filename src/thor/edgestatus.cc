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
                     const EdgeStatusType status) {
  edgestatus_[edgeid] = status;
}

// Get the edge status of a GraphId. If not found in the map the
// edge is considered unreached.
EdgeStatusType EdgeStatus::Get(const baldr::GraphId& edgeid) const {
  auto p = edgestatus_.find(edgeid);
  return (p == edgestatus_.end()) ? kUnreached : p->second;
}

}
}
