#include "thor/edgestatus.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

EdgeStatus::EdgeStatus() {
}

void EdgeStatus::Init() {
  edgestatus_.clear();
}

void EdgeStatus::Set(const baldr::GraphId& edgeid,
                     const EdgeStatusType status) {
  edgestatus_.emplace(edgeid, status);
}

EdgeStatusType EdgeStatus::Get(const baldr::GraphId& edgeid) const {
  auto p = edgestatus_.find(edgeid);
  return (p == edgestatus_.end()) ? kUnreached : p->second;
}
}
}
