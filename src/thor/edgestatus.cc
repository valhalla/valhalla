#include "thor/edgestatus.h"

using namespace valhalla::baldr;

namespace valhalla{
namespace thor{

  EdgeStatus::EdgeStatus() { }

  void EdgeStatus::Init() {
    edgestatus_.clear();
  }

  void EdgeStatus::Set(const baldr::GraphId& edgeid, const EdgeLabel status) {
  //  std::pair<GraphId, EdgeLabel> p(edgeid, status);
  //  edgestatus_.insert(p);
    edgestatus_.emplace(edgeid, status);
  }

  EdgeLabel EdgeStatus::Get(const baldr::GraphId& edgeid) const {
    auto p = edgestatus_.find(edgeid);
    return (p == edgestatus_.end()) ? kUnreached : p->second;
  }
}
}
