#include "baldr/edgeinfo.h"

namespace valhalla {
namespace baldr {

EdgeInfo::EdgeInfo() {
}

const GraphId& EdgeInfo::nodea() const {
  return nodea_;
}

const GraphId& EdgeInfo::nodeb() const {
  return nodeb_;
}

const std::vector<PointLL>& EdgeInfo::shape() const {
  return shape_;
}

const std::vector<std::string>& EdgeInfo::nameindexes() const {
  return nameindexes_;
}

bool EdgeInfo::operator ==(const EdgeInfo& rhs) const {
  return ((nodea_ == rhs.nodea_ && nodeb_ == rhs.nodeb_)
      || (nodea_ == rhs.nodeb_ && nodeb_ == rhs.nodea_));
}

}
}
