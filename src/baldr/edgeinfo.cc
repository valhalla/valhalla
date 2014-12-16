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

const std::vector<uint32_t>& EdgeInfo::nameindexes() const {
  return nameindexes_;
}

bool EdgeInfo::operator ==(const EdgeInfo& rhs) const {
  return ((nodea_ == rhs.nodea_ && nodeb_ == rhs.nodeb_)
      || (nodea_ == rhs.nodeb_ && nodeb_ == rhs.nodea_));
}

std::size_t EdgeInfo::SizeOf() const {
  std::size_t size = 0;
  size += sizeof(GraphId);                           // nodea_
  size += sizeof(GraphId);                           // nodeb_
  size += sizeof(std::size_t);                       // shape_ size
  size += (shape_.size() * sizeof(PointLL));         // shape_
  size += sizeof(std::size_t);                       // nameindexes_ size
  size += (nameindexes_.size() * sizeof(uint32_t));  // nameindexes_

  return size;
}

}
}
