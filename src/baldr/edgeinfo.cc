#include "baldr/edgeinfo.h"

namespace valhalla {
namespace baldr {

EdgeInfo::EdgeInfo() {
  item_.value = 0;
}

const GraphId& EdgeInfo::nodea() const {
  return nodea_;
}

const GraphId& EdgeInfo::nodeb() const {
  return nodeb_;
}

const uint32_t EdgeInfo::name_indexes_offset() const {
  return item_.fields.name_indexes_offset;
}

const uint32_t EdgeInfo::name_count() const {
  return item_.fields.name_count;
}

const uint32_t EdgeInfo::shape_count() const {
  return item_.fields.shape_count;
}

const uint32_t EdgeInfo::exit_sign_count() const {
  return item_.fields.exit_sign_count;
}

// TODO - implement later
//const uint32_t EdgeInfo::GetNameIndex(uint8_t index) const {
//  return *(name_indexes_ + index);
//}
//
//const PointLL* EdgeInfo::GetShapePoint(uint8_t index) const {
//  char* byte_ptr = static_cast<char*>(this);
//  byte_ptr += GetShapeOffset();
//  return (static_cast<PointLL*>(byte_ptr) + index);
//}

bool EdgeInfo::operator ==(const EdgeInfo& rhs) const {
  return ((nodea_ == rhs.nodea_ && nodeb_ == rhs.nodeb_)
      || (nodea_ == rhs.nodeb_ && nodeb_ == rhs.nodea_));
}

const uint32_t EdgeInfo::GetShapeOffset() const {
  return (name_indexes_offset() + name_count() * sizeof(uint32_t));
}

const uint32_t EdgeInfo::GetExitSignsOffset() const {
  return (GetShapeOffset() + shape_count() * sizeof(PointLL));
}

}
}
