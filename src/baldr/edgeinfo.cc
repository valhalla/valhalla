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

const uint32_t EdgeInfo::street_name_offset_list_offset() const {
  return item_.fields.street_name_offset_list_offset;
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

const size_t EdgeInfo::GetStreetNameOffset(uint8_t index) const {
  return street_name_offset_list_[index];
}

const PointLL EdgeInfo::GetShapePoint(uint8_t index) const {
  return shape_[index];
}

bool EdgeInfo::operator ==(const EdgeInfo& rhs) const {
  return ((nodea_ == rhs.nodea_ && nodeb_ == rhs.nodeb_)
      || (nodea_ == rhs.nodeb_ && nodeb_ == rhs.nodea_));
}

void EdgeInfo::ToOstream(std::ostream& out) const {
  out << "nodea_.tileid=" << nodea_.tileid() << "  nodea_.level="
      << nodea_.level() << "  nodea_.id=" << nodea_.id() << std::endl;
  out << "nodeb_.tileid=" << nodeb_.tileid() << "  nodeb_.level="
      << nodeb_.level() << "  nodeb_.id=" << nodeb_.id() << std::endl;
  out << "street_name_offset_list_offset=" << street_name_offset_list_offset()
      << "  name_count=" << name_count() << std::endl;
  for (uint32_t x = 0, n = name_count(); x < n; ++x) {
    out << "name[" << x << "]=" << GetStreetNameOffset(x) << std::endl;
  }
  out << "shape_count=" << shape_count() << std::endl;
  out << "GetShapeOffset=" << GetShapeOffset() << std::endl;
  for (int32_t x = 0, n = shape_count(); x < n; ++x) {
    PointLL ll = GetShapePoint(x);
    out << "shape[" << x << "]=" << ll.lat() << "," << ll.lng() << std::endl;
  }
  out << "exit_sign_count=" << exit_sign_count() << std::endl;
  out << "GetExitSignsOffset=" << GetExitSignsOffset() << std::endl;

}

void EdgeInfo::SetPointers() {
  char* ptr = reinterpret_cast<char*>(this);

  // Set street_name_offset_list_ pointer
  ptr += street_name_offset_list_offset();
  street_name_offset_list_ = reinterpret_cast<size_t*>(ptr);
  ptr += (name_count() * sizeof(size_t));

  // Set shape_ pointer
  shape_ = reinterpret_cast<PointLL*>(ptr);
  ptr += (shape_count() * sizeof(PointLL));

  // Set exit_signs_ pointer
  exit_signs_ = reinterpret_cast<ExitSign*>(ptr);
}

const uint32_t EdgeInfo::GetShapeOffset() const {
  return (street_name_offset_list_offset() + name_count() * sizeof(size_t));
}

const uint32_t EdgeInfo::GetExitSignsOffset() const {
  return (GetShapeOffset() + shape_count() * sizeof(PointLL));
}

}
}
