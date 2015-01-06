#include "baldr/edgeinfo.h"

namespace valhalla {
namespace baldr {

EdgeInfo::EdgeInfo(char* ptr) {
  item_ = reinterpret_cast<PackedItem*>(ptr);
  ptr += sizeof(PackedItem);

  // Set street_name_offset_list_ pointer
  street_name_offset_list_ = reinterpret_cast<size_t*>(ptr);
  ptr += (name_count() * sizeof(size_t));

  // Set shape_ pointer
  shape_ = reinterpret_cast<PointLL*>(ptr);
  ptr += (shape_count() * sizeof(PointLL));

  // Set exit_signs_ pointer
  exit_signs_ = reinterpret_cast<ExitSign*>(ptr);
}

EdgeInfo::~EdgeInfo() {
  //nothing to delete these are all shallow pointers for the moment held
  //by another object
}

EdgeInfo::EdgeInfo()
    : item_(nullptr),
      street_name_offset_list_(nullptr),
      shape_(nullptr),
      exit_signs_(nullptr) {
}

const uint32_t EdgeInfo::name_count() const {
  return item_->fields.name_count;
}

const uint32_t EdgeInfo::shape_count() const {
  return item_->fields.shape_count;
}

const uint32_t EdgeInfo::exit_sign_count() const {
  return item_->fields.exit_sign_count;
}

const size_t EdgeInfo::GetStreetNameOffset(uint8_t index) const {
  return street_name_offset_list_[index];
}

const PointLL EdgeInfo::GetShapePoint(uint16_t index) const {
  return shape_[index];
}

void EdgeInfo::ToOstream(std::ostream& out) const {
  out << "  name_count=" << name_count() << std::endl;
  for (uint32_t x = 0, n = name_count(); x < n; ++x) {
    out << "   street name offset[" << x << "]=" << GetStreetNameOffset(x)
        << std::endl;
  }
  out << "shape_count=" << shape_count() << std::endl;
  for (int32_t x = 0, n = shape_count(); x < n; ++x) {
    PointLL ll = GetShapePoint(x);
    out << "   shape[" << x << "]=" << ll.lat() << "," << ll.lng() << std::endl;
  }
  out << "exit_sign_count=" << exit_sign_count() << std::endl;
}

}
}
