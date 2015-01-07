#include "baldr/edgeinfo.h"

namespace valhalla {
namespace baldr {

EdgeInfo::EdgeInfo(char* ptr) {
  item_ = reinterpret_cast<PackedItem*>(ptr);
  ptr += sizeof(PackedItem);

  // Set street_name_offset_list_ pointer
  street_name_offset_list_ = reinterpret_cast<uint32_t*>(ptr);
  ptr += (name_count() * sizeof(uint32_t));

  // Set encoded_shape_ pointer
  encoded_shape_ = ptr;
  ptr += (encoded_shape_size() * sizeof(char));

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
      encoded_shape_(nullptr),
      exit_signs_(nullptr) {
}

const uint32_t EdgeInfo::name_count() const {
  return item_->fields.name_count;
}

const uint32_t EdgeInfo::encoded_shape_size() const {
  return item_->fields.encoded_shape_size ;
}

const uint32_t EdgeInfo::exit_sign_count() const {
  return item_->fields.exit_sign_count;
}

const uint32_t EdgeInfo::GetStreetNameOffset(uint8_t index) const {
  if(index < item_->fields.name_count)
    return street_name_offset_list_[index];
  else
    throw std::runtime_error("StreetNameOffset index was out of bounds");
}

const std::vector<PointLL>& EdgeInfo::shape() const {
  //if we haven't yet decoded the shape, do so
  if(encoded_shape_ != nullptr) {
    shape_ = midgard::decode<std::vector<PointLL> >(std::string(encoded_shape_, item_->fields.encoded_shape_size));
    encoded_shape_ = nullptr;
  }

  //hand it back
  return shape_;
}

}
}
