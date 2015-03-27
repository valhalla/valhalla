#include "baldr/admininfo.h"

namespace valhalla {
namespace baldr {

AdminInfo::AdminInfo(char* ptr, const char* names_list, const size_t names_list_length)
  : names_list_(names_list), names_list_length_(names_list_length) {

  iso_code_index_ = 0;

  item_ = reinterpret_cast<PackedItem*>(ptr);
  ptr += sizeof(PackedItem);
  AdminInfo
  // Set name_offset_list_ pointer
  name_offset_list_ = reinterpret_cast<uint32_t*>(ptr);
  ptr += (name_count() * sizeof(uint32_t));
}

AdminInfo::~AdminInfo() {
  //nothing to delete these are all shallow pointers for the moment held
  //by another object
}

const uint32_t AdminInfo::name_count() const {
  return item_->fields.name_count;
}

// Returns the iso code
const uint32_t AdminInfo::iso_code_index() const {
  return iso_code_index_;
}

// Set the iso code index.
void AdminInfo::set_iso_code_index(const uint32_t iso_code_index) {
  iso_code_index_ = iso_code_index;
}

// Returns the parent admin id
const uint32_t AdminInfo::parent_admin_id() const {
  return item_->fields.parent_admin_id;
}

// Returns the admin id
const uint32_t AdminInfo::admin_id() const {
  return item_->fields.admin_id;
}

const char* AdminInfo::StartDST() const {
  return start_dst_;
}

void AdminInfo::SetStartDST(const std::string& start_dst) {
  //YYYYMMDD
  std::size_t length = start_dst.copy(start_dst_,kDstSize-1);
  start_dst_[length]='\0';
}

void AdminInfo::SetEndDST(const std::string& end_dst) {
  //YYYYMMDD
  std::size_t length = end_dst.copy(end_dst_,kDstSize-1);
  end_dst_[length]='\0';
}

const char* AdminInfo::EndDST() const {
  return start_dst_;
}

const uint32_t AdminInfo::GetNameOffset(uint8_t index) const {
  if(index < item_->fields.name_count)
    return name_offset_list_[index];
  else
    throw std::runtime_error("NameOffset index was out of bounds");
}

const std::vector<std::string> AdminInfo::GetNames() const {
  // Get each name
  std::vector<std::string> names;
  for (uint32_t i = 0; i < name_count(); i++) {
    uint32_t offset = GetNameOffset(i);
    if (offset < names_list_length_) {
      names.push_back(names_list_ + offset);
    } else
      throw std::runtime_error("Admininfo:GetNames: offset exceeds size of text list");
  }
  return names;
}

}
}
