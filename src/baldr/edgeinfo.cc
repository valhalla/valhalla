#include "baldr/edgeinfo.h"

using namespace valhalla::baldr;

namespace {

json::ArrayPtr names_json(const std::vector<std::string>& names) {
  auto a = json::array({});
  for(const auto& n : names)
    a->push_back(n);
  return a;
}

}

namespace valhalla {
namespace baldr {

EdgeInfo::EdgeInfo(char* ptr, const char* names_list,
                   const size_t names_list_length)
  : names_list_(names_list), names_list_length_(names_list_length) {

  wayid_ = *(reinterpret_cast<uint64_t*>(ptr));
  ptr += sizeof(uint64_t);

  item_ = reinterpret_cast<PackedItem*>(ptr);
  ptr += sizeof(PackedItem);

  // Set street_name_offset_list_ pointer
  street_name_offset_list_ = reinterpret_cast<uint32_t*>(ptr);
  ptr += (name_count() * sizeof(uint32_t));

  // Set encoded_shape_ pointer
  encoded_shape_ = ptr;
  ptr += (encoded_shape_size() * sizeof(char));
}

EdgeInfo::~EdgeInfo() {
  //nothing to delete these are all shallow pointers for the moment held
  //by another object
}

// Gets the OSM way Id.
uint64_t EdgeInfo::wayid() const {
  return wayid_;
}

// Get the number of names.
uint32_t EdgeInfo::name_count() const {
  return item_->name_count;
}

// Get the size of the encoded shape (number of bytes).
uint32_t EdgeInfo::encoded_shape_size() const {
  return item_->encoded_shape_size;
}

uint32_t EdgeInfo::GetStreetNameOffset(uint8_t index) const {
  if(index < item_->name_count)
    return street_name_offset_list_[index];
  else
    throw std::runtime_error("StreetNameOffset index was out of bounds");
}

// Get a list of names
std::vector<std::string> EdgeInfo::GetNames() const {
  // Get each name
  std::vector<std::string> names; names.reserve(name_count());
  for (uint32_t i = 0; i < name_count(); i++) {
    uint32_t offset = GetStreetNameOffset(i);

    if (offset < names_list_length_) {
      names.push_back(names_list_ + offset);
    } else {
      throw std::runtime_error("GetNames: offset exceeds size of text list");
    }
  }
  return names;
}

// Returns shape as a vector of PointLL
const std::vector<PointLL>& EdgeInfo::shape() const {
  //if we haven't yet decoded the shape, do so
  if(encoded_shape_ != nullptr) {
    shape_ = midgard::decode7<std::vector<PointLL> >(std::string(encoded_shape_,
                                  item_->encoded_shape_size));
    encoded_shape_ = nullptr;
  }

  //hand it back
  return shape_;
}

// Returns the encoded shape string
std::string EdgeInfo::encoded_shape() const {
  return encoded_shape_ == nullptr ? midgard::encode7(shape_) : std::string(encoded_shape_, item_->encoded_shape_size);
}

json::MapPtr EdgeInfo::json() const {
  auto encoded = midgard::encode(nullptr ? shape_ :
    midgard::decode7<std::vector<PointLL> >(std::string(encoded_shape_, item_->encoded_shape_size)));

  return json::map({
    {"way_id", static_cast<uint64_t>(wayid_)},
    {"names", names_json(GetNames())},
    {"shape", encoded},
  });
}

}
}
