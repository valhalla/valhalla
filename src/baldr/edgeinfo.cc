#include "baldr/edgeinfo.h"

#include "midgard/encoded.h"

using namespace valhalla::baldr;

namespace {

json::MapPtr bike_network_json(uint8_t mask) {
  return json::map({
      {"national", static_cast<bool>(mask & kNcn)},
      {"regional", static_cast<bool>(mask & kRcn)},
      {"local", static_cast<bool>(mask & kLcn)},
      {"mountain", static_cast<bool>(mask & kMcn)},
  });
}

json::ArrayPtr names_json(const std::vector<std::string>& names) {
  auto a = json::array({});
  for (const auto& n : names) {
    a->push_back(n);
  }
  return a;
}

} // namespace

namespace valhalla {
namespace baldr {

EdgeInfo::EdgeInfo(char* ptr, const char* names_list, const size_t names_list_length)
    : names_list_(names_list), names_list_length_(names_list_length) {

  w0_.value_ = *(reinterpret_cast<uint64_t*>(ptr));
  ptr += sizeof(uint64_t);

  item_ = reinterpret_cast<PackedItem*>(ptr);
  ptr += sizeof(PackedItem);

  // Set name info list pointer
  name_info_list_ = reinterpret_cast<NameInfo*>(ptr);
  ptr += (name_count() * sizeof(NameInfo));

  // Set encoded_shape_ pointer
  encoded_shape_ = ptr;
  ptr += (encoded_shape_size() * sizeof(char));
}

EdgeInfo::~EdgeInfo() {
  // nothing to delete these are all shallow pointers for the moment held
  // by another object
}

// Get the name info for the specified name index.
NameInfo EdgeInfo::GetNameInfo(uint8_t index) const {
  if (index < item_->name_count) {
    return name_info_list_[index];
  } else {
    throw std::runtime_error("StreetNameOffset index was out of bounds");
  }
}

// Get a list of names
std::vector<std::string> EdgeInfo::GetNames() const {
  // Get each name
  std::vector<std::string> names;
  names.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    // Skip any tagged names (FUTURE code may make use of them)
    if (ni->tagged_) {
      continue;
    }
    if (ni->name_offset_ < names_list_length_) {
      names.push_back(names_list_ + ni->name_offset_);
    } else {
      throw std::runtime_error("GetNames: offset exceeds size of text list");
    }
  }
  return names;
}

// Get a list of names
std::vector<std::pair<std::string, bool>> EdgeInfo::GetNamesAndTypes() const {
  // Get each name
  std::vector<std::pair<std::string, bool>> name_type_pairs;
  name_type_pairs.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    // Skip any tagged names (FUTURE code may make use of them)
    if (ni->tagged_) {
      continue;
    }
    if (ni->name_offset_ < names_list_length_) {
      name_type_pairs.push_back({names_list_ + ni->name_offset_, ni->is_route_num_});
    } else {
      throw std::runtime_error("GetNamesAndTypes: offset exceeds size of text list");
    }
  }
  return name_type_pairs;
}

// Get the types.  Are these names route numbers or not?
uint16_t EdgeInfo::GetTypes() const {
  // Get the types.
  uint16_t types = 0;
  for (uint32_t i = 0; i < name_count(); i++) {
    NameInfo info = GetNameInfo(i);
    types |= static_cast<uint64_t>(info.is_route_num_) << i;
  }
  return types;
}

// Returns shape as a vector of PointLL
const std::vector<midgard::PointLL>& EdgeInfo::shape() const {
  // if we haven't yet decoded the shape, do so
  if (encoded_shape_ != nullptr && shape_.empty()) {
    shape_ =
        midgard::decode7<std::vector<midgard::PointLL>>(encoded_shape_, item_->encoded_shape_size);
  }
  return shape_;
}

// Returns the encoded shape string
std::string EdgeInfo::encoded_shape() const {
  return encoded_shape_ == nullptr ? midgard::encode7(shape_)
                                   : std::string(encoded_shape_, item_->encoded_shape_size);
}

json::MapPtr EdgeInfo::json() const {
  return json::map({
      {"way_id", static_cast<uint64_t>(wayid())},
      {"mean_elevation", static_cast<uint64_t>(mean_elevation())},
      {"bike_network", bike_network_json(bike_network())},
      {"speed_limit", static_cast<uint64_t>(speed_limit())},
      {"names", names_json(GetNames())},
      {"shape", midgard::encode(shape())},
  });
}

} // namespace baldr
} // namespace valhalla
