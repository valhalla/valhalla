#include "baldr/edgeinfo.h"
#include "baldr/graphconstants.h"

#include "midgard/encoded.h"

using namespace valhalla::baldr;

namespace {

// should return true for any tags which we should consider "named"
bool IsNameTag(char ch) {
  static const std::unordered_set<TaggedValue> kNameTags = {TaggedValue::kBridge,
                                                            TaggedValue::kTunnel};
  return kNameTags.count(static_cast<TaggedValue>(static_cast<uint8_t>(ch))) > 0;
}

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

  ei_ = *reinterpret_cast<EdgeInfoInner*>(ptr);
  ptr += sizeof(EdgeInfoInner);

  // Set name info list pointer
  name_info_list_ = reinterpret_cast<NameInfo*>(ptr);
  ptr += (name_count() * sizeof(NameInfo));

  // Set encoded_shape_ pointer
  encoded_shape_ = ptr;
  ptr += (encoded_shape_size() * sizeof(char));

  // Optional second half of 64bit way id
  extended_wayid2_ = extended_wayid3_ = 0;
  if (ei_.extended_wayid_size_ > 0) {
    extended_wayid2_ = static_cast<uint8_t>(*ptr);
    ptr += sizeof(uint8_t);
  }
  if (ei_.extended_wayid_size_ > 1) {
    extended_wayid3_ = static_cast<uint8_t>(*ptr);
    ptr += sizeof(uint8_t);
  }
}

EdgeInfo::~EdgeInfo() {
  // nothing to delete these are all shallow pointers for the moment held
  // by another object
}

// Get the name info for the specified name index.
NameInfo EdgeInfo::GetNameInfo(uint8_t index) const {
  if (index < ei_.name_count_) {
    return name_info_list_[index];
  } else {
    throw std::runtime_error("StreetNameOffset index was out of bounds");
  }
}

// Get a list of names
std::vector<std::string> EdgeInfo::GetNames() const {
  return GetTaggedValuesOrNames(false);
}

std::vector<std::string> EdgeInfo::GetTaggedValues() const {
  return GetTaggedValuesOrNames(true);
}

std::vector<std::string> EdgeInfo::GetTaggedValuesOrNames(bool only_tagged_values) const {
  // Get each name
  std::vector<std::string> names;
  names.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    if ((only_tagged_values && !ni->tagged_) || (!only_tagged_values && ni->tagged_)) {
      continue;
    }
    if (ni->name_offset_ < names_list_length_) {
      names.emplace_back(names_list_ + ni->name_offset_);
    } else {
      throw std::runtime_error("GetNames: offset exceeds size of text list");
    }
  }
  return names;
}

// Get a list of names
std::vector<std::pair<std::string, bool>>
EdgeInfo::GetNamesAndTypes(bool include_tagged_values) const {
  // Get each name
  std::vector<std::pair<std::string, bool>> name_type_pairs;
  name_type_pairs.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    // Skip any tagged names (FUTURE code may make use of them)
    if (ni->tagged_ && !include_tagged_values) {
      continue;
    }
    if (ni->tagged_) {
      if (ni->name_offset_ < names_list_length_) {
        std::string name = names_list_ + ni->name_offset_;
        if (name.size() > 1 && IsNameTag(name[0])) {
          try {
            name_type_pairs.push_back({name.substr(1), false});
          } catch (const std::invalid_argument& arg) {
            LOG_DEBUG("invalid_argument thrown for name: " + name);
          }
        }
      } else
        throw std::runtime_error("GetNamesAndTypes: offset exceeds size of text list");
    } else if (ni->name_offset_ < names_list_length_) {
      name_type_pairs.push_back({names_list_ + ni->name_offset_, ni->is_route_num_});
    } else {
      throw std::runtime_error("GetNamesAndTypes: offset exceeds size of text list");
    }
  }
  return name_type_pairs;
}

// Get a list of tagged names
const std::multimap<TaggedValue, std::string>& EdgeInfo::GetTags() const {
  // we could check `tag_cache_.empty()` here, but many edges contain no tags
  // and it would mean we traverse all names on each `GetTags` call
  // for such edges
  if (!tag_cache_ready_) {
    // Get each name
    const NameInfo* ni = name_info_list_;
    for (uint32_t i = 0; i < name_count(); i++, ni++) {
      // Skip any non tagged names
      if (ni->tagged_) {
        if (ni->name_offset_ < names_list_length_) {
          std::string name = names_list_ + ni->name_offset_;
          if (name.size() > 1) {
            uint8_t num = 0;
            try {
              num = static_cast<uint8_t>(name.at(0));
              tag_cache_.emplace(static_cast<TaggedValue>(num), name.substr(1));
            } catch (const std::logic_error& arg) {
              LOG_DEBUG("logic_error thrown for name: " + name);
            }
          }
        } else {
          throw std::runtime_error("GetTags: offset exceeds size of text list");
        }
      }
    }

    tag_cache_ready_ = true;
  }

  return tag_cache_;
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
// TODO: use shared ptr here so that we dont have to worry about lifetime
const std::vector<midgard::PointLL>& EdgeInfo::shape() const {
  // if we haven't yet decoded the shape, do so
  if (encoded_shape_ != nullptr && shape_.empty()) {
    shape_ = midgard::decode7<std::vector<midgard::PointLL>>(encoded_shape_, ei_.encoded_shape_size_);
  }
  return shape_;
}

// Returns the encoded shape string
std::string EdgeInfo::encoded_shape() const {
  return encoded_shape_ == nullptr ? midgard::encode7(shape_)
                                   : std::string(encoded_shape_, ei_.encoded_shape_size_);
}

int8_t EdgeInfo::layer() const {
  const auto& tags = GetTags();
  auto itr = tags.find(TaggedValue::kLayer);
  if (itr == tags.end()) {
    return 0;
  }
  const auto& value = itr->second;
  if (value.size() != 1) {
    throw std::runtime_error("layer must contain 1-byte value");
  }
  return static_cast<int8_t>(value.front());
}

json::MapPtr EdgeInfo::json() const {
  json::MapPtr edge_info = json::map({
      {"way_id", static_cast<uint64_t>(wayid())},
      {"mean_elevation", static_cast<uint64_t>(mean_elevation())},
      {"bike_network", bike_network_json(bike_network())},
      {"names", names_json(GetNames())},
      {"shape", midgard::encode(shape())},
  });

  if (speed_limit() == kUnlimitedSpeedLimit) {
    edge_info->emplace("speed_limit", std::string("unlimited"));
  } else {
    edge_info->emplace("speed_limit", static_cast<uint64_t>(speed_limit()));
  }

  return edge_info;
}

} // namespace baldr
} // namespace valhalla
