#include "baldr/edgeinfo.h"
#include "baldr/graphconstants.h"

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

std::vector<std::string> split(const std::string& source, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(source);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
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
  // Get each name
  std::vector<std::string> names;
  names.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    if (ni->tagged_)
      continue;

    if (ni->name_offset_ < names_list_length_) {
      names.push_back(names_list_ + ni->name_offset_);
    } else {
      throw std::runtime_error("GetNames: offset exceeds size of text list");
    }
  }
  return names;
}

// Get a list of tagged names
std::vector<std::string> EdgeInfo::GetTaggedNames(bool only_pronunciations) const {
  // Get each name
  std::vector<std::string> names;
  names.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    if (!ni->tagged_)
      continue;

    if (ni->name_offset_ < names_list_length_) {
      std::string name = names_list_ + ni->name_offset_;
      if (name.size() > 1) {
        uint8_t num = 0;
        try {
          num = std::stoi(name.substr(0, 1));
          if (static_cast<baldr::TaggedName>(num) == baldr::TaggedName::kPronunciation) {
            if (!only_pronunciations)
              continue;

            size_t location = name.length() + 1; // start from here
            name = name.substr(1);               // remove the tagged name type...actual data remains

            auto verbal_tokens = split(name, '#');
            // 0 \0 1 \0 ˌwɛst ˈhaʊstən stɹiːt
            // key  type value
            uint8_t index = 0;
            std::string key, type;
            for (const auto& v : verbal_tokens) {
              if (index == 0) { // key
                key = v;
                index++;
              } else if (index == 1) { // type
                type = v;
                index++;
              } else { // value
                names.push_back(key + '#' + type + '#' + v);
                index = 0;
              }
            }
          } else if (!only_pronunciations) {
            names.push_back(names_list_ + ni->name_offset_);
          }
        } catch (const std::invalid_argument& arg) {
          LOG_DEBUG("invalid_argument thrown for name: " + name);
        }
      } else {
        throw std::runtime_error("GetTaggedNames: Tagged name with no text");
      }
    } else {
      throw std::runtime_error("GetTaggedNames: offset exceeds size of text list");
    }
  }
  return names;
}

// Get a list of names
std::vector<std::pair<std::string, bool>>
EdgeInfo::GetNamesAndTypes(std::vector<uint8_t>& types, bool include_tagged_names) const {
  // Get each name
  std::vector<std::pair<std::string, bool>> name_type_pairs;
  name_type_pairs.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    // Skip any tagged names (FUTURE code may make use of them)
    if (ni->tagged_ && !include_tagged_names) {
      continue;
    }
    if (ni->tagged_) {
      if (ni->name_offset_ < names_list_length_) {
        std::string name = names_list_ + ni->name_offset_;
        if (name.size() > 1) {
          uint8_t num = 0;
          try {
            num = std::stoi(name.substr(0, 1));
            if (static_cast<baldr::TaggedName>(num) != baldr::TaggedName::kPronunciation) {
              name_type_pairs.push_back({name.substr(1), false});
              types.push_back(num);
            }

          } catch (const std::invalid_argument& arg) {
            LOG_DEBUG("invalid_argument thrown for name: " + name);
          }
        }
      } else
        throw std::runtime_error("GetNamesAndTypes: offset exceeds size of text list");
    } else if (ni->name_offset_ < names_list_length_) {
      name_type_pairs.push_back({names_list_ + ni->name_offset_, ni->is_route_num_});
      types.push_back(0);
    } else {
      throw std::runtime_error("GetNamesAndTypes: offset exceeds size of text list");
    }
  }
  return name_type_pairs;
}

// Get a list of tagged names
std::vector<std::pair<std::string, uint8_t>> EdgeInfo::GetTaggedNamesAndTypes() const {
  // Get each name
  std::vector<std::pair<std::string, uint8_t>> name_type_pairs;
  name_type_pairs.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    // Skip any non tagged names
    if (ni->tagged_) {
      if (ni->name_offset_ < names_list_length_) {
        std::string name = names_list_ + ni->name_offset_;
        if (name.size() > 1) {
          uint8_t num = 0;
          try {
            num = std::stoi(name.substr(0, 1));
            if (static_cast<baldr::TaggedName>(num) != baldr::TaggedName::kPronunciation)
              name_type_pairs.push_back({name.substr(1), num});

          } catch (const std::invalid_argument& arg) {
            LOG_DEBUG("invalid_argument thrown for name: " + name);
          }
        }
      } else {
        throw std::runtime_error("GetTaggedNamesAndTypes: offset exceeds size of text list");
      }
    }
  }
  return name_type_pairs;
}

std::unordered_map<uint8_t, std::pair<uint8_t, std::string>> EdgeInfo::GetPronunciationsMap() const {
  std::unordered_map<uint8_t, std::pair<uint8_t, std::string>> index_pronunciation_map;
  index_pronunciation_map.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    if (!ni->tagged_)
      continue;

    if (ni->name_offset_ < names_list_length_) {
      std::string name = names_list_ + ni->name_offset_;
      if (name.length() > 1) {
        uint8_t num = 0;
        try {
          num = std::stoi(name.substr(0, 1));
          if (static_cast<baldr::TaggedName>(num) == baldr::TaggedName::kPronunciation) {
            size_t location = name.length() + 1; // start from here
            name = name.substr(1);               // remove the tagged name type...actual data remains

            auto pronunciation_tokens = split(name, '#');
            // index # alphabet # pronunciation
            // 0     # 1        # ˌwɛst ˈhaʊstən stɹiːt
            uint8_t token_index = 0, index, pronunciation_alphabet;
            for (const auto& token : pronunciation_tokens) {
              if (token_index == 0) { // index
                index = std::stoi(token);
                token_index++;
              } else if (token_index == 1) { // pronunciation_alphabet
                pronunciation_alphabet = std::stoi(token);
                token_index++;
              } else { // value
                std::unordered_map<uint8_t, std::pair<uint8_t, std::string>>::iterator iter =
                    index_pronunciation_map.find(index);

                if (iter == index_pronunciation_map.end())
                  index_pronunciation_map.emplace(
                      std::make_pair(index, std::make_pair(pronunciation_alphabet, token)));
                else {
                  if (pronunciation_alphabet > (iter->second).first) {
                    iter->second = std::make_pair(pronunciation_alphabet, token);
                  }
                }
                token_index = 0;
              }
            }
          }
        } catch (const std::invalid_argument& arg) {
          LOG_DEBUG("invalid_argument thrown for name: " + name);
        }
      } else {
        throw std::runtime_error("GetPronunciationsMap: Tagged name with no text");
      }
    } else {
      throw std::runtime_error("GetPronunciationsMap: offset exceeds size of text list");
    }
  }

  return index_pronunciation_map;
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
