#include "baldr/edgeinfo.h"
#include "baldr/graphconstants.h"
#include "midgard/elevation_encoding.h"

using namespace valhalla::baldr;

namespace {

// should return true if not equal to TaggedValue::kLinguistic
bool IsNonLiguisticTagValue(char ch) {
  return static_cast<TaggedValue>(ch) != TaggedValue::kLinguistic;
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

/**
 * Parse a 7-bit encoded varint.
 *
 * @param encoded the encoded c string
 * @param i       value to store the varint size in
 *
 */
int32_t parse_varint(const char*& encoded) {
  int32_t byte = 0, shift = 0, result = 0;

  while (byte & 0x80 || shift == 0) {
    byte = int32_t(*encoded);
    result |= (byte & 0x7f) << shift;
    shift += 7;
    ++encoded;
  }

  return (result & 1 ? ~result : result) >> 1;
}

// per tag parser. each returned string includes the leading TaggedValue.
std::vector<std::string> parse_tagged_value(const char* ptr) {
  switch (static_cast<TaggedValue>(ptr[0])) {
    case TaggedValue::kLayer:
    case TaggedValue::kBssInfo:
    case TaggedValue::kLevel:
    case TaggedValue::kLevelRef:
    case TaggedValue::kTunnel:
    case TaggedValue::kBridge:
      return {std::string(ptr)};
    case TaggedValue::kLandmark: {
      std::string landmark_name = ptr + 10;
      size_t landmark_size = landmark_name.size() + 10;
      return {std::string(ptr, landmark_size)};
    }
    case TaggedValue::kLevels: {
      auto start = ptr + 1;
      int size = static_cast<int>(parse_varint(start));
      return {std::string(ptr, (start + size) - ptr)};
    }
    case TaggedValue::kConditionalSpeedLimits: {
      return {std::string(ptr, 1 + sizeof(ConditionalSpeedLimit))};
    }
    case TaggedValue::kLinguistic:
    default:
      return {};
  }
}

} // namespace

namespace valhalla {
namespace baldr {

std::pair<std::vector<std::pair<float, float>>, uint32_t> decode_levels(const std::string& encoded) {
  uint32_t precision = 0;
  std::vector<std::pair<float, float>> decoded;
  decoded.reserve(4);

  auto ptr = encoded.data();

  // first varint is the size
  auto size = parse_varint(ptr);
  // we keep track of where the string ends
  auto end = ptr + size;
  // second varint is the precision
  if (int32_t prec_power = parse_varint(ptr) > 0)
    precision = pow(10, prec_power);

  // keep track of whether we're looking at the first or second value
  // of a contiguous block
  bool prev = false;

  while (ptr != end) {
    int32_t val = parse_varint(ptr);
    if (val == kLevelRangeSeparator) {
      prev = false;
      continue;
    }
    float f = precision == 0 ? static_cast<float>(val)
                             : static_cast<float>(val) / static_cast<float>(precision);
    if (!prev) {
      // first value
      // (temporarily) set to the same value to indicate a single number
      decoded.emplace_back(std::pair<float, float>{f, f});
      prev = true;
    } else {
      // we found a second value
      decoded.back().second = f;
    }
  }

  return {decoded, static_cast<uint32_t>(precision)};
}

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

  // Set encoded elevation pointer
  encoded_elevation_ = reinterpret_cast<int8_t*>(ptr);
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

// Get a list of names tagged and/or untagged with tag status
std::vector<std::pair<std::string, bool>> EdgeInfo::GetNames(bool include_tagged_values) const {
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
        const char* name = names_list_ + ni->name_offset_;
        if (IsNonLiguisticTagValue(name[0])) {
          name_type_pairs.push_back({std::string(name + 1), false});
        }
      } else
        throw std::runtime_error("GetNames: offset exceeds size of text list");
    } else if (ni->name_offset_ < names_list_length_) {
      name_type_pairs.push_back({names_list_ + ni->name_offset_, ni->is_route_num_});
    } else {
      throw std::runtime_error("GetNames: offset exceeds size of text list");
    }
  }
  return name_type_pairs;
}

// Get the linguistic, tagged names for an edge
std::vector<std::string> EdgeInfo::GetLinguisticTaggedValues() const {
  // Get each name
  std::vector<std::string> names;

  names.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    if (!ni->tagged_)
      continue;

    if (ni->name_offset_ < names_list_length_) {
      const auto* name = names_list_ + ni->name_offset_;
      try {
        TaggedValue tv = static_cast<baldr::TaggedValue>(name[0]);
        if (tv == baldr::TaggedValue::kLinguistic) {
          name += 1;
          while (*name != '\0') {
            const auto header = midgard::unaligned_read<linguistic_text_header_t>(name);
            names.emplace_back(
                std::string(reinterpret_cast<const char*>(&header), kLinguisticHeaderSize) +
                std::string((name + kLinguisticHeaderSize), header.length_));
            name += header.length_ + kLinguisticHeaderSize;
          }
        }
      } catch (const std::invalid_argument& arg) {
        LOG_DEBUG("invalid_argument thrown for name: " + std::string(name));
      }
    } else {
      throw std::runtime_error("GetTaggedNames: offset exceeds size of text list");
    }
  }
  return names;
}

// Get a list of names
std::vector<std::tuple<std::string, bool, uint8_t>>
EdgeInfo::GetNamesAndTypes(bool include_tagged_values) const {
  // Get each name
  std::vector<std::tuple<std::string, bool, uint8_t>> name_type_pairs;

  name_type_pairs.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    // Skip any tagged names (FUTURE code may make use of them)
    if (ni->tagged_ && !include_tagged_values) {
      continue;
    }

    if (ni->tagged_) {
      if (ni->name_offset_ < names_list_length_) {
        const char* name = names_list_ + ni->name_offset_;
        auto tag = name[0];
        if (IsNonLiguisticTagValue(tag)) {
          name_type_pairs.push_back({std::string(name + 1), false, static_cast<uint8_t>(tag)});
        }
      } else {
        throw std::runtime_error("GetNamesAndTypes: offset exceeds size of text list");
      }
    } else if (ni->name_offset_ < names_list_length_) {
      name_type_pairs.push_back({names_list_ + ni->name_offset_, ni->is_route_num_, 0});
    } else {
      throw std::runtime_error("GetNamesAndTypes: offset exceeds size of text list");
    }
  }
  return name_type_pairs;
}

// Get a list of tagged values.  We do not return linguistic tagged values here.  Use
// GetLinguisticTaggedValues to obtain those
std::vector<std::string> EdgeInfo::GetTaggedValues() const {
  // Get each name
  std::vector<std::string> tagged_values;
  tagged_values.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    if (!ni->tagged_)
      continue;

    if (ni->name_offset_ < names_list_length_) {
      const char* value = names_list_ + ni->name_offset_;
      try {
        TaggedValue tv = static_cast<baldr::TaggedValue>(value[0]);
        if (tv == baldr::TaggedValue::kLinguistic) {
          continue;
        }

        // add a per tag parser that returns 0 or more strings, parser skips tags it doesnt know
        std::vector<std::string> contents = parse_tagged_value(value);
        std::move(contents.begin(), contents.end(), std::back_inserter(tagged_values));
      } catch (const std::invalid_argument& arg) {
        LOG_DEBUG("invalid_argument thrown for tagged value: " + std::string(value));
      }
    } else {
      throw std::runtime_error("GetTaggedNames: offset exceeds size of text list");
    }
  }
  return tagged_values;
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
          const char* value = names_list_ + ni->name_offset_;
          try {
            // no pronunciations for some reason...
            TaggedValue tv = static_cast<baldr::TaggedValue>(value[0]);
            if (tv == baldr::TaggedValue::kLinguistic) {
              continue;
            }
            // get whatever tag value was in there
            // add a per tag parser that returns 0 or more strings, parser skips tags it doesnt know
            auto contents = parse_tagged_value(value);
            for (const std::string& c : contents) {
              // remove the leading TaggedValue byte from the content
              tag_cache_.emplace(tv, c.substr(1));
            }
          } catch (const std::logic_error& arg) {
            LOG_DEBUG("logic_error thrown for tagged value: " + std::string(value));
          }
        } else {
          throw std::runtime_error("GetTags: offset exceeds size of text list");
        }
      }
    }

    if (tag_cache_.size())
      tag_cache_ready_ = true;
  }

  return tag_cache_;
}

std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>>
EdgeInfo::GetLinguisticMap() const {
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> index_linguistic_map;
  index_linguistic_map.reserve(name_count());
  const NameInfo* ni = name_info_list_;
  for (uint32_t i = 0; i < name_count(); i++, ni++) {
    if (!ni->tagged_)
      continue;

    if (ni->name_offset_ < names_list_length_) {
      const auto* name = names_list_ + ni->name_offset_;
      try {
        TaggedValue tv = static_cast<baldr::TaggedValue>(name[0]);
        if (tv == baldr::TaggedValue::kLinguistic) {
          name += 1;
          while (*name != '\0') {
            std::tuple<uint8_t, uint8_t, std::string> liguistic_attributes;
            uint8_t name_index = 0;
            const auto header = midgard::unaligned_read<linguistic_text_header_t>(name);

            std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(liguistic_attributes) =
                header.phonetic_alphabet_;
            std::get<kLinguisticMapTupleLanguageIndex>(liguistic_attributes) = header.language_;

            std::get<kLinguisticMapTuplePronunciationIndex>(liguistic_attributes) =
                std::string(name + kLinguisticHeaderSize, header.length_);
            name += header.length_ + kLinguisticHeaderSize;
            name_index = header.name_index_;

            auto iter = index_linguistic_map.insert(std::make_pair(name_index, liguistic_attributes));

            // Edge case.  Sometimes when phonemes exist but the language for that phoneme is not
            // supported in that area, we toss the phoneme but add the default language for that
            // name/destination key.  We only want to return the highest ranking phoneme type
            // over the language.
            if (!iter.second &&
                (std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(liguistic_attributes) >
                 std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(iter.first->second)) &&
                (std::get<kLinguisticMapTuplePhoneticAlphabetIndex>(liguistic_attributes) !=
                 static_cast<uint8_t>(PronunciationAlphabet::kNone)) &&
                (std::get<kLinguisticMapTupleLanguageIndex>(liguistic_attributes) ==
                 std::get<kLinguisticMapTupleLanguageIndex>(iter.first->second))) {
              iter.first->second = liguistic_attributes;
            }
          }
        }
      } catch (const std::invalid_argument& arg) {
        LOG_DEBUG("invalid_argument thrown for name: " + std::string(name));
      }
    } else {
      throw std::runtime_error("GetLinguisticMap: offset exceeds size of text list");
    }
  }

  return index_linguistic_map;
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

// Returns the encoded elevation along the edge as well as the sampling interval.
// The sampling interval is uniform (based on the length of the edge).
std::vector<int8_t> EdgeInfo::encoded_elevation(const uint32_t length, double& interval) const {
  if (!has_elevation()) {
    // If no elevation then the edge length is shorter than the sampling interval...set the
    // interval to the edge length. The elevation at the nodes will be used in this case.
    interval = length;
    return std::vector<int8_t>();
  }

  // Set the sampling interval
  interval = midgard::sampling_interval(length);

  // Number of elevations encoded (start and end of the edge are stored in NodeInfo)
  uint32_t n = midgard::encoded_elevation_count(length);
  return std::vector<int8_t>(encoded_elevation_, encoded_elevation_ + n);
}

std::vector<ConditionalSpeedLimit> EdgeInfo::conditional_speed_limits() const {
  std::vector<ConditionalSpeedLimit> limits;
  const auto cond_limits_range = GetTags().equal_range(TaggedValue::kConditionalSpeedLimits);
  for (auto it = cond_limits_range.first; it != cond_limits_range.second; ++it) {
    const ConditionalSpeedLimit* l =
        reinterpret_cast<const ConditionalSpeedLimit*>(it->second.data());
    limits.push_back(*l);
  }
  return limits;
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

std::pair<std::vector<std::pair<float, float>>, uint32_t> EdgeInfo::levels() const {
  const auto& tags = GetTags();
  auto itr = tags.find(TaggedValue::kLevels);
  if (itr == tags.end()) {
    return {};
  }
  try {
    return decode_levels(itr->second);
  } catch (...) { throw std::runtime_error("failed to decode levels"); };
}

bool EdgeInfo::includes_level(float lvl) const {
  const auto& tags = GetTags();
  auto itr = tags.find(TaggedValue::kLevels);
  if (itr == tags.end()) {
    return false;
  }
  auto decoded = std::get<0>(decode_levels(itr->second));
  auto lower = std::lower_bound(decoded.cbegin(), decoded.cend(), lvl,
                                [&](const decltype(decoded)::value_type& val, float lvl) {
                                  return val.second < lvl;
                                });
  return lower != decoded.end() && lower->first <= lvl && lvl <= lower->second;
}

std::vector<std::string> EdgeInfo::level_ref() const {
  const auto& tags = GetTags();
  std::vector<std::string> values;
  for (auto [itr, range_end] = tags.equal_range(TaggedValue::kLevelRef); itr != range_end; ++itr) {
    values.emplace_back(itr->second);
  }
  return values;
}

json::MapPtr EdgeInfo::json() const {
  json::MapPtr edge_info = json::map({
      {"way_id", static_cast<uint64_t>(wayid())},
      {"bike_network", bike_network_json(bike_network())},
      {"names", names_json(GetNames())},
      {"shape", midgard::encode(shape())},
  });
  // add the mean_elevation depending on its validity
  const auto elev = mean_elevation();
  if (elev == kNoElevationData) {
    edge_info->emplace("mean_elevation", nullptr);
  } else {
    edge_info->emplace("mean_elevation", static_cast<int64_t>(elev));
  }

  if (speed_limit() == kUnlimitedSpeedLimit) {
    edge_info->emplace("speed_limit", std::string("unlimited"));
  } else {
    edge_info->emplace("speed_limit", static_cast<uint64_t>(speed_limit()));
  }

  json::MapPtr conditional_speed_limits;
  for (const auto& [tag, value] : GetTags()) {
    switch (tag) {
      case TaggedValue::kLayer:
        break;
      case TaggedValue::kLinguistic:
        break;
      case TaggedValue::kBssInfo:
        break;
      case TaggedValue::kLevel:
        break;
      case TaggedValue::kLevelRef:
        break;
      case TaggedValue::kLandmark:
        break;
      case TaggedValue::kLevels: {
        json::ArrayPtr levels = json::array({});
        std::vector<std::pair<float, float>> decoded;
        uint32_t precision;
        std::tie(decoded, precision) = decode_levels(value);
        for (auto& range : decoded) {
          if (range.first == range.second) {
            // single number
            levels->emplace_back(json::fixed_t{range.first, precision});
          } else {
            // range
            json::ArrayPtr level = json::array({});
            level->emplace_back(json::fixed_t{range.first, precision});
            level->emplace_back(json::fixed_t{range.second, precision});
            levels->emplace_back(level);
          }
        }
        edge_info->emplace("levels", levels);
        break;
      }
      case TaggedValue::kConditionalSpeedLimits: {
        if (!conditional_speed_limits) {
          conditional_speed_limits = json::map({});
        }
        const ConditionalSpeedLimit* l = reinterpret_cast<const ConditionalSpeedLimit*>(value.data());
        conditional_speed_limits->emplace(l->td_.to_string(), static_cast<uint64_t>(l->speed_));
        break;
      }
      case TaggedValue::kTunnel:
        break;
      case TaggedValue::kBridge:
        break;
    }
  }
  if (conditional_speed_limits) {
    edge_info->emplace("conditional_speed_limits", std::move(conditional_speed_limits));
  }

  return edge_info;
}

} // namespace baldr
} // namespace valhalla
