#include "mjolnir/osmway.h"
#include "mjolnir/util.h"

#include "midgard/logging.h"
#include <boost/algorithm/string.hpp>
#include <iostream>

using namespace valhalla::baldr;

namespace {

constexpr uint32_t kMaxNodesPerWay = 65535;
constexpr uint8_t kUnlimitedOSMSpeed = std::numeric_limits<uint8_t>::max();
constexpr float kMaxOSMSpeed = 140.0f;

} // namespace

namespace valhalla {
namespace mjolnir {

// Set the number of nodes for this way.
void OSMWay::set_node_count(const uint32_t count) {
  if (count > kMaxNodesPerWay) {
    LOG_WARN("Exceeded max nodes per way: " + std::to_string(count));
    nodecount_ = static_cast<uint16_t>(kMaxNodesPerWay);
  } else {
    nodecount_ = static_cast<uint16_t>(count);
  }
}

// Sets the speed in KPH.
void OSMWay::set_speed(const float speed) {
  if (speed > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max speed for way id: " + std::to_string(osmwayid_));
    speed_ = kMaxOSMSpeed;
  } else {
    speed_ = static_cast<unsigned char>(speed + 0.5f);
  }
}

// Sets the speed limit in KPH.
void OSMWay::set_speed_limit(const float speed_limit) {
  if (speed_limit == kUnlimitedOSMSpeed) {
    speed_limit_ = kUnlimitedOSMSpeed;
  } else if (speed_limit > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max speed for way id: " + std::to_string(osmwayid_));
    speed_limit_ = kMaxOSMSpeed;
  } else {
    speed_limit_ = static_cast<unsigned char>(speed_limit + 0.5f);
  }
}

// Sets the backward speed in KPH.
void OSMWay::set_backward_speed(const float backward_speed) {
  if (backward_speed > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max backward speed for way id: " + std::to_string(osmwayid_));
    backward_speed_ = kMaxOSMSpeed;
  } else {
    backward_speed_ = static_cast<unsigned char>(backward_speed + 0.5f);
  }
}

// Sets the backward speed in KPH.
void OSMWay::set_forward_speed(const float forward_speed) {
  if (forward_speed > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max forward speed for way id: " + std::to_string(osmwayid_));
    forward_speed_ = kMaxOSMSpeed;
  } else {
    forward_speed_ = static_cast<unsigned char>(forward_speed + 0.5f);
  }
}

// Sets the truck speed in KPH.
void OSMWay::set_truck_speed(const float speed) {
  if (speed > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max truck speed for way id: " + std::to_string(osmwayid_));
    truck_speed_ = kMaxOSMSpeed;
  } else {
    truck_speed_ = static_cast<unsigned char>(speed + 0.5f);
  }
}

// Sets the number of lanes
void OSMWay::set_lanes(const uint32_t lanes) {
  lanes_ = (lanes > kMaxLaneCount) ? kMaxLaneCount : lanes;
}

// Sets the number of backward lanes
void OSMWay::set_backward_lanes(const uint32_t backward_lanes) {
  backward_lanes_ = (backward_lanes > kMaxLaneCount) ? kMaxLaneCount : backward_lanes;
}

// Sets the number of forward lanes
void OSMWay::set_forward_lanes(const uint32_t forward_lanes) {
  forward_lanes_ = (forward_lanes > kMaxLaneCount) ? kMaxLaneCount : forward_lanes;
}

void OSMWay::AddPronunciations(std::vector<std::string>& pronunciations,
                               const UniqueNames& name_offset_map,
                               const uint32_t pronunciation_index,
                               const size_t name_tokens_size,
                               const size_t key,
                               const baldr::VerbalType verbal_type) const {

  size_t k = key;
  std::vector<std::string> pronunciation_tokens;
  if (pronunciation_index != 0)
    pronunciation_tokens = GetTagTokens(name_offset_map.name(pronunciation_index));
  else
    return;

  if (pronunciation_tokens.size() && name_tokens_size == pronunciation_tokens.size()) {
    for (const auto& t : pronunciation_tokens) {
      pronunciations.emplace_back(std::to_string(static_cast<uint8_t>(k)) + '\0' +
                                  std::to_string(static_cast<uint8_t>(verbal_type)) + '\0' + t);
      k++;
    }
  }
}

// Get the names for the edge info based on the road class.
void OSMWay::GetNames(const std::string& ref,
                      const UniqueNames& name_offset_map,
                      const OSMPronunciation& pronunciation,
                      uint16_t& types,
                      std::vector<std::string>& names,
                      std::vector<std::string>& pronunciations) const {

  uint16_t location = 0;
  types = 0;

  // Process motorway and trunk refs
  if ((ref_index_ != 0 || !ref.empty()) &&
      ((static_cast<RoadClass>(road_class_) == RoadClass::kMotorway) ||
       (static_cast<RoadClass>(road_class_) == RoadClass::kTrunk))) {
    std::vector<std::string> tokens;
    std::vector<std::string> pronunciation_tokens;

    if (!ref.empty()) {
      tokens = GetTagTokens(ref); // use updated refs from relations.
    } else {
      tokens = GetTagTokens(name_offset_map.name(ref_index_));
    }

    for (const auto& t : tokens) {
      types |= static_cast<uint64_t>(1) << location;
      location++;
    }

    names.insert(names.end(), tokens.begin(), tokens.end());

    size_t key = names.size() - tokens.size();
    AddPronunciations(pronunciations, name_offset_map, pronunciation.ref_pronunciation_ipa_index(),
                      names.size(), key, VerbalType::kIpa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.ref_pronunciation_x_sampa_index(), names.size(), key,
                      VerbalType::kXSampa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.ref_pronunciation_katakana_index(), names.size(), key,
                      VerbalType::kPlainText);
  }

  // TODO int_ref

  // Process name
  if (name_index_ != 0) {

    std::vector<std::string> tokens;
    tokens = GetTagTokens(name_offset_map.name(name_index_));
    location += tokens.size();

    names.insert(names.end(), tokens.begin(), tokens.end());

    size_t key = names.size() - tokens.size();
    AddPronunciations(pronunciations, name_offset_map, pronunciation.name_pronunciation_ipa_index(),
                      names.size(), key, VerbalType::kIpa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.name_pronunciation_x_sampa_index(), names.size(), key,
                      VerbalType::kXSampa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.name_pronunciation_katakana_index(), names.size(), key,
                      VerbalType::kPlainText);
  }

  // Process non limited access refs
  if (ref_index_ != 0 && (static_cast<RoadClass>(road_class_) != RoadClass::kMotorway) &&
      (static_cast<RoadClass>(road_class_) != RoadClass::kTrunk)) {
    std::vector<std::string> tokens;

    if (!ref.empty()) {
      tokens = GetTagTokens(ref); // use updated refs from relations.
    } else {
      tokens = GetTagTokens(name_offset_map.name(ref_index_));
    }

    for (const auto& t : tokens) {
      types |= static_cast<uint64_t>(1) << location;
      location++;
    }

    names.insert(names.end(), tokens.begin(), tokens.end());

    size_t key = names.size() - tokens.size();
    AddPronunciations(pronunciations, name_offset_map, pronunciation.ref_pronunciation_ipa_index(),
                      names.size(), key, VerbalType::kIpa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.ref_pronunciation_x_sampa_index(), names.size(), key,
                      VerbalType::kXSampa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.ref_pronunciation_katakana_index(), names.size(), key,
                      VerbalType::kPlainText);
  }

  // Process alt_name
  if (alt_name_index_ != 0 && alt_name_index_ != name_index_) {

    std::vector<std::string> tokens;
    tokens = GetTagTokens(name_offset_map.name(alt_name_index_));
    location += tokens.size();

    names.insert(names.end(), tokens.begin(), tokens.end());

    size_t key = names.size() - tokens.size();
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.alt_name_pronunciation_ipa_index(), names.size(), key,
                      VerbalType::kIpa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.alt_name_pronunciation_x_sampa_index(), names.size(), key,
                      VerbalType::kXSampa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.alt_name_pronunciation_katakana_index(), names.size(), key,
                      VerbalType::kPlainText);
  }
  // Process official_name
  if (official_name_index_ != 0 && official_name_index_ != name_index_ &&
      official_name_index_ != alt_name_index_) {

    std::vector<std::string> tokens;
    tokens = GetTagTokens(name_offset_map.name(official_name_index_));
    location += tokens.size();

    names.insert(names.end(), tokens.begin(), tokens.end());

    size_t key = names.size() - tokens.size();
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.official_name_pronunciation_ipa_index(), names.size(), key,
                      VerbalType::kIpa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.official_name_pronunciation_x_sampa_index(), names.size(), key,
                      VerbalType::kXSampa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.official_name_pronunciation_katakana_index(), names.size(), key,
                      VerbalType::kPlainText);
  }
  // Process name_en_
  // TODO: process country specific names
  if (name_en_index_ != 0 && name_en_index_ != name_index_ && name_en_index_ != alt_name_index_ &&
      name_en_index_ != official_name_index_) {

    std::vector<std::string> tokens;
    tokens = GetTagTokens(name_offset_map.name(name_en_index_));
    location += tokens.size();

    names.insert(names.end(), tokens.begin(), tokens.end());

    size_t key = names.size() - tokens.size();
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.name_en_pronunciation_ipa_index(), names.size(), key,
                      VerbalType::kIpa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.name_en_pronunciation_x_sampa_index(), names.size(), key,
                      VerbalType::kXSampa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.name_en_pronunciation_katakana_index(), names.size(), key,
                      VerbalType::kPlainText);
  }
}

// Get the tagged names for an edge
void OSMWay::GetTaggedNames(const UniqueNames& name_offset_map,
                            const OSMPronunciation& pronunciation,
                            const size_t& names_size,
                            std::vector<std::string>& names,
                            std::vector<std::string>& pronunciations) const {
  std::vector<std::string> tokens;

  if (tunnel_name_index_ != 0) {
    // tunnel names
    tokens = GetTagTokens(name_offset_map.name(tunnel_name_index_));
    for (const auto& t : tokens) {
      names.emplace_back(std::to_string(static_cast<uint8_t>(TaggedName::kTunnel)) + t);
    }

    size_t key = (names_size + names.size()) - tokens.size();
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.tunnel_name_pronunciation_ipa_index(), names.size(), key,
                      VerbalType::kIpa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.tunnel_name_pronunciation_x_sampa_index(), names.size(), key,
                      VerbalType::kXSampa);
    AddPronunciations(pronunciations, name_offset_map,
                      pronunciation.tunnel_name_pronunciation_katakana_index(), names.size(), key,
                      VerbalType::kPlainText);
  }
}

} // namespace mjolnir
} // namespace valhalla
