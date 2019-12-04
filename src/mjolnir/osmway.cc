#include "mjolnir/osmway.h"
#include "mjolnir/util.h"

#include "midgard/logging.h"
#include <iostream>

using namespace valhalla::baldr;

namespace {

constexpr uint32_t kMaxNodesPerWay = 65535;
constexpr float kMaxOSMSpeed = 255.0f;

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
    speed_ = 255;
  } else {
    speed_ = static_cast<unsigned char>(speed + 0.5f);
  }
}

// Sets the speed limit in KPH.
void OSMWay::set_speed_limit(const float speed_limit) {
  if (speed_limit > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max speed for way id: " + std::to_string(osmwayid_));
    speed_limit_ = 255;
  } else {
    speed_limit_ = static_cast<unsigned char>(speed_limit + 0.5f);
  }
}

// Sets the backward speed in KPH.
void OSMWay::set_backward_speed(const float backward_speed) {
  if (backward_speed > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max backward speed for way id: " + std::to_string(osmwayid_));
    backward_speed_ = 255;
  } else {
    backward_speed_ = static_cast<unsigned char>(backward_speed + 0.5f);
  }
}

// Sets the backward speed in KPH.
void OSMWay::set_forward_speed(const float forward_speed) {
  if (forward_speed > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max forward speed for way id: " + std::to_string(osmwayid_));
    forward_speed_ = 255;
  } else {
    forward_speed_ = static_cast<unsigned char>(forward_speed + 0.5f);
  }
}

// Sets the truck speed in KPH.
void OSMWay::set_truck_speed(const float speed) {
  if (speed > kMaxOSMSpeed) {
    LOG_WARN("Exceeded max truck speed for way id: " + std::to_string(osmwayid_));
    truck_speed_ = 255;
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

// Get the names for the edge info based on the road class.
std::vector<std::string>
OSMWay::GetNames(const std::string& ref, const UniqueNames& name_offset_map, uint16_t& types) const {

  uint16_t location = 0;
  types = 0;

  std::vector<std::string> names;
  // Process motorway and trunk refs
  if ((ref_index_ != 0 || !ref.empty()) &&
      ((static_cast<RoadClass>(road_class_) == RoadClass::kMotorway) ||
       (static_cast<RoadClass>(road_class_) == RoadClass::kTrunk))) {
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
  }

  // TODO int_ref

  // Process name
  if (name_index_ != 0) {
    names.emplace_back(name_offset_map.name(name_index_));
    location++;
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
  }

  // Process alt_name
  if (alt_name_index_ != 0 && alt_name_index_ != name_index_) {
    names.emplace_back(name_offset_map.name(alt_name_index_));
    location++;
  }
  // Process official_name
  if (official_name_index_ != 0 && official_name_index_ != name_index_ &&
      official_name_index_ != alt_name_index_) {
    names.emplace_back(name_offset_map.name(official_name_index_));
    location++;
  }
  // Process name_en_
  // TODO: process country specific names
  if (name_en_index_ != 0 && name_en_index_ != name_index_ && name_en_index_ != alt_name_index_ &&
      name_en_index_ != official_name_index_) {
    names.emplace_back(name_offset_map.name(name_en_index_));
    location++;
  }
  return names;
}

} // namespace mjolnir
} // namespace valhalla
