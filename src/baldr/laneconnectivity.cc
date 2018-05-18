#include "baldr/laneconnectivity.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <string.h>
#include <valhalla/midgard/logging.h>
#include <vector>

namespace valhalla {
namespace baldr {

// Constructor with arguments
LaneConnectivity::LaneConnectivity(const uint32_t idx,
                                   const uint64_t from,
                                   const std::string& to_lanes,
                                   const std::string& from_lanes)
    : to_(idx), from_(from), to_lanes_(to_lanes), from_lanes_(from_lanes) {
  if (from >= (1LL << 42)) {
    throw std::out_of_range("from way_id is too large");
  }
}

// Set the directed edge index.
void LaneConnectivity::set_to(const uint32_t idx) {
  to_ = idx;
}

// Returns the directed edge index.
uint32_t LaneConnectivity::to() const {
  return to_;
}

// Get the OSM id of the incoming way of this lane connection.
uint64_t LaneConnectivity::from() const {
  return from_;
}

// Get the text representation of lanes in the incoming way.
std::string LaneConnectivity::from_lanes() const {
  return from_lanes_.to_string();
}

// Get the text representation of lanes in the current way.
std::string LaneConnectivity::to_lanes() const {
  return to_lanes_.to_string();
}

// operator < - for sorting. Sort by `to` Id.
bool LaneConnectivity::operator<(const LaneConnectivity& other) const {
  return to() < other.to();
}

// Constructor with arguments.
LaneConnectivityLanes::LaneConnectivityLanes(const std::string& lanes) : value_(0) {
  std::vector<std::string> tokens;
  boost::split(tokens, lanes, boost::is_any_of("|"));
  uint8_t n = 1;
  for (const auto& t : tokens) {
    set_lane(n++, std::stoi(t));
  }
}

// Get the string representation of lane mask.
std::string LaneConnectivityLanes::to_string() const {
  std::string result;
  for (size_t i = 1; i <= kMaxLanesPerConnection; ++i) {
    uint8_t lane = get_lane(i);
    if (lane) {
      result += (result.empty() ? "" : "|") + std::to_string(lane);
    }
  }
  return result;
}

void LaneConnectivityLanes::set_lane(uint8_t n, uint8_t lane) {
  if (n == 0 || n > kMaxLanesPerConnection || lane > kMaxLanesPerConnection) {
    throw std::out_of_range("lane or index out of bounds");
  }
  value_ |= uint64_t(lane) << ((n - 1) * kMaxLanesPerConnectionBits);
}

uint8_t LaneConnectivityLanes::get_lane(uint8_t n) const {
  if (n == 0 || n > kMaxLanesPerConnection) {
    throw std::out_of_range("index out of bounds");
  }
  uint8_t v = (value_ >> ((n - 1) * kMaxLanesPerConnectionBits)) & kMaxLanesPerConnection;
  return v;
}

} // namespace baldr
} // namespace valhalla
