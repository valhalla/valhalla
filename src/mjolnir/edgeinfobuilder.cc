#include <algorithm>
#include <iostream>
#include <ostream>

#include "baldr/edgeinfo.h"
#include "baldr/graphconstants.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "mjolnir/edgeinfobuilder.h"

namespace valhalla {
namespace mjolnir {

// Set the OSM way Id.
void EdgeInfoBuilder::set_wayid(const uint64_t wayid) {
  // mask off the various parts of the id into their respective spots
  ei_.wayid_ = wayid & 0xFFFFFFFF;
  ei_.extended_wayid0_ = (wayid >> 32) & 0xFF;
  ei_.extended_wayid1_ = (wayid >> 40) & 0xFF;
  extended_wayid2_ = (wayid >> 48) & 0xFF;
  extended_wayid3_ = (wayid >> 56) & 0xFF;
  ei_.extended_wayid_size_ = extended_wayid3_ > 0 ? 2 : (extended_wayid2_ > 0 ? 1 : 0);
}

// Set the mean elevation.
void EdgeInfoBuilder::set_mean_elevation(const float mean_elev) {
  if (mean_elev <= kMinElevation) {
    ei_.mean_elevation_ = 0;
  } else {
    uint32_t elev = static_cast<uint32_t>((mean_elev - kMinElevation) / kElevationBinSize);
    ei_.mean_elevation_ = (elev > kMaxStoredElevation) ? kMaxStoredElevation : elev;
  }
}

// Sets the bike network mask indicating which (if any) bicycle networks are
// along this edge. See baldr/directededge.h for definitions.
void EdgeInfoBuilder::set_bike_network(const uint32_t bike_network) {
  ei_.bike_network_ = bike_network;
}

// Sets the speed limit in KPH.
void EdgeInfoBuilder::set_speed_limit(const uint32_t speed_limit) {
  if (speed_limit == kUnlimitedSpeedLimit) {
    ei_.speed_limit_ = kUnlimitedSpeedLimit;
  } else if (speed_limit > kMaxAssumedSpeed) {
    LOG_WARN("Exceeding maximum.  Speed limit: " + std::to_string(speed_limit));
    ei_.speed_limit_ = kMaxAssumedSpeed;
  } else {
    ei_.speed_limit_ = speed_limit;
  }
}

// Set the list of name info (offsets, etc.) used by this edge.
void EdgeInfoBuilder::set_name_info_list(const std::vector<NameInfo>& name_info_list) {
  if (name_info_list.size() > kMaxNamesPerEdge) {
    LOG_WARN("Tried to exceed max names per edge: " + std::to_string(name_info_list.size()));
  } else {
    name_info_list_ = name_info_list;
  }
}

// Add street name info to the list.
void EdgeInfoBuilder::AddNameInfo(const baldr::NameInfo& info) {
  if (name_info_list_.size() == kMaxNamesPerEdge) {
    LOG_WARN("Tried to exceed max names per edge");
  } else {
    name_info_list_.push_back(info);
  }
}

// Set the shape of the edge. Encode the vector of lat,lng to a string.
template <class shape_container_t> void EdgeInfoBuilder::set_shape(const shape_container_t& shape) {
  encoded_shape_ = midgard::encode7<shape_container_t>(shape);
}
template void EdgeInfoBuilder::set_shape<std::vector<PointLL>>(const std::vector<PointLL>&);
template void EdgeInfoBuilder::set_shape<std::list<PointLL>>(const std::list<PointLL>&);

// Set the encoded shape string.
void EdgeInfoBuilder::set_encoded_shape(const std::string& encoded_shape) {
  std::copy(encoded_shape.begin(), encoded_shape.end(), back_inserter(encoded_shape_));
}

// Get the size of the edge info (including name offsets and shape string)
std::size_t EdgeInfoBuilder::BaseSizeOf() const {
  std::size_t size = sizeof(EdgeInfo::EdgeInfoInner);
  size += (name_info_list_.size() * sizeof(NameInfo));
  size += (encoded_shape_.size() * sizeof(std::string::value_type));
  size += ei_.extended_wayid_size_;
  return size;
}

// Get the size of the edge info (including name offsets and shape string)
std::size_t EdgeInfoBuilder::SizeOf() const {
  std::size_t size = BaseSizeOf();

  // Add padding to get to 4 byte boundaries
  size_t n = size % 4;
  if (n != 0) {
    size += 4 - n;
  }
  return size;
}

// Output edge info to output stream
std::ostream& operator<<(std::ostream& os, const EdgeInfoBuilder& eib) {
  // Pack the name count and encoded shape size. Check against limits.
  auto ei = eib.ei_;
  uint32_t name_count = eib.name_info_list_.size();
  if (name_count > kMaxNamesPerEdge) {
    LOG_WARN("Exceeding max names per edge: " + std::to_string(name_count));
    name_count = kMaxNamesPerEdge;
  }
  ei.name_count_ = name_count;

  // Check if we are exceeding the max encoded size
  if (eib.encoded_shape_.size() > kMaxEncodedShapeSize) {
    LOG_WARN("Exceeding max encoded shape size: " + std::to_string(eib.encoded_shape_.size()));
    ei.encoded_shape_size_ = static_cast<uint32_t>(kMaxEncodedShapeSize);
  } else {
    ei.encoded_shape_size_ = static_cast<uint32_t>(eib.encoded_shape_.size());
  }

  // Write out the bytes
  os.write(reinterpret_cast<const char*>(&ei), sizeof(ei));
  os.write(reinterpret_cast<const char*>(eib.name_info_list_.data()),
           (name_count * sizeof(NameInfo)));
  os << eib.encoded_shape_;
  if (ei.extended_wayid_size_ > 0) {
    os.write(reinterpret_cast<const char*>(&eib.extended_wayid2_), sizeof(eib.extended_wayid2_));
  }
  if (ei.extended_wayid_size_ > 1) {
    os.write(reinterpret_cast<const char*>(&eib.extended_wayid3_), sizeof(eib.extended_wayid3_));
  }

  // Pad to a 4 byte boundary
  std::size_t padding = (eib.BaseSizeOf() % 4);
  padding = padding > 0 ? 4 - padding : 0;
  if (padding > 0) {
    os.write("\0\0\0\0", padding);
  }

  return os;
}

} // namespace mjolnir
} // namespace valhalla
