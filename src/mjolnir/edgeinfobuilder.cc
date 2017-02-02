#include <algorithm>
#include <ostream>
#include <iostream>

#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "baldr/edgeinfo.h"
#include "mjolnir/edgeinfobuilder.h"

namespace valhalla {
namespace mjolnir {

// Set the OSM way Id.
void EdgeInfoBuilder::set_wayid(const uint64_t wayid) {
  wayid_ = wayid;
}

// Set the list of name info (offsets, etc.) used by this edge.
void EdgeInfoBuilder::set_name_info_list(
    const std::vector<NameInfo>& name_info_list) {
  if (name_info_list.size() > kMaxNamesPerEdge) {
    LOG_WARN("Tried to exceed max names per edge: " +
                  std::to_string(name_info_list.size()));
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
template <class shape_container_t>
void EdgeInfoBuilder::set_shape(const shape_container_t& shape) {
  encoded_shape_ = midgard::encode7<shape_container_t>(shape);
}
template void EdgeInfoBuilder::set_shape<std::vector<PointLL> >(const std::vector<PointLL>&);
template void EdgeInfoBuilder::set_shape<std::list<PointLL> >(const std::list<PointLL>&);


// Set the encoded shape string.
void EdgeInfoBuilder::set_encoded_shape(const std::string& encoded_shape) {
  std::copy(encoded_shape.begin(), encoded_shape.end(),
            back_inserter(encoded_shape_));
}

// Get the size of the edge info (including name offsets and shape string)
std::size_t EdgeInfoBuilder::BaseSizeOf() const {
  std::size_t size = sizeof(uint64_t);
  size += sizeof(baldr::EdgeInfo::PackedItem);
  size += (name_info_list_.size() * sizeof(NameInfo));
  size += (encoded_shape_.size() * sizeof(std::string::value_type));
  return size;
}

// Get the size of the edge info (including name offsets and shape string)
std::size_t EdgeInfoBuilder::SizeOf() const {
  std::size_t size = BaseSizeOf();

  // Add padding to get to 8 byte boundaries
  size_t n = size % 8;
  if (n != 0) {
    size += 8 - n;
  }
  return size;
}

// Output edge info to output stream
std::ostream& operator<<(std::ostream& os, const EdgeInfoBuilder& eib) {
  // Pack the name count and encoded shape size. Check against limits.
  baldr::EdgeInfo::PackedItem item;
  uint32_t name_count = eib.name_info_list_.size();
  if (name_count > kMaxNamesPerEdge) {
    LOG_WARN("Exceeding max names per edge: " + std::to_string(name_count));
    name_count = kMaxNamesPerEdge;
  }
  item.name_count = name_count;

  // Check if we are exceeding the max encoded size
  if (eib.encoded_shape_.size() > kMaxEncodedShapeSize) {
    LOG_WARN("Exceeding max encoded shape size: " +
              std::to_string(eib.encoded_shape_.size()));
    item.encoded_shape_size = static_cast<uint32_t>(kMaxEncodedShapeSize);
  } else {
    item.encoded_shape_size = static_cast<uint32_t>(eib.encoded_shape_.size());
  }

  // Write out the bytes
  os.write(reinterpret_cast<const char*>(&eib.wayid_), sizeof(uint64_t));
  os.write(reinterpret_cast<const char*>(&item), sizeof(baldr::EdgeInfo::PackedItem));
  os.write(reinterpret_cast<const char*>(&eib.name_info_list_[0]),
            (name_count * sizeof(NameInfo)));
  os << eib.encoded_shape_;

  // Pad to an 8 byte boundary
  std::size_t n = (eib.BaseSizeOf() % 8);
  if (n != 0) {
    for (std::size_t i = 0; i < 8 - n; i++) {
      os << static_cast<char>(0);
    }
  }
  return os;
}

}
}
