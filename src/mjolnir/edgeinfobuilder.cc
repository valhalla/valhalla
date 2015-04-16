#include <algorithm>
#include <ostream>
#include <iostream>

#include <valhalla/midgard/util.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/edgeinfo.h>
#include "mjolnir/edgeinfobuilder.h"

namespace valhalla {
namespace mjolnir {

/**
 * Set the OSM way Id.
 * @param wayid  Way Id.
 */
void EdgeInfoBuilder::set_wayid(const uint64_t wayid) {
  wayid_ = wayid;
}

// Set the indexes to names used by this edge.
void EdgeInfoBuilder::set_text_name_offset_list(
    const std::vector<uint32_t>& text_name_offset_list) {
  text_name_offset_list_ = text_name_offset_list;
}

// Set the shape of the edge. Encode the vector of lat,lng to a string.
void EdgeInfoBuilder::set_shape(const std::vector<PointLL>& shape) {
  encoded_shape_ = midgard::encode<std::vector<PointLL> >(shape);
}

// Get the size of the edge info (including name offsets and shape string)
std::size_t EdgeInfoBuilder::SizeOf() const {
  std::size_t size = sizeof(uint64_t);
  size += sizeof(baldr::EdgeInfo::PackedItem);
  size += (text_name_offset_list_.size() * sizeof(uint32_t));
  size += (encoded_shape_.size() * sizeof(std::string::value_type));
  return size;
}

std::ostream& operator<<(std::ostream& os, const EdgeInfoBuilder& eib) {
  // Pack the name count and encoded shape size. Check against limits.
  baldr::EdgeInfo::PackedItem item;
  uint32_t name_count = eib.text_name_offset_list_.size();
  if (name_count > kMaxNamesPerEdge) {
    LOG_WARN("Exceeding max names per edge: " + std::to_string(name_count));
    name_count = kMaxNamesPerEdge;
  }
  item.name_count = name_count;

  // Check if we are exceeding the max encoded size
  if (eib.encoded_shape_.size() > kMaxEncodedShapeSize) {
    LOG_ERROR("Exceeding max encoded shape size: " +
              std::to_string(eib.encoded_shape_.size()));
    item.encoded_shape_size = static_cast<uint32_t>(kMaxEncodedShapeSize);
  } else {
    item.encoded_shape_size = static_cast<uint32_t>(eib.encoded_shape_.size());
  }

  // Write out the bytes
  os.write(reinterpret_cast<const char*>(&eib.wayid_), sizeof(uint64_t));
  os.write(reinterpret_cast<const char*>(&item), sizeof(baldr::EdgeInfo::PackedItem));
  os.write(reinterpret_cast<const char*>(&eib.text_name_offset_list_[0]),
            (name_count * sizeof(uint32_t)));
  os << eib.encoded_shape_;

  return os;
}

}
}
