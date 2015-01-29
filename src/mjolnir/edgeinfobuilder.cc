#include <algorithm>
#include <ostream>
#include <iostream>

#include <valhalla/midgard/util.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/edgeinfo.h>
#include "mjolnir/edgeinfobuilder.h"

namespace valhalla {
namespace mjolnir {

// Set the indexes to names used by this edge.
void EdgeInfoBuilder::set_street_name_offset_list(
    const std::vector<uint32_t>& street_name_offset_list) {
  street_name_offset_list_ = street_name_offset_list;
}

// Set the shape of the edge.
void EdgeInfoBuilder::set_shape(const std::vector<PointLL>& shape) {
  // Set the shape
  encoded_shape_ = midgard::encode<std::vector<PointLL> >(shape);
}

// Set the exit signs used by this edge.
void EdgeInfoBuilder::set_exit_signs(std::vector<ExitSignBuilder>&& exit_signs) {
  exit_signs_ = std::move(exit_signs);
}

std::size_t EdgeInfoBuilder::SizeOf() const {
  std::size_t size = 0;
  size += sizeof(baldr::EdgeInfo::PackedItem);
  size += (street_name_offset_list_.size() * sizeof(uint32_t));
  size += (encoded_shape_.size() * sizeof(std::string::value_type));
  size += (exit_signs_.size() * sizeof(ExitSign));
  return size;
}

std::ostream& operator<<(std::ostream& os, const EdgeInfoBuilder& eib) {
  // Make packeditem
  // TODO - protect against exceeding sizes!
  baldr::EdgeInfo::PackedItem item;
  item.fields.name_count = static_cast<uint32_t>(eib.street_name_offset_list_.size());

  // Check if we are exceeding the max encoded size
  if (eib.encoded_shape_.size() > kMaxEncodedShapeSize) {
    LOG_ERROR("EXCEEDING kMaxEncodedShapeSize: " +
              std::to_string(eib.encoded_shape_.size()));
    item.fields.encoded_shape_size = static_cast<uint32_t>(kMaxEncodedShapeSize);
  } else {
    item.fields.encoded_shape_size = static_cast<uint32_t>(eib.encoded_shape_.size());
  }
  item.fields.exit_sign_count = static_cast<uint32_t>(eib.exit_signs_.size());

  // Write out the bytes
  os.write(reinterpret_cast<const char*>(&item), sizeof(baldr::EdgeInfo::PackedItem));
  os.write(reinterpret_cast<const char*>(&eib.street_name_offset_list_[0]),
            (eib.street_name_offset_list_.size() * sizeof(uint32_t)));
  os << eib.encoded_shape_;
  os.write(reinterpret_cast<const char*>(&eib.exit_signs_[0]),
            (eib.exit_signs_.size() * sizeof(ExitSign)));

  return os;
}

}
}
