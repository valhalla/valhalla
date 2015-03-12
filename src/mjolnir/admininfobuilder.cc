#include <algorithm>
#include <ostream>
#include <iostream>

#include <valhalla/midgard/util.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/admininfo.h>
#include "mjolnir/admininfobuilder.h"

namespace valhalla {
namespace mjolnir {

// Set the indexes to names used by this admin.
void AdminInfoBuilder::set_name_offset_list(
    const std::vector<uint32_t>& name_offset_list) {
  name_offset_list_ = name_offset_list;
}

std::size_t AdminInfoBuilder::SizeOf() const {
  std::size_t size = 0;
  size += sizeof(baldr::AdminInfo::PackedItem);
  size += (name_offset_list_.size() * sizeof(uint32_t));
  return size;
}

std::ostream& operator<<(std::ostream& os, const AdminInfoBuilder& aib) {
  // Make packeditem
  // TODO - protect against exceeding sizes!
  baldr::AdminInfo::PackedItem item;
  item.fields.name_count = static_cast<uint32_t>(aib.name_offset_list_.size());

  // Check if we are exceeding the max name size
  if (aib.name_offset_list_.size() > kMaxNames) {
    LOG_ERROR("Exceeding max name size: " +
              std::to_string(aib.name_offset_list_.size()));
    item.fields.name_count = static_cast<uint32_t>(kMaxNames);
  } else {
    item.fields.name_count = static_cast<uint32_t>(aib.name_offset_list_.size());
  }

  // Write out the bytes
  os.write(reinterpret_cast<const char*>(&item), sizeof(baldr::AdminInfo::PackedItem));
  os.write(reinterpret_cast<const char*>(&aib.name_offset_list_[0]),
            (aib.name_offset_list_.size() * sizeof(uint32_t)));

  return os;
}

}
}
