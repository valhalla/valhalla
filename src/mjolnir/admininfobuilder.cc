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
void AdminInfoBuilder::set_text_name_offset_list(
    const std::vector<uint32_t>& text_name_offset_list) {
  text_name_offset_list_ = text_name_offset_list;
}

std::size_t AdminInfoBuilder::SizeOf() const {
  std::size_t size = 0;
  size += sizeof(baldr::AdminInfo::PackedItem);
  size += (text_name_offset_list_.size() * sizeof(uint32_t));
  return size;
}

std::ostream& operator<<(std::ostream& os, const AdminInfoBuilder& aib) {
  // Make packeditem
  // TODO - protect against exceeding sizes!
  baldr::AdminInfo::PackedItem item;

  // Check if we are exceeding the max name size
  if (aib.text_name_offset_list_.size() > kMaxNames) {
    LOG_ERROR("Exceeding max name size: " +
              std::to_string(aib.text_name_offset_list_.size()));
    item.fields.name_count = static_cast<uint32_t>(kMaxNames);
  } else {
    item.fields.name_count = static_cast<uint32_t>(aib.text_name_offset_list_.size());
  }

  // Write out the bytes
  os.write(reinterpret_cast<const char*>(&item), sizeof(baldr::AdminInfo::PackedItem));
  os.write(reinterpret_cast<const char*>(&aib.text_name_offset_list_[0]),
            (aib.text_name_offset_list_.size() * sizeof(uint32_t)));

  return os;
}

}
}
