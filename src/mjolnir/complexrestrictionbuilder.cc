#include <algorithm>
#include <iostream>
#include <ostream>

#include "baldr/complexrestriction.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "mjolnir/complexrestrictionbuilder.h"

namespace valhalla {
namespace mjolnir {

// Set the via edge ids used by this complex restriction.
void ComplexRestrictionBuilder::set_via_list(const std::vector<GraphId>& via_list) {
  if (via_list.size() > kMaxViasPerRestriction) {
    LOG_WARN("Tried to exceed max vias per restriction: " + std::to_string(via_list.size()));
  } else {
    via_list_ = via_list;
  }
  set_via_count(via_list_.size());
}

// Output complex restriction to output stream
std::ostream& operator<<(std::ostream& os, const ComplexRestrictionBuilder& crb) {
  // Check the number of via edges against limits.
  uint32_t via_count = crb.via_list_.size();
  if (via_count > kMaxViasPerRestriction) {
    LOG_WARN("Exceeding max vias per restriction: " + std::to_string(via_count));
    via_count = kMaxViasPerRestriction;
  }

  // Write out the bytes - write the fixed part of the structure and then
  // write the via Ids (if any).
  os.write(reinterpret_cast<const char*>(&crb), 3 * sizeof(uint64_t));
  if (via_count > 0) {
    os.write(reinterpret_cast<const char*>(crb.via_list_.data()), (via_count * sizeof(GraphId)));
  }
  return os;
}

// overloaded == operator - used to ensure no dups in tiles.
bool ComplexRestrictionBuilder::operator==(const ComplexRestrictionBuilder& other) const {
  if (from_graphid_ != other.from_graphid_ || to_graphid_ != other.to_graphid_ ||
      type_ != other.type_ || modes_ != other.modes_ || has_dt_ != other.has_dt_ ||
      probability_ != other.probability_) {
    return false;
  }
  if (has_dt_ && (begin_day_dow_ != other.begin_day_dow_ || begin_hrs_ != other.begin_hrs_ ||
                  begin_mins_ != other.begin_mins_ || begin_month_ != other.begin_month_ ||
                  begin_week_ != other.begin_week_ || dow_ != other.dow_ ||
                  dt_type_ != other.dt_type_ || end_day_dow_ != other.end_day_dow_ ||
                  end_hrs_ != other.end_hrs_ || end_mins_ != other.end_mins_ ||
                  end_month_ != other.end_month_ || end_week_ != other.end_week_)) {
    return false;
  }

  if (via_list_ != other.via_list_) {
    return false;
  }

  return true;
}

} // namespace mjolnir
} // namespace valhalla
