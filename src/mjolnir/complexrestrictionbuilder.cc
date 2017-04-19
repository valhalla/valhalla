#include <algorithm>
#include <ostream>
#include <iostream>

#include "midgard/util.h"
#include "midgard/logging.h"
#include "baldr/complexrestriction.h"
#include "mjolnir/complexrestrictionbuilder.h"

namespace valhalla {
namespace mjolnir {

// Set the from edge id.
void ComplexRestrictionBuilder::set_from_id(const GraphId from_id) {
  from_id_ = from_id;
}

// Set the to edge id.
void ComplexRestrictionBuilder::set_to_id(const GraphId to_id) {
  to_id_ = to_id;
}

// set the restriction type.
void ComplexRestrictionBuilder::set_type(const RestrictionType type) {
  restriction_.type_ = (static_cast<uint64_t>(type));
}

// set the access modes for the restriction
void ComplexRestrictionBuilder::set_modes(const uint64_t modes) {
  restriction_.modes_ = modes;
}

/*
// set the begin dow for this restriction
void ComplexRestrictionBuilder::set_begin_day(const DOW day) {
  restriction_.begin_day_ = static_cast<uint64_t>(day);
}

// set the end dow for this restriction
void ComplexRestrictionBuilder::set_end_day(const DOW day) {
  restriction_.end_day_ = static_cast<uint64_t>(day);
}

// set the begin time for this restriction
void ComplexRestrictionBuilder::set_begin_time(const uint64_t begin_time) {
  restriction_.begin_time_ = begin_time;
}

// set the elapsed time for this restriction.
void ComplexRestrictionBuilder::set_elapsed_time(const uint64_t elapsed_time) {
  restriction_.elapsed_time_ = elapsed_time;
}
*/

// Set the via edge ids used by this complex restriction.
void ComplexRestrictionBuilder::set_via_list(
  const std::vector<GraphId>& via_list) {
  if (via_list.size() > kMaxViasPerRestriction) {
    LOG_WARN("Tried to exceed max vias per restriction: " +
             std::to_string(via_list.size()));
  } else {
    via_list_ = via_list;
  }
}

// Get the size of the complex restriction
std::size_t ComplexRestrictionBuilder::BaseSizeOf() const {
  std::size_t size = sizeof(GraphId);
  size += sizeof(GraphId);
  size += sizeof(baldr::ComplexRestriction::PackedRestriction);
  size += (via_list_.size() * sizeof(GraphId));
  return size;
}

// Get the size of the complex restriction
std::size_t ComplexRestrictionBuilder::SizeOf() const {
  std::size_t size = BaseSizeOf();

  // Add padding to get to 8 byte boundaries
  size_t n = size % 8;
  if (n != 0) {
    size += 8 - n;
  }
  return size;
}

// Output complex restriction to output stream
std::ostream& operator<<(std::ostream& os, const ComplexRestrictionBuilder& crb) {
  // Pack the name count and encoded shape size. Check against limits.
  baldr::ComplexRestriction::PackedRestriction restriction;
  uint32_t via_count = crb.via_list_.size();
  if (via_count > kMaxViasPerRestriction) {
    LOG_WARN("Exceeding max vias per restriction: " + std::to_string(via_count));
    via_count = kMaxViasPerRestriction;
  }
  restriction.via_count_ = via_count;
  restriction.type_ = crb.restriction_.type_;
  restriction.modes_ = crb.restriction_.modes_;
  /** TODO
  restriction.begin_day_ = crb.restriction_.begin_day_;
  restriction.end_day_ = crb.restriction_.end_day_;
  restriction.begin_time_ = crb.restriction_.begin_time_;
  restriction.elapsed_time_ = crb.restriction_.elapsed_time_; */

  // Write out the bytes
  os.write(reinterpret_cast<const char*>(&crb.from_id_), sizeof(GraphId));
  os.write(reinterpret_cast<const char*>(&crb.to_id_), sizeof(GraphId));
  os.write(reinterpret_cast<const char*>(&restriction),
           sizeof(baldr::ComplexRestriction::PackedRestriction));
  os.write(reinterpret_cast<const char*>(&crb.via_list_[0]),
            (via_count * sizeof(GraphId)));

  // Pad to an 8 byte boundary
  std::size_t n = (crb.BaseSizeOf() % 8);
  if (n != 0) {
    for (std::size_t i = 0; i < 8 - n; i++) {
      os << static_cast<char>(0);
    }
  }
  return os;
}

// overloaded == operator - used to ensure no dups in tiles.
bool ComplexRestrictionBuilder::operator == (const ComplexRestrictionBuilder& other) const {

  if (from_id_ != other.from_id_ || to_id_ != other.to_id_ ||
      via_list_ != other.via_list_ ||
      restriction_.type_ != other.restriction_.type_ ||
      restriction_.modes_ != other.restriction_.modes_)
      /** TODO
      restriction_.begin_day_ != other.restriction_.begin_day_ ||
      restriction_.end_day_ != other.restriction_.end_day_ ||
      restriction_.begin_time_ != other.restriction_.begin_time_ ||
      restriction_.elapsed_time_ != other.restriction_.elapsed_time_) */
    return false;

  return true;
}

}
}
