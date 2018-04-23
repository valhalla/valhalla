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
void ComplexRestrictionBuilder::set_from_id(const FromGraphId from_id) {
  from_id_ = from_id;
}

// Set the to edge id.
void ComplexRestrictionBuilder::set_to_id(const ToGraphId to_id) {
  to_id_ = to_id;
}

// Set the from edge graph id.
void ComplexRestrictionBuilder::set_from_id(const GraphId from_id) {
  from_id_.level = from_id.level();
  from_id_.tileid = from_id.tileid();
  from_id_.id = from_id.id();
}

// Set the to edge graph id.
void ComplexRestrictionBuilder::set_to_id(const GraphId to_id) {
  to_id_.level = to_id.level();
  to_id_.tileid = to_id.tileid();
  to_id_.id = to_id.id();
}

// set the restriction type.
void ComplexRestrictionBuilder::set_type(const RestrictionType type) {
  restriction_.type_ = (static_cast<uint64_t>(type));
}

// set the access modes for the restriction
void ComplexRestrictionBuilder::set_modes(const uint64_t modes) {
  restriction_.modes_ = modes;
}

// set the date time flag for the restriction
void ComplexRestrictionBuilder::set_dt(const bool dt) {
  from_id_.has_dt = dt;
}

// set the begin day or dow for the restriction.
void ComplexRestrictionBuilder::set_begin_day_dow(const uint64_t begin_day_dow) {
  from_id_.begin_day_dow = begin_day_dow;
}

// set the begin month for the restriction.
void ComplexRestrictionBuilder::set_begin_month(const uint64_t begin_month) {
  from_id_.begin_month = begin_month;
}

// set the begin week for the restriction.
void ComplexRestrictionBuilder::set_begin_week(const uint64_t begin_week) {
  from_id_.begin_week = begin_week;
}

// set the begin hours for the restriction.
void ComplexRestrictionBuilder::set_begin_hrs(const uint64_t begin_hrs) {
  from_id_.begin_hrs = begin_hrs;
}

// set the date time type for the restriction
void ComplexRestrictionBuilder::set_dt_type(const bool type) {
  to_id_.dt_type = type;
}

// set the end day or dow for the restriction.
void ComplexRestrictionBuilder::set_end_day_dow(const uint64_t end_day_dow) {
  to_id_.end_day_dow = end_day_dow;
}

// set the end month for the restriction.
void ComplexRestrictionBuilder::set_end_month(const uint64_t end_month) {
  to_id_.end_month = end_month;
}

// set the end week for the restriction.
void ComplexRestrictionBuilder::set_end_week(const uint64_t end_week) {
  to_id_.end_week = end_week;
}

// set the end hours for the restriction.
void ComplexRestrictionBuilder::set_end_hrs(const uint64_t end_hrs) {
  to_id_.end_hrs = end_hrs;
}

// set the dow mask.  indicates days of week to apply the restriction
void ComplexRestrictionBuilder::set_dow(const uint64_t dow) {
  restriction_.dow = dow;
}

// set the begin minutes for the restriction.
void ComplexRestrictionBuilder::set_begin_mins(const uint64_t begin_mins) {
  restriction_.begin_mins = begin_mins;
}

// set the end minutes for the restriction.
void ComplexRestrictionBuilder::set_end_mins(const uint64_t end_mins) {
  restriction_.end_mins = end_mins;
}

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
  std::size_t size = sizeof(FromGraphId);
  size += sizeof(ToGraphId);
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

  restriction.dow = crb.restriction_.dow;
  restriction.begin_mins = crb.restriction_.begin_mins;
  restriction.end_mins = crb.restriction_.end_mins;

  if (1304764469345 == GraphId(crb.from_id_.tileid,crb.from_id_.level, crb.from_id_.id).value) {

  std::cout << crb.to_id_.dt_type << " " <<  restriction.dow   <<  " "
  << crb.from_id_.begin_month  <<  " "  <<  crb.from_id_.begin_day_dow   <<  " "
  << crb.from_id_.begin_week  <<  " "  <<  crb.from_id_.begin_hrs   <<  " "
  << restriction.begin_mins  <<  " "  <<  crb.to_id_.end_month   <<  " "
  << crb.to_id_.end_day_dow  <<  " "  <<  crb.to_id_.end_week   <<  " "
  << crb.to_id_.end_hrs  <<  " " <<  restriction.end_mins << std::endl;
  }

  // Write out the bytes
  os.write(reinterpret_cast<const char*>(&crb.from_id_), sizeof(FromGraphId));
  os.write(reinterpret_cast<const char*>(&crb.to_id_), sizeof(ToGraphId));
  os.write(reinterpret_cast<const char*>(&restriction),
           sizeof(baldr::ComplexRestriction::PackedRestriction));
  os.write(reinterpret_cast<const char*>(crb.via_list_.data()),
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
      restriction_.modes_ != other.restriction_.modes_ ||
      restriction_.dow != other.restriction_.dow ||
      restriction_.begin_mins != other.restriction_.begin_mins ||
      restriction_.end_mins != other.restriction_.end_mins)
    return false;

  return true;
}

}
}
