#include <string.h>
#include <algorithm>
#include "baldr/complexrestriction.h"


namespace valhalla {
namespace baldr {

ComplexRestriction::ComplexRestriction(char* ptr) {

  from_id_ = *(reinterpret_cast<FromGraphId*>(ptr));
  ptr += sizeof(FromGraphId);

  to_id_ = *(reinterpret_cast<ToGraphId*>(ptr));
  ptr += sizeof(ToGraphId);

  restriction_ = reinterpret_cast<PackedRestriction*>(ptr);
  ptr += sizeof(PackedRestriction);

  via_list_ = reinterpret_cast<GraphId*>(ptr);
  ptr += (via_count() * sizeof(GraphId));

}

ComplexRestriction::~ComplexRestriction() {
  //nothing to delete these are all shallow pointers for the moment held
  //by another object
}

// restriction is from this id
GraphId ComplexRestriction::from_graphid() const {
  return GraphId(from_id_.tileid, from_id_.level, from_id_.id);
}

// restriction is to this id
GraphId ComplexRestriction::to_graphid() const {
  return GraphId(to_id_.tileid, to_id_.level, to_id_.id);
}

// restriction is from this id
FromGraphId ComplexRestriction::from_id() const {
  return from_id_;
}

// restriction is to this id
ToGraphId ComplexRestriction::to_id() const {
  return to_id_;
}

// Get the number of vias.
uint64_t ComplexRestriction::via_count() const {
  return restriction_->via_count_;
}

// Get the via based on the index.
GraphId ComplexRestriction::GetViaId(uint8_t index) const {
  if(index < restriction_->via_count_)
    return via_list_[index];
  else
    throw std::runtime_error("Via Id index was out of bounds");
}

// Get the type
RestrictionType ComplexRestriction::type() const {
  return static_cast<RestrictionType>(restriction_->type_);
}

// Get the modes impacted by the restriction.
uint64_t ComplexRestriction::modes() const {
  return restriction_->modes_;
}

// Get a list of vias
const std::vector<GraphId> ComplexRestriction::GetVias() const {
  // Get each via
  std::vector<GraphId> vias;
  for (uint32_t i = 0; i < via_count(); i++) {
    GraphId id = GetViaId(i);

    if (i < kMaxViasPerRestriction) {
      vias.push_back(id);
    } else {
      throw std::runtime_error("GetVias: count exceeds max via amount per restriction.");
    }
  }
  return vias;
}

// Get the date time flag for the restriction
bool ComplexRestriction::has_dt() const {
  return from_id_.has_dt;
}

// Get the begin day or dow for the restriction.
uint64_t ComplexRestriction::begin_day_dow() const {
  return from_id_.begin_day_dow;
}

// Get the begin month for the restriction.
uint64_t ComplexRestriction::begin_month() const {
  return from_id_.begin_month;
}

// Get the begin week for the restriction.
uint64_t ComplexRestriction::begin_week() const {
  return from_id_.begin_week;
}

// Get the begin hours for the restriction.
uint64_t ComplexRestriction::begin_hrs() const {
  return from_id_.begin_hrs;
}

// Get the type for the restriction
bool ComplexRestriction::dt_type() const {
  return to_id_.dt_type;
}

// Get the end day or dow for the restriction.
uint64_t ComplexRestriction::end_day_dow() const {
  return to_id_.end_day_dow;
}

// Get the end month for the restriction.
uint64_t ComplexRestriction::end_month() const {
  return to_id_.end_month;
}

// Get the end week for the restriction.
uint64_t ComplexRestriction::end_week() const {
  return to_id_.end_week;
}

// Get the end hours for the restriction.
uint64_t ComplexRestriction::end_hrs() const {
  return to_id_.end_hrs;
}

// Get the dow mask.  indicates days of week to apply the restriction
uint64_t ComplexRestriction::dow() const {
  return restriction_->dow;
}

// Get the begin minutes for the restriction.
uint64_t ComplexRestriction::begin_mins() const {
  return restriction_->begin_mins;
}

// Get the end minutes for the restriction.
uint64_t ComplexRestriction::end_mins() const {
  return restriction_->end_mins;
}

// Get the size of the complex restriction
std::size_t ComplexRestriction::BaseSizeOf() const {
  std::size_t size = sizeof(FromGraphId);
  size += sizeof(ToGraphId);
  size += sizeof(baldr::ComplexRestriction::PackedRestriction);
  size += (restriction_->via_count_ * sizeof(GraphId));
  return size;
}

// Get the size of the complex restriction
std::size_t ComplexRestriction::SizeOf() const {
  std::size_t size = BaseSizeOf();

  // Add padding to get to 8 byte boundaries
  size_t n = size % 8;
  if (n != 0) {
    size += 8 - n;
  }
  return size;
}

}
}
