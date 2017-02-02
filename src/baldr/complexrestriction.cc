#include <string.h>
#include <algorithm>
#include "baldr/complexrestriction.h"


namespace valhalla {
namespace baldr {

ComplexRestriction::ComplexRestriction(char* ptr) {

  from_id_ = *(reinterpret_cast<GraphId*>(ptr));
  ptr += sizeof(GraphId);

  to_id_ = *(reinterpret_cast<GraphId*>(ptr));
  ptr += sizeof(GraphId);

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
GraphId ComplexRestriction::from_id() const {
  return from_id_;
}

// restriction is to this id
GraphId ComplexRestriction::to_id() const {
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

// Get the size of the complex restriction
std::size_t ComplexRestriction::BaseSizeOf() const {
  std::size_t size = sizeof(GraphId);
  size += sizeof(GraphId);
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
