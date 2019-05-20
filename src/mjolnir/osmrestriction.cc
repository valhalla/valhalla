#include "mjolnir/osmrestriction.h"
#include "midgard/logging.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Set the restriction type
void OSMRestriction::set_type(RestrictionType type) {
  attributes_.type_ = static_cast<uint32_t>(type);
}

// Get the restriction type
RestrictionType OSMRestriction::type() const {
  return static_cast<RestrictionType>(attributes_.type_);
}

// Set the via id
void OSMRestriction::set_via(uint64_t via) {
  via_.osmid = via;
}

// Get the via OSM node id
uint64_t OSMRestriction::via() const {
  return via_.osmid;
}

// Set the vias
void OSMRestriction::set_vias(const std::vector<uint64_t>& vias) {

  if (vias.size() > kMaxViasPerRestriction) {
    LOG_INFO("skipping restriction with vias > the max allowed.");
    return;
  }

  for (unsigned i = 0; i < vias.size(); ++i) {
    vias_[i] = vias[i];
  }
}

// Get the vias
std::vector<uint64_t> OSMRestriction::vias() const {

  std::vector<uint64_t> vias;
  std::vector<uint64_t> tmp(std::begin(vias_), std::end(vias_));
  for (const auto& v : tmp) {
    if (v != 0) {
      vias.push_back(v);
    }
  }

  return vias;
}

// Set the via id
void OSMRestriction::set_via(const GraphId& id) {
  via_.id = id;
}

// Get the via node's GraphId
const GraphId& OSMRestriction::via_graphid() const {
  return via_.id;
}

// Set the modes
void OSMRestriction::set_modes(uint32_t modes) {
  attributes_.modes_ = modes;
}

// Get the modes
uint32_t OSMRestriction::modes() const {
  return attributes_.modes_;
}

// Set the from way id
void OSMRestriction::set_from(uint32_t from) {
  from_ = from;
}

// Get the from way id
uint32_t OSMRestriction::from() const {
  return from_;
}

// Set the to way id
void OSMRestriction::set_to(uint32_t to) {
  to_ = to;
}

// Get the to way id
uint32_t OSMRestriction::to() const {
  return to_;
}

// Set the time domain
void OSMRestriction::set_time_domain(uint64_t time_domain) {
  time_domain_ = time_domain;
}

// Get the time domain
uint64_t OSMRestriction::time_domain() const {
  return time_domain_;
}

} // namespace mjolnir
} // namespace valhalla
