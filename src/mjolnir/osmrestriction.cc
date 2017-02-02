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

// Set the day on
void OSMRestriction::set_day_on(DOW dow) {
  attributes_.day_on_ = static_cast<uint32_t>(dow);
}

// Get the day on
DOW OSMRestriction::day_on() const {
  return static_cast<DOW>(attributes_.day_on_);
}

// Set the day off
void OSMRestriction::set_day_off(DOW dow) {
  attributes_.day_off_ = static_cast<uint32_t>(dow);
}

// Get the day off
DOW OSMRestriction::day_off() const {
  return static_cast<DOW>(attributes_.day_off_);
}

// Set the hour on
void OSMRestriction::set_hour_on(uint32_t hour_on) {
  attributes_.hour_on_ = hour_on;
}

// Get the hour on
uint32_t OSMRestriction::hour_on() const {
  return attributes_.hour_on_;
}

// Set the minute on
void OSMRestriction::set_minute_on(uint32_t minute_on) {
  attributes_.minute_on_ = minute_on;
}

// Get the minute on
uint32_t OSMRestriction::minute_on() const {
  return attributes_.minute_on_;
}

// Set the hour off
void OSMRestriction::set_hour_off(uint32_t hour_off) {
  attributes_.hour_off_ = hour_off;
}

// Get the hour off
uint32_t OSMRestriction::hour_off() const {
  return attributes_.hour_off_;
}

// Set the minute off
void OSMRestriction::set_minute_off(uint32_t minute_off) {
  attributes_.minute_off_ = minute_off;
}

// Get the minute off
uint32_t OSMRestriction::minute_off() const {
  return attributes_.minute_off_;
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

  for (unsigned i=0; i < vias.size(); ++i) {
    vias_[i] = vias[i];
  }
}

// Get the vias
std::vector<uint64_t> OSMRestriction::vias() const {

  std::vector<uint64_t> vias;
  std::vector<uint64_t> tmp(std::begin(vias_), std::end(vias_));
  for (const auto& v : tmp)
    if (v != 0)
      vias.push_back(v);

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
  modes_ = modes;
}

// Get the modes
uint32_t OSMRestriction::modes() const {
  return modes_;
}

// Set the from way id
void OSMRestriction::set_from(uint64_t from) {
  from_ = from;
}

// Get the from way id
 uint64_t OSMRestriction::from() const {
   return from_;
 }

// Set the to way id
void OSMRestriction::set_to(uint64_t to) {
  to_ = to;
}

// Get the to way id
 uint64_t OSMRestriction::to() const {
   return to_;
 }

}
}
