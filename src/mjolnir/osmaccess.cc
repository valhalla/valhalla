#include "mjolnir/osmaccess.h"
#include "mjolnir/util.h"

#include <iostream>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Set way id.
void OSMAccess::set_way_id(const uint64_t id) {
  osmwayid_ = id;
}

// Get the way id
uint64_t OSMAccess::way_id() const {
  return osmwayid_;
}

// Set auto tag flag.
void OSMAccess::set_auto_tag(const bool auto_tag) {
  attributes_.fields.auto_tag = auto_tag;
}
// Get auto tag flag.
bool OSMAccess::auto_tag() const {
  return attributes_.fields.auto_tag;
}

// Set bike tag flag.
void OSMAccess::set_bike_tag(const bool bike_tag) {
  attributes_.fields.bike_tag = bike_tag;
}
// Get bike tag flag.
bool OSMAccess::bike_tag() const {
  return attributes_.fields.bike_tag;
}

// Set bus tag flag.
void OSMAccess::set_bus_tag(const bool bus_tag) {
  attributes_.fields.bus_tag = bus_tag;
}
// Get bus tag flag.
bool OSMAccess::bus_tag() const {
  return attributes_.fields.bus_tag;
}

// Set emergency tag flag.
void OSMAccess::set_emergency_tag(const bool emergency_tag) {
  attributes_.fields.emergency_tag = emergency_tag;
}
// Get emergency tag flag.
bool OSMAccess::emergency_tag() const {
  return attributes_.fields.emergency_tag;
}

// Set foot tag flag.
void OSMAccess::set_foot_tag(const bool foot_tag) {
  attributes_.fields.foot_tag = foot_tag;
}
// Get foot tag flag.
bool OSMAccess::foot_tag() const {
  return attributes_.fields.foot_tag;
}

// Set truck tag flag.
void OSMAccess::set_truck_tag(const bool truck_tag) {
  attributes_.fields.truck_tag = truck_tag;
}
// Get truck tag flag.
bool OSMAccess::truck_tag() const {
  return attributes_.fields.truck_tag;
}

}
}
