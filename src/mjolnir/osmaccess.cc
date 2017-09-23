#include "mjolnir/osmaccess.h"
#include "mjolnir/util.h"

#include <iostream>
#include <cstring>
#include "midgard/logging.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Default constructor. Initialize to all 0's.
OSMAccess::OSMAccess() {
  osmwayid_ = 0;
  attributes_ = {0};
}

// Default constructor. Initialize to all 0's except for the way id.
OSMAccess::OSMAccess(const uint64_t id) {
  set_way_id(id);
  attributes_ = {0};
}

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

// Set moped tag flag
void OSMAccess::set_moped_tag(const bool moped_tag) {
  attributes_.fields.moped_tag = moped_tag;
}

// Get moped tag flag
bool OSMAccess::moped_tag() const {
  return attributes_.fields.moped_tag;
}

// Set bus tag flag.
void OSMAccess::set_bus_tag(const bool bus_tag) {
  attributes_.fields.bus_tag = bus_tag;
}
// Get bus tag flag.
bool OSMAccess::bus_tag() const {
  return attributes_.fields.bus_tag;
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

// Set hov tag flag.
void OSMAccess::set_hov_tag(const bool hov_tag) {
  attributes_.fields.hov_tag = hov_tag;
}
// Get hov tag flag.
bool OSMAccess::hov_tag() const {
  return attributes_.fields.hov_tag;
}

// Set motorroad tag flag.
void OSMAccess::set_motorroad_tag(const bool motorroad_tag) {
  attributes_.fields.motorroad_tag = motorroad_tag;
}
// Get motorroad tag flag.
bool OSMAccess::motorroad_tag() const {
  return attributes_.fields.motorroad_tag;
}

}
}
