#include "mjolnir/osmadmin.h"
#include "mjolnir/util.h"

#include "midgard/logging.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Set admin id.
void OSMAdmin::set_admin_id(const uint64_t id) {
  osmrelationid_ = id;
}

// Get the admin id
uint64_t OSMAdmin::admin_id() const {
  return osmrelationid_;
}

// Set the ways list.
void OSMAdmin::set_ways(const std::list<uint64_t> ways) {
  ways_ = ways;
}

// Get the ways for this admin.
std::list<uint64_t> OSMAdmin::ways() const {
  return ways_;
}

// Set the number of members/ways for this admin.
void OSMAdmin::set_member_count(const uint32_t count) {
  memberid_count_ = count;
}

// Get the number of members/ways for this admin.
uint32_t OSMAdmin::member_count() const {
  return memberid_count_;
}

// Set the index for the name.
void OSMAdmin::set_name_index(const uint32_t idx) {
  name_index_ = idx;
}

// Get the name.
uint32_t OSMAdmin::name_index() const {
  return name_index_;
}

// Set the index for the name:en.
void OSMAdmin::set_name_en_index(const uint32_t idx) {
  name_en_index_ = idx;
}

// Get the name:en.
uint32_t OSMAdmin::name_en_index() const {
  return name_en_index_;
}

// Set the index for the iso coce.
void OSMAdmin::set_iso_code_index(const uint32_t idx) {
  iso_code_index_ = idx;
}

// Get the iso code.
uint32_t OSMAdmin::iso_code_index() const {
  return iso_code_index_;
}

// Set admin level.
void OSMAdmin::set_admin_level(const uint32_t level) {
  admin_level_ = level;
}

// Get the admin level
uint32_t OSMAdmin::admin_level() const {
  return admin_level_;
}

// Set drive on right.
void OSMAdmin::set_drive_on_right(const bool drive_on_right) {
  drive_on_right_ = drive_on_right;
}

// Get the drive on right flag.
bool OSMAdmin::drive_on_right() const {
  return drive_on_right_;
}

}
}
