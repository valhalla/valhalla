#include "mjolnir/osmadmin.h"
#include "mjolnir/util.h"

#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

OSMAdmin::OSMAdmin()
  : memberid_index_(0), memberid_count_(0), admin_level_(0),
    osmrelationid_(std::numeric_limits<uint64_t>::max()) {

  name_= "";
}

OSMAdmin::OSMAdmin(uint64_t id)
  : memberid_index_(0), memberid_count_(0), admin_level_(0) {

  osmrelationid_ = id;
  name_= "";
}

OSMAdmin::~OSMAdmin() {
}

// Set admin id.
void OSMAdmin::set_admin_id(const uint64_t id) {
  osmrelationid_ = id;
}

// Get the admin id
uint64_t OSMAdmin::admin_id() const {
  return osmrelationid_;
}

// Set the index into the member id
void OSMAdmin::set_member_index(const uint32_t idx) {
  memberid_index_ = idx;
}

// Get the index into the member ids
uint32_t OSMAdmin::member_index() const {
  return memberid_index_;
}

// Set the number of members/ways for this admin.
void OSMAdmin::set_member_count(const uint32_t count) {
  memberid_count_ = count;
}

// Get the number of members/ways for this admin.
uint32_t OSMAdmin::member_count() const {
  return memberid_count_;
}

// Sets the name
void OSMAdmin::set_name(const std::string& name) {
  name_ = name;
}

// Get the name.
const std::string& OSMAdmin::name() const {
  return name_;
}

// Set admin level.
void OSMAdmin::set_admin_level(const uint32_t level) {
  admin_level_ = level;
}

// Get the admin level
uint32_t OSMAdmin::admin_level() const {
  return admin_level_;
}

}
}
