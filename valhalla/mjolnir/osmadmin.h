#ifndef VALHALLA_MJOLNIR_OSMADMIN_H
#define VALHALLA_MJOLNIR_OSMADMIN_H

#include <cstdint>
#include <list>
#include <string>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/pointll.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace mjolnir {

constexpr uint32_t kMaxMembersPerAdmin = 65535;

// OSM Admin
struct OSMAdmin {

  /**
   * Set admin id.
   * @param   id  admin id
   */
  void set_admin_id(const uint64_t id) {
    osmrelationid_ = id;
  }

  /**
   * Get the admin id
   * @return  Returns admin id.
   */
  uint64_t admin_id() const {
    return osmrelationid_;
  }

  /**
   * Set the ways list.
   * @param  ways   Ways for this admin.
   */
  void set_ways(const std::list<uint64_t>& ways) {
    ways_ = ways;
  }

  /**
   * Get the ways for this admin.
   * @return  Returns the ways for this admin
   */
  std::list<uint64_t> ways() const {
    return ways_;
  }

  /**
   * Set the number of members/ways for this admin.
   * @param count  Number of ways for this admin.
   */
  void set_member_count(const uint32_t count) {
    memberid_count_ = count;
  }

  /**
   * Get the number of members/ways for this admin.
   */
  uint32_t member_count() const {
    return memberid_count_;
  }

  /**
   * Sets the index for name
   * @param  idx  Index for the name.
   */
  void set_name_index(const uint32_t idx) {
    name_index_ = idx;
  }

  /**
   * Get the name index.
   * @return  Returns the index for the name.
   */
  uint32_t name_index() const {
    return name_index_;
  }

  /**
   * Sets the index for name:en
   * @param  idx  Index for the English name.
   */
  void set_name_en_index(const uint32_t idx) {
    name_en_index_ = idx;
  }

  /**
   * Get the name:en index.
   * @return  Returns the index for the English name.
   */
  uint32_t name_en_index() const {
    return name_en_index_;
  }

  /**
   * Sets the index for iso code
   * @param  idx  Index for the iso code.
   */
  void set_iso_code_index(const uint32_t idx) {
    iso_code_index_ = idx;
  }

  /**
   * Get the iso code index.
   * @return  Returns the index for the iso code.
   */
  uint32_t iso_code_index() const {
    return iso_code_index_;
  }

  /**
   * Set admin level.
   * @param   level  admin level
   */
  void set_admin_level(const uint32_t level) {
    admin_level_ = level;
  }

  /**
   * Get the admin level
   * @return  Returns admin level.
   */
  uint32_t admin_level() const {
    return admin_level_;
  }

  /**
   * Set drive on right.
   * @param   drive_on_right
   */
  void set_drive_on_right(const bool drive_on_right) {
    drive_on_right_ = drive_on_right;
  }

  /**
   * Get the drive on right flag.
   * @return  Drive on right?
   */
  bool drive_on_right() const {
    return drive_on_right_;
  }

  // OSM admin/relation id
  uint64_t osmrelationid_;

  // List of ways/member ids
  std::list<uint64_t> ways_;

  // Names of country or state/prov
  uint32_t name_index_;
  uint32_t name_en_index_;

  // ISO code
  uint32_t iso_code_index_;

  // Count of members.
  uint16_t memberid_count_;

  // Admin level.  2 = country; 4 = state.
  uint8_t admin_level_;

  // drive on right side of the road in this country?
  bool drive_on_right_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMADMIN_H
