#ifndef VALHALLA_MJOLNIR_OSMADMIN_H
#define VALHALLA_MJOLNIR_OSMADMIN_H

#include <cstdint>
#include <string>
#include <list>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphconstants.h>

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
  void set_admin_id(const uint64_t id);

  /**
   * Get the admin id
   * @return  Returns admin id.
   */
  uint64_t admin_id() const;

  /**
   * Set the ways list.
   * @param  ways   Ways for this admin.
   */
  void set_ways(const std::list<uint64_t> ways);

  /**
   * Get the ways for this admin.
   * @return  Returns the ways for this admin
   */
  std::list<uint64_t> ways() const;

  /**
   * Set the number of members/ways for this admin.
   * @param count  Number of ways for this admin.
   */
  void set_member_count(const uint32_t count);

  /**
   * Get the number of members/ways for this admin.
   */
  uint32_t member_count() const;

  /**
   * Sets the name
   * @param  name   Name.
   */
  void set_name(const std::string& name);

  /**
   * Get the name.
   * @return  Returns name.
   */
  const std::string& name() const;

  /**
   * Set admin level.
   * @param   level  admin level
   */
  void set_admin_level(const uint32_t level);

  /**
   * Get the admin level
   * @return  Returns admin level.
   */
  uint32_t admin_level() const;

  // OSM admin/relation id
  uint64_t osmrelationid_;

  // List of ways/member ids
  std::list<uint64_t> ways_;

  // Count of members.
  uint16_t memberid_count_;

  // Admin level.  2 = country; 4 = state.
  uint8_t admin_level_;

  // Name of country or state/prov
  std::string name_;

};

}
}

#endif  // VALHALLA_MJOLNIR_OSMADMIN_H
