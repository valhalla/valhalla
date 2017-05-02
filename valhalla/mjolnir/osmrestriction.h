#ifndef VALHALLA_MJOLNIR_OSMRESTRICTION_H
#define VALHALLA_MJOLNIR_OSMRESTRICTION_H

#include <cstdint>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/complexrestriction.h>

#include <vector>

namespace valhalla {
namespace mjolnir {

/**
 * OSM restriction information. Result of parsing OSM simple restrictions
 * found in the relations. Restrictions are stored in a multimap keyed by
 * the Id of the "from" way of the restriction.
 */
struct OSMRestriction {
  /**
   * Set the restriction type
   */
  void set_type(baldr::RestrictionType type);

  /**
   * Get the restriction type
   */
  baldr::RestrictionType type() const;

  /**
   * Set the day on
   */
  void set_day_on(baldr::DOW dow);

  /**
   * Get the day on
   */
  baldr::DOW day_on() const;

  /**
   * Set the day off
   */
  void set_day_off(baldr::DOW dow);

  /**
   * Get the day off
   */
  baldr::DOW day_off() const;

  /**
   * Set the hour on
   */
  void set_hour_on(uint32_t hour_on);

  /**
   * Get the hour on
   */
  uint32_t hour_on() const;

  /**
   * Set the minute on
   */
  void set_minute_on(uint32_t minute_on);

  /**
   * Get the minute on
   */
  uint32_t minute_on() const;

  /**
   * Set the hour off
   */
  void set_hour_off(uint32_t hour_off);

  /**
   * Get the hour off
   */
  uint32_t hour_off() const;

  /**
   * Set the minute off
   */
  void set_minute_off(uint32_t minute_off);

  /**
   * Get the minute off
   */
  uint32_t minute_off() const;

  /**
   * Set the via OSM node id
   */
  void set_via(uint64_t via);

  /**
   * Get the via OSM node id.
   */
  uint64_t via() const;

  /**
   * Set the vias -- used for complex the restrictions.
   */
  void set_vias(const std::vector<uint64_t>& vias);

  /**
   * Get the vias -- used for complex the restrictions.
   */
  std::vector<uint64_t> vias() const;

  /**
   * Set the via node GraphId.
   */
  void set_via(const baldr::GraphId& id);

  /**
   * Get the via GraphId
   */
  const baldr::GraphId& via_graphid() const;

  /**
   * Set the modes
   */
  void set_modes(uint32_t modes);

  /**
   * Get the modes
   */
  uint32_t modes() const;

  /**
   * Set the from way id
   */
  void set_from(uint64_t from);

  /**
   * Get the from way id
   */
  uint64_t from() const;

  /**
   * Set the to way id
   */
  void set_to(uint64_t to);

  /**
   * Get the to way id
   */
  uint64_t to() const;

  /**
   * overloaded < operator - used to sort
   */
  bool operator < (const OSMRestriction& o)const{
    if (from() == o.from()) {
      if (to() == o.to()) {
        if (std::memcmp(vias_, o.vias_, sizeof(vias_)) == 0) {
          if (modes() == o.modes()) {
            if (day_on() == o.day_on()){
              if (day_off() == o.day_off()){
                if (hour_on() == o.hour_on()){
                  if (hour_off() == o.hour_off()){
                    if (minute_on() == o.minute_on()){
                      return (minute_off() < o.minute_off());
                    } else return minute_on() < o.minute_on();
                  } else return hour_off() < o.hour_off();
                } else return hour_on() < o.hour_on();
              } else return day_off() < o.day_off();
            } else return day_on() < o.day_on();
          } else return modes() < o.modes();
        } else return vias() < o.vias();
      } else return to() < o.to();
    } else return from() < o.from();
  }

  /**
   * overloaded == operator - used to compare complex restrictions
   */
  bool operator == (const OSMRestriction& o) const {
    return (from() == o.from() && to() == o.to() &&
        std::memcmp(vias_, o.vias_, sizeof(vias_)) == 0 &&
        modes() == o.modes() && day_on() == o.day_on() &&
        day_off() == o.day_off() &&
        hour_on() == o.hour_on() && hour_off() == o.hour_off() &&
        minute_on() == o.minute_on());
  }

  // from is a way - uses OSM way Id.
  uint64_t from_;

  // Via is a node. When parsing OSM this is stored as an OSM node Id.
  // It later gets changed into a GraphId.
  union ViaNode {
    ViaNode() {}
    baldr::GraphId id;
    uint64_t osmid;
  };
  ViaNode via_;

  // fixed size of vias.
  uint64_t vias_[valhalla::baldr::kMaxViasPerRestriction];

  // to is a way - uses OSM way Id.
  uint64_t to_;

  // Type and time information of the restriction.
  struct Attributes {
    uint32_t type_        : 4;
    uint32_t day_on_      : 3;
    uint32_t day_off_     : 3;
    uint32_t hour_on_     : 5;
    uint32_t minute_on_   : 6;
    uint32_t hour_off_    : 5;
    uint32_t minute_off_  : 6;
  };
  Attributes attributes_;

  // access modes -- who does this restriction apply to?  cars, bus, etc.
  uint32_t modes_;

};

}
}

#endif  // VALHALLA_MJOLNIR_OSMRESTRICTION_H
