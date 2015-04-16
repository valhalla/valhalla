#ifndef VALHALLA_MJOLNIR_OSMRESTRICTION_H
#define VALHALLA_MJOLNIR_OSMRESTRICTION_H

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace mjolnir {

/**
 * OSM restriction information. Result of parsing OSM simple restrictions
 * found in the relations. Restrictions are stored in a multimap keyed by
 * the Id of the "from" way of the restriction.
 */
class OSMRestriction {
 public:
  /**
   * Constructor
   */
  OSMRestriction();

  /**
   * Destructor.
   */
  ~OSMRestriction();

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
   * Set the via node GraphId.
   */
  void set_via(const baldr::GraphId& id);

  /**
   * Get the via OSM node id.
   */
  uint64_t via() const;

  /**
   * Get the via GraphId
   */
  const baldr::GraphId& via_graphid() const;

  /**
   * Set the to way id
   */
  void set_to(uint64_t to);

  /**
   * Get the to way id
   */
  uint64_t to() const;

 protected:
  // Via is a node. When parsing OSM this is stored as an OSM node Id.
  // It later gets changed into a GraphId.
  union ViaNode {
    baldr::GraphId id;
    uint64_t osmid;
  };
  ViaNode via_;

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
};

}
}

#endif  // VALHALLA_MJOLNIR_OSMRESTRICTION_H
