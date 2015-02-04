#ifndef VALHALLA_MJOLNIR_OSMRESTRICTION_H
#define VALHALLA_MJOLNIR_OSMRESTRICTION_H

#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

/**
 * OSM restriction information. Result of parsing OSM simple restrictions found in the relations.
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
  void set_type(RestrictionType type);

  /**
   * Get the restriction type
   */
  RestrictionType type() const;

  /**
   * Set the day on
   */
  void set_day_on(DOW dow);

  /**
   * Get the day on
   */
  DOW day_on() const;

  /**
   * Set the day off
   */
  void set_day_off(DOW dow);

  /**
   * Get the day off
   */
  DOW day_off() const;

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
   * Set the from way id
   */
  void set_from(uint64_t from);

  /**
   * Get the from way id
   */
  uint64_t from() const;

  /**
   * Set the via id
   */
  void set_via(uint64_t via);

  /**
   * Get the via id
   */
  uint64_t via() const;

  /**
   * Set the to way id
   */
  void set_to(uint64_t to);

  /**
   * Get the to way id
   */
  uint64_t to() const;

 protected:

  //from is a way
  uint64_t from_;

  //For now, via is of type node.
  uint64_t via_;

  //to is a way
  uint64_t to_;

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
