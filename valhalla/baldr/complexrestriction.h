#ifndef VALHALLA_BALDR_COMPLEXRESTRICTION_H_
#define VALHALLA_BALDR_COMPLEXRESTRICTION_H_

#include <cstdint>
#include <vector>
#include <string>
#include <ostream>
#include <iostream>

#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/json.h>
#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

constexpr size_t kMaxViasPerRestriction = 31;

struct FromGraphId {
  uint64_t level  : 3;   // Hierarchy level
  uint64_t tileid : 22;  // Tile Id within the hierarchy level
  uint64_t id     : 21;  // Id of the element within the tile
  uint64_t dow    : 7;   // day of week for this restriction
  uint64_t hrs    : 5;   // hours
  uint64_t mins   : 6;   // minutes

  // Operator not equal
  bool operator !=(const FromGraphId& from) const {
    return (level != from.level ||
            tileid != from.tileid ||
            id != from.id ||
            dow != from.dow ||
            hrs != from.hrs ||
            hrs != from.hrs);
  }

};

struct ToGraphId {
  uint64_t level  : 3;   // Hierarchy level
  uint64_t tileid : 22;  // Tile Id within the hierarchy level
  uint64_t id     : 21;  // Id of the element within the tile
  uint64_t day    : 5;   // begin day
  uint64_t month  : 4;   // begin month
  uint64_t year   : 8;   // minutes
  uint64_t spare  : 1;

  // Operator not equal
  bool operator !=(const ToGraphId& from) const {
    return (level != from.level ||
            tileid != from.tileid ||
            id != from.id ||
            day != from.day ||
            month != from.month ||
            year != from.year);
  }
};

/**
 * Information held for each complex access restriction.
 */
class ComplexRestriction {
 public:
  ComplexRestriction() = delete;

  /**
   * Constructor
   * @param   ptr  Pointer to a bit of memory that has the info for this complex restriction
   */
  ComplexRestriction(char* ptr);

  /**
   * Destructor
   */
  virtual ~ComplexRestriction();

  /**
   * Get the restriction's from graph id
   * @return  Returns the from graph id
   */
  GraphId from_graphid() const;

  /**
   * Get the restriction's to graph id
   * @return  Returns the to graph id
   */
  GraphId to_graphid() const;

  /**
   * Get the restriction's from id
   * @return  Returns the from id
   */
  FromGraphId from_id() const;

  /**
   * Get the restriction's to id
   * @return  Returns the to id
   */
  ToGraphId to_id() const;

  /**
   * Get the number of vias.
   * @return  Returns the number of vias
   */
  uint64_t via_count() const;

  /**
   * Get the via id at a index
   * @param  index  Index into the via list.
   * @return  Returns the via
   */
  GraphId GetViaId(uint8_t index) const;

  /**
   * Get the restriction type
   * @return  Returns the restriction type
   */
  RestrictionType type() const;

  /**
   * Get the modes impacted by the restriction.
   * @return  Returns the access modes
   */
  uint64_t modes() const;

  /**
   * Get a list of vias
   * @return  Returns the list of vias
   */
  const std::vector<GraphId> GetVias() const;

  /**
   * Get the size of this complex restriction (without padding).
   * @return  Returns the size in bytes of this object.
   */
  std::size_t BaseSizeOf() const;

  /**
   * Get the size of this complex restriction. Includes padding to align to
   * 8-byte boundaries.
   * @return  Returns the size in bytes of this object.
   */
  std::size_t SizeOf() const;

  struct PackedRestriction {
    uint64_t type_          :  4; // Restriction type
    uint64_t modes_         : 12; // Mode(s) this access restriction applies to
    uint64_t via_count_     :  5; // size of via list.
    uint64_t has_dt_        :  1; // bit indicating if we have dt time information
    uint64_t end_time       : 11; // end time of restriction
    uint64_t end_days       : 16; // end date.  Days from begin date.  max 65535 days
    uint64_t spare_         : 15; //
  };

 protected:

  FromGraphId from_id_;           // from edge id
  ToGraphId to_id_;               // to edge id

  // Where we keep most of the static data
  PackedRestriction* restriction_;

  // List of vias
  GraphId* via_list_;             // via edge ids

};

}
}

#endif  // VALHALLA_BALDR_COMPLEXRESTRICTION_H_
