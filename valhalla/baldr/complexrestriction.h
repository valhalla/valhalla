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
  uint64_t level            : 3;   // Hierarchy level
  uint64_t tileid           : 22;  // Tile Id within the hierarchy level
  uint64_t id               : 21;  // Id of the element within the tile
  uint64_t has_dt           : 1;   // bit indicating if we have dt time information
  uint64_t begin_day_dow    : 5;   // begin day or dow enum i.e. 1st Sunday
  uint64_t begin_month      : 4;   // begin month
  uint64_t begin_week       : 3;   // which begin week does this start.  i.e. 1st week in Oct
  uint64_t begin_hrs        : 5;   // begin hours

  // Operator not equal
  bool operator !=(const FromGraphId& from) const {
    return (level != from.level ||
            tileid != from.tileid ||
            id != from.id ||
            has_dt != from.has_dt ||
            begin_day_dow != from.begin_day_dow ||
            begin_month != from.begin_month ||
            begin_week != from.begin_week ||
            begin_hrs != from.begin_hrs);
  }

};

struct ToGraphId {
  uint64_t level          : 3;   // Hierarchy level
  uint64_t tileid         : 22;  // Tile Id within the hierarchy level
  uint64_t id             : 21;  // Id of the element within the tile
  uint64_t dt_type        : 1;   // type of date time restriction: YMD = 0 or nth dow = 1
  uint64_t end_day_dow    : 5;   // end day or dow enum i.e. last Sunday
  uint64_t end_month      : 4;   // end month
  uint64_t end_week       : 3;   // which end week does this end.  i.e. last week in Oct
  uint64_t end_hrs        : 5;   // end hours

  // Operator not equal
  bool operator !=(const ToGraphId& to) const {
    return (level != to.level ||
            tileid != to.tileid ||
            id != to.id ||
            dt_type != to.dt_type ||
            end_day_dow != to.end_day_dow ||
            end_month != to.end_month ||
            end_week != to.end_week ||
            end_hrs != to.end_hrs);
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
   * FromGraphId contains the graphid info
   * and the date time data.
   * @return  Returns the from id
   */
  FromGraphId from_id() const;

  /**
   * Get the restriction's to id
   * ToGraphId contains the graphid info
   * and the date time data.
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
   * Get the date time flag for the restriction
   * @param  dt  date time bit indicating if there is date time
   *             info for this restriction.
   *
   */
  bool has_dt() const;

  /**
   * Get the begin day or dow for the restriction.
   * @return  Returns the begin day or dow for this restriction.
   */
  uint64_t begin_day_dow() const;

  /**
   * Get the begin month for the restriction.
   * @return  Returns the begin month for this restriction.
   */
  uint64_t begin_month() const;

  /**
   * Get the begin week for the restriction.
   * @return  Returns the begin week for this restriction.
   */
  uint64_t begin_week() const;

  /**
   * Get the begin hours for the restriction.
   * @return  Returns the begin hours for this restriction.
   */
  uint64_t begin_hrs() const;

  /**
   * Get the type for the restriction
   * @return  Returns the the type of date time restriction.
   *          YMD = 0 or nth dow = 1
   */
  bool dt_type() const;

  /**
   * Get the end day or dow for the restriction.
   * @return  Returns the end day or dow for this restriction.
   */
  uint64_t end_day_dow() const;

  /**
   * Get the end month for the restriction.
   * @return  Returns the end month for this restriction.
   */
  uint64_t end_month() const;

  /**
   * Get the end week for the restriction.
   * @return  Returns the end week for this restriction.
   */
  uint64_t end_week() const;

  /**
   * Get the end hours for the restriction.
   * @return  Returns the end hours for this restriction.
   */
  uint64_t end_hrs() const;

  /**
   * Get the dow mask.  indicates days of week to apply the restriction
   * @return  Returns the day of week - This is a mask (e.g., Mo-Fr = 62).
   */
  uint64_t dow() const;

  /**
   * Get the begin minutes for the restriction.
   * @return  Returns the begin minutes for this restriction.
   */
  uint64_t begin_mins() const;

  /**
   * Get the end minutes for the restriction.
   * @return  Returns the end minutes for this restriction.
   */
  uint64_t end_mins() const;

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
    uint64_t type_          :  4;  // Restriction type
    uint64_t modes_         : 12;  // Mode(s) this access restriction applies to
    uint64_t via_count_     :  5;  // size of via list.
    uint64_t dow            :  7;  // day of week for this restriction
    uint64_t begin_mins     :  6;  // begin minutes
    uint64_t end_mins       :  6;  // end minutes
    uint64_t spare_         : 24;
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
