#ifndef VALHALLA_MJOLNIR_COMPLEXRESTRICTIONBUILDER_H_
#define VALHALLA_MJOLNIR_COMPLEXRESTRICTIONBUILDER_H_

#include <cstdint>
#include <vector>
#include <list>
#include <string>
#include <iostream>

#include <valhalla/baldr/complexrestriction.h>
#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

class ComplexRestrictionBuilder {
 public:

  /**
   * Set the from edge id.
   * @param  from_id  from id.
   *
   */
  void set_from_id(const FromGraphId from_id);

  /**
   * Set the to edge id.
   * @param  to_id  to id.
   *
   */
  void set_to_id(const ToGraphId to_id);

  /**
   * Set the from edge graph id.
   * @param  from_id  from graph id.
   *
   */
  void set_from_id(const GraphId from_id);

  /**
   * Set the to edge graph id.
   * @param  to_id  to graph id.
   *
   */
  void set_to_id(const GraphId to_id);

  /**
   * set the vias for this restriction
   * @param  via_list  via list.
   *
   */
  void set_via_list(const std::vector<GraphId>& via_list);

  /**
   * set the restriction type.
   * @param  type  restriction type.
   *
   */
  void set_type(const RestrictionType type);

  /**
   * set the access modes for the restriction
   * @param  modes  access modes - mask (auto, bus, truck, etc.).
   *
   */
  void set_modes(const uint64_t modes);

  /**
   * set the date time flag for the restriction
   * @param  dt  date time bit indicating if there is date time
   *             info for this restriction.
   *
   */
  void set_dt(const bool dt);

  /**
   * set the begin day or dow for the restriction.
   * @param  begin_day_dow  begin day or dow for this restriction.
   *
   */
  void set_begin_day_dow(const uint64_t begin_day_dow);

  /**
   * set the begin month for the restriction.
   * @param  begin_month  begin month for this restriction.
   *
   */
  void set_begin_month(const uint64_t begin_month);

  /**
   * set the begin week for the restriction.
   * @param  begin_week  begin week for this restriction.
   *
   */
  void set_begin_week(const uint64_t begin_week);

  /**
   * set the begin hours for the restriction.
   * @param  begin_hrs  begin hours for this restriction.
   *
   */
  void set_begin_hrs(const uint64_t begin_hrs);

  /**
   * set the type for the restriction
   * @param  type   the type of date time restriction.
   *                YMD = 0 or nth dow = 1
   *
   */
  void set_dt_type(const bool type);

  /**
   * set the end day or dow for the restriction.
   * @param  end_day_dow  end day or dow for this restriction.
   *
   */
  void set_end_day_dow(const uint64_t end_day_dow);

  /**
   * set the end month for the restriction.
   * @param  end_month  end month for this restriction.
   *
   */
  void set_end_month(const uint64_t end_month);

  /**
   * set the end week for the restriction.
   * @param  end_week  end week for this restriction.
   *
   */
  void set_end_week(const uint64_t end_week);

  /**
   * set the end hours for the restriction.
   * @param  end_hrs  end hours for this restriction.
   *
   */
  void set_end_hrs(const uint64_t end_hrs);

  /**
   * set the dow mask.  indicates days of week to apply the restriction
   * @param  dow  day of week - This is a mask (e.g., Mo-Fr = 62).
   *
   */
  void set_dow(const uint64_t dow);

  /**
   * set the begin minutes for the restriction.
   * @param  begin_mins  begin minutes for this restriction.
   *
   */
  void set_begin_mins(const uint64_t begin_mins);

  /**
   * set the end minutes for the restriction.
   * @param  end_mins  end minutes for this restriction.
   *
   */
  void set_end_mins(const uint64_t begin_mins);

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

  /**
   * overloaded == operator - used to ensure no dups in tiles.
   * @param  other  ComplexRestrictionBuilder to compare to.
   * @return  Returns true or false if equal or not.
   *
   */
  bool operator == (const ComplexRestrictionBuilder& other) const;

 protected:
  //from edgeid
  FromGraphId from_id_;

  // to edgeid
  ToGraphId to_id_;

  // packed restriction data.
  ComplexRestriction::PackedRestriction restriction_;

  // via list
  std::vector<GraphId> via_list_;

  friend std::ostream& operator<<(std::ostream& os, const ComplexRestrictionBuilder& crb);
};

}
}

#endif  // VALHALLA_MJOLNIR_COMPLEXRESTRICTIONBUILDER_H_
