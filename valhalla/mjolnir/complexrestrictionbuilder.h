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
  void set_from_id(const GraphId from_id);

  /**
   * Set the to edge id.
   * @param  to_id  to id.
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
   * set the begin dow for this restriction
   * @param  day  begin dow.
   *
   */
//  void set_begin_day(const DOW day);

  /**
   * set the end dow for this restriction
   * @param  day  end dow.
   *
   */
//  void set_end_day(const DOW day);

  /**
   * set the begin time for this restriction
   * @param  begin_time  when does it start - sec from midnight.
   *
   */
//  void set_begin_time(const uint64_t begin_time);

  /**
   * set the elapsed time for this restriction
   * @param  elapsed_time  elapsed time in secs.
   *
   */
//  void set_elapsed_time(const uint64_t elapsed_time);

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
  GraphId from_id_;

  // to edgeid
  GraphId to_id_;

  // packed restriction data.
  ComplexRestriction::PackedRestriction restriction_;

  // via list
  std::vector<GraphId> via_list_;

  friend std::ostream& operator<<(std::ostream& os, const ComplexRestrictionBuilder& crb);
};

}
}

#endif  // VALHALLA_MJOLNIR_COMPLEXRESTRICTIONBUILDER_H_
