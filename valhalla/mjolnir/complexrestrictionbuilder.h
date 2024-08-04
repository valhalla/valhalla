#ifndef VALHALLA_MJOLNIR_COMPLEXRESTRICTIONBUILDER_H_
#define VALHALLA_MJOLNIR_COMPLEXRESTRICTIONBUILDER_H_

#include <cstdint>
#include <vector>

#include <valhalla/baldr/complexrestriction.h>
#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

/**
 * Class to build complex restrictions. Derived from ComplexRestriction.
 * Adds methods to set fields of the structure. Contains a vector of
 * via edge GraphIds. Includes a method to serialize the structure.
 */
class ComplexRestrictionBuilder : public baldr::ComplexRestriction {
public:
  ComplexRestrictionBuilder() = default;

  ComplexRestrictionBuilder(const ComplexRestriction& restriction) : ComplexRestriction(restriction) {
  }

  /**
   * Set the from edge graph id.
   * @param  from_id  from graph id.
   */
  void set_from_id(const GraphId& from_id) {
    from_graphid_ = from_id.value;
  }

  /**
   * Set the to edge graph id.
   * @param  to_id  to graph id.
   */
  void set_to_id(const GraphId& to_id) {
    to_graphid_ = to_id.value;
  }

  /**
   * Set the vias for this restriction
   * @param  via_list  via list.
   */
  void set_via_list(const std::vector<GraphId>& via_list);

  /**
   * Set the restriction type.
   * @param  type  restriction type.
   */
  void set_type(const RestrictionType type) {
    type_ = (static_cast<uint8_t>(type));
  }

  /**
   * Set the access modes for the restriction
   * @param  modes  access modes - mask (auto, bus, truck, etc.).
   */
  void set_modes(const uint16_t modes) {
    modes_ = modes;
  }

  /**
   * Set the date time flag for the restriction
   * @param  dt  date time bit indicating if there is date time
   *             info for this restriction.
   */
  void set_dt(const bool dt) {
    has_dt_ = dt;
  }

  /**
   * Set the begin day or dow for the restriction.
   * @param  begin_day_dow  begin day or dow for this restriction.
   */
  void set_begin_day_dow(const uint8_t begin_day_dow) {
    begin_day_dow_ = begin_day_dow;
  }

  /**
   * Set the begin month for the restriction.
   * @param  begin_month  begin month for this restriction.
   */
  void set_begin_month(const uint8_t begin_month) {
    begin_month_ = begin_month;
  }

  /**
   * Set the begin week for the restriction.
   * @param  begin_week  begin week for this restriction.
   */
  void set_begin_week(const uint8_t begin_week) {
    begin_week_ = begin_week;
  }

  /**
   * Set the begin hours for the restriction.
   * @param  begin_hrs  begin hours for this restriction.
   */
  void set_begin_hrs(const uint8_t begin_hrs) {
    begin_hrs_ = begin_hrs;
  }

  /**
   * Set the type for the restriction
   * @param  type   the type of date time restriction.
   *                YMD = 0 or nth dow = 1
   */
  void set_dt_type(const bool type) {
    dt_type_ = type;
  }

  /**
   * Set the end day or dow for the restriction.
   * @param  end_day_dow  end day or dow for this restriction.
   */
  void set_end_day_dow(const uint8_t end_day_dow) {
    end_day_dow_ = end_day_dow;
  }

  /**
   * Set the end month for the restriction.
   * @param  end_month  end month for this restriction.
   */
  void set_end_month(const uint8_t end_month) {
    end_month_ = end_month;
  }

  /**
   * set the end week for the restriction.
   * @param  end_week  end week for this restriction.
   */
  void set_end_week(const uint8_t end_week) {
    end_week_ = end_week;
  }

  /**
   * Set the end hours for the restriction.
   * @param  end_hrs  end hours for this restriction.
   */
  void set_end_hrs(const uint8_t end_hrs) {
    end_hrs_ = end_hrs;
  }

  /**
   * Set the dow mask.  indicates days of week to apply the restriction
   * @param  dow  day of week - This is a mask with first day of week being sunday
   *                            e.g., Mo-Th = 30 = 0011110.
   *                            Sunday is least significant bit.
   */
  void set_dow(const uint8_t dow) {
    dow_ = dow;
  }

  /**
   * Set the begin minutes for the restriction.
   * @param  begin_mins  begin minutes for this restriction.
   */
  void set_begin_mins(const uint8_t begin_mins) {
    begin_mins_ = begin_mins;
  }

  /**
   * Set the end minutes for the restriction.
   * @param  end_mins  end minutes for this restriction.
   */
  void set_end_mins(const uint8_t end_mins) {
    end_mins_ = end_mins;
  }

  /**
   * Set the probability(percentage) for the restriction.  Range for the probability is 0 to 100.
   * @param  probability  Probability(percentage) for this restriction.
   */
  void set_probability(const uint8_t probability) {
    probability_ = probability;
  }

  /**
   * overloaded == operator - used to ensure no dups in tiles.
   * @param  other  ComplexRestrictionBuilder to compare to.
   * @return  Returns true or false if equal or not.
   *
   */
  bool operator==(const ComplexRestrictionBuilder& other) const;

  /**
   * This function makes sense for ComplexRestriction but it is likely unintended
   * that the builder is called with this method. Therefore throw here
   */
  template <typename Callback> void WalkVias(Callback /*callback*/) const {
    throw std::logic_error("You probably didn't intend to walk vias on builder");
  }

protected:
  /**
   * Set the number of vias.
   * @param  count Number of vias
   */
  void set_via_count(const uint8_t count) {
    via_count_ = (count > kMaxViasPerRestriction) ? kMaxViasPerRestriction : count;
  }

  // via list
  std::vector<GraphId> via_list_;

  friend std::ostream& operator<<(std::ostream& os, const ComplexRestrictionBuilder& crb);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_COMPLEXRESTRICTIONBUILDER_H_
