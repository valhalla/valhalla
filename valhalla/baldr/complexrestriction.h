#ifndef VALHALLA_BALDR_COMPLEXRESTRICTION_H_
#define VALHALLA_BALDR_COMPLEXRESTRICTION_H_

#include <cstdint>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/json.h>
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace baldr {

constexpr size_t kMaxViasPerRestriction = 31;

enum class WalkingVia {
  KeepWalking,
  StopWalking,
};

/**
 * Information held for each complex access restriction. A complex restriction
 * is a restriction that either:
 *    1) Has via ways (that is not a simple edge to edge restriction),
 *    2) Applies to specific travel modes (not all driving modes),
 *    3) Has specific time periods when the restriction is in effect.
 * This class forms the fixed size portion of the complex restriction.
 * A list of GraphIds follows immediately after the structure.
 */
class ComplexRestriction {
public:
  ComplexRestriction()
      : from_graphid_(kInvalidGraphId), has_dt_(0), to_graphid_(kInvalidGraphId), type_(0), modes_(0),
        via_count_(0) {
  }

  /**
   * Get the restriction's from graph id
   * @return  Returns the from graph id
   */
  GraphId from_graphid() const {
    return GraphId(from_graphid_);
  }

  /**
   * Get the restriction's to graph id
   * @return  Returns the to graph id
   */
  GraphId to_graphid() const {
    return GraphId(to_graphid_);
  }

  /**
   * Get the number of vias.
   * @return  Returns the number of vias
   */
  uint8_t via_count() const {
    return via_count_;
  }

  /**
   * Get the restriction type
   * @return  Returns the restriction type
   */
  RestrictionType type() const {
    return static_cast<RestrictionType>(type_);
  }

  /**
   * Get the modes impacted by the restriction.
   * @return  Returns the access modes
   */
  uint16_t modes() const {
    return modes_;
  }

  /**
   * Get the date time flag for the restriction
   * @return  Returns bool indicating if there is date time info
   *          for this restriction.
   */
  bool has_dt() const {
    return has_dt_;
  }

  /**
   * Get the begin day or dow for the restriction.
   * @return  Returns the begin day or dow for this restriction.
   */
  uint8_t begin_day_dow() const {
    return begin_day_dow_;
  }

  /**
   * Get the begin month for the restriction.
   * @return  Returns the begin month for this restriction.
   */
  uint8_t begin_month() const {
    return begin_month_;
  }

  /**
   * Get the begin week for the restriction.
   * @return  Returns the begin week for this restriction.
   */
  uint8_t begin_week() const {
    return begin_week_;
  }

  /**
   * Get the begin hours for the restriction.
   * @return  Returns the begin hours for this restriction.
   */
  uint8_t begin_hrs() const {
    return begin_hrs_;
  }

  /**
   * Get the type for the restriction
   * @return  Returns the the type of date time restriction.
   *          YMD = 0 or nth dow = 1
   */
  bool dt_type() const {
    return dt_type_;
  }

  /**
   * Get the end day or dow for the restriction.
   * @return  Returns the end day or dow for this restriction.
   */
  uint8_t end_day_dow() const {
    return end_day_dow_;
  }

  /**
   * Get the end month for the restriction.
   * @return  Returns the end month for this restriction.
   */
  uint8_t end_month() const {
    return end_month_;
  }

  /**
   * Get the end week for the restriction.
   * @return  Returns the end week for this restriction.
   */
  uint8_t end_week() const {
    return end_week_;
  }

  /**
   * Get the end hours for the restriction.
   * @return  Returns the end hours for this restriction.
   */
  uint8_t end_hrs() const {
    return end_hrs_;
  }

  /**
   * Get the dow mask.  indicates days of week to apply the restriction
   * @return  Returns the day of week - This is a mask (e.g., Mo-Fr = 62).
   */
  uint8_t dow() const {
    return dow_;
  }

  /**
   * Get the begin minutes for the restriction.
   * @return  Returns the begin minutes for this restriction.
   */
  uint8_t begin_mins() const {
    return begin_mins_;
  }

  /**
   * Get the end minutes for the restriction.
   * @return  Returns the end minutes for this restriction.
   */
  uint8_t end_mins() const {
    return end_mins_;
  }

  /**
   * Get the probability(percentage) for the restriction.  Range for the probability is 0 to 100
   * @return  Returns the probability(percentage) for this restriction.
   */
  uint8_t probability() const {
    return probability_;
  }

  /**
   * Get the size of this complex restriction. Includes the fixed size
   * structure plus the via edge Id list that immediately follows.
   * @return  Returns the size in bytes of this object.
   */
  std::size_t SizeOf() const {
    return (sizeof(ComplexRestriction)) + (via_count_ * sizeof(GraphId));
  }

  /**
   * Walks the vias of the restriction and calls `callback`
   * Return false from `callback` if done walking early
   */
  template <typename Callback> void WalkVias(Callback callback) const {
    if (via_count() > 0) {
      const baldr::GraphId* via = reinterpret_cast<const baldr::GraphId*>(this + 1);
      for (uint32_t i = 0; i < via_count(); i++, via++) {
        if (callback(via) == WalkingVia::StopWalking) {
          break;
        }
      }
    }
  }

protected:
  // From graph Id plus begin date time information (if applicable)
  uint64_t from_graphid_ : 46; // From Graph Id
  uint64_t has_dt_ : 1;        // bit indicating if we have dt time information
  uint64_t begin_day_dow_ : 5; // begin day or dow enum i.e. 1st Sunday
  uint64_t begin_month_ : 4;   // begin month
  uint64_t begin_week_ : 3;    // which begin week does this start.  i.e. 1st week in Oct
  uint64_t begin_hrs_ : 5;     // begin hours

  // To graph Id plus end date time information (if applicable)
  uint64_t to_graphid_ : 46; // To graph Id
  uint64_t dt_type_ : 1;     // type of date time restriction: YMD = 0 or nth dow = 1
  uint64_t end_day_dow_ : 5; // end day or dow enum i.e. last Sunday
  uint64_t end_month_ : 4;   // end month
  uint64_t end_week_ : 3;    // which end week does this end.  i.e. last week in Oct
  uint64_t end_hrs_ : 5;     // end hours

  // Restriction data
  uint64_t type_ : 4;        // Restriction type
  uint64_t modes_ : 12;      // Mode(s) this access restriction applies to
  uint64_t via_count_ : 5;   // size of via list.
  uint64_t dow_ : 7;         // day of week for this restriction
  uint64_t begin_mins_ : 6;  // begin minutes
  uint64_t end_mins_ : 6;    // end minutes
  uint64_t probability_ : 7; // used for probable restrictions.
  uint64_t spare_ : 17;

  // List of vias follows the structure immediately on disk
  // TODO - perhaps use spare to store offset to a separate list?
  // TODO - Maybe but need to consider the fact that we may add more date time data.
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_COMPLEXRESTRICTION_H_
