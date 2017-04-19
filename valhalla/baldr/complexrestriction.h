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
   * Get the restriction's from id
   * @return  Returns the from id
   */
  GraphId from_id() const;

  /**
   * Get the restriction's to id
   * @return  Returns the to id
   */
  GraphId to_id() const;

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
    uint64_t spare_         : 43; // For time/date information.
  };

 protected:

  GraphId from_id_;              // from edge id
  GraphId to_id_;                // to edge id

  // Where we keep most of the static data
  PackedRestriction* restriction_;

  // List of vias
  GraphId* via_list_;             // via edge ids

};

}
}

#endif  // VALHALLA_BALDR_COMPLEXRESTRICTION_H_
