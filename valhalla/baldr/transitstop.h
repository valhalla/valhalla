#ifndef VALHALLA_BALDR_TRANSITSTOP_H_
#define VALHALLA_BALDR_TRANSITSTOP_H_

#include <cstdint>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Information held for each transit stop. This is information not required
 * during path generation. Such information is held within NodeInfo (lat,lng,
 * type, etc.).
 */
class TransitStop {
 public:
  // Constructor with arguments
  TransitStop(const uint32_t one_stop_offset,
              const uint32_t name_offset,
              const bool generated,
              const uint32_t traversability);

  /**
   * Get the TransitLand one stop Id offset for the stop.
   * @return  Returns the TransitLand one stop Id offset.
   */
  uint32_t one_stop_offset() const;

  /**
   * Get the text/name offset for the stop name.
   * @return  Returns the name offset in the text/name list.
   */
  uint32_t name_offset() const;

  /**
   * Get the generated flag that indicates if
   * the stop has been generated or exists in
   * real world
   * @return  Returns the generated flag.
   */
  bool generated() const;

  /**
   * Get the traversability indicates if
   * the egress can be entered, exited, or both
   * in the real world.
   * @return  Returns the traversability.
   */
  Traversability traversability() const;

 protected:
  uint64_t one_stop_offset_ : 24;  // TransitLand one stop Id offset.
  uint64_t name_offset_     : 24;  // Stop name offset in the text/name list.
  uint64_t generated_       : 1;
  uint64_t traversability_  : 2;
  uint64_t spare_           : 13;
  //size of tests

};

}
}

#endif  // VALHALLA_BALDR_TRANSITSTOP_H_
