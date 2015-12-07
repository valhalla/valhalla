#ifndef VALHALLA_BALDR_TRANSITSTOP_H_
#define VALHALLA_BALDR_TRANSITSTOP_H_

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
              const uint32_t name_offset);

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

 protected:
  uint32_t one_stop_offset_;  // TransitLand one stop Id offset.
  uint32_t name_offset_;      // Stop name offset in the text/name list.
};

}
}

#endif  // VALHALLA_BALDR_TRANSITSTOP_H_
