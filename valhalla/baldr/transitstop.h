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
  /**
   * Get the internal stop Id.
   * @return  Returns the internal stop Id.
   */
  uint32_t stopid() const;

  /**
   * Get the TransitLand one stop Id for the stop.
   * @return  Returns the TransitLand one stop Id.
   */
  const char* tl_stopid() const;

  /**
   * Get the text/name index for the stop name.
   * @return  Returns the name index in the text/name list.
   */
  uint32_t name_index() const;

  /**
   * Get the text/name index for the stop description.
   * @return  Returns the description index in the text/name list.
   */
  uint32_t desc_index() const;

  /**
   * Get the internal stop Id of the parent stop.
   * @return  Returns the internal Id of the parent stop, 0 if there is
   *          no parent.
   */
  uint32_t parent_stopid() const;

  /**
   * Get the fare zone Id.
   * @return  Returns the fare zone Id (0 if none).
   */
  uint32_t fare_zoneid() const;

 protected:
  // Internal stop Id. Used to lookup/index stops.
  uint32_t stopid_;

  // TransitLand one stop Id.
  char tl_stopid_[kOneStopIdSize];

  // Stop name offset in the text/name list.
  uint32_t name_index_;

  // Stop description offset in the text/name list.
  uint32_t desc_index_;

  // Stop Id of the parent station.
  uint32_t parent_stopid_;

  // Zone Id for fares
  uint32_t fare_zoneid_;
};

}
}

#endif  // VALHALLA_BALDR_TRANSITSTOP_H_
