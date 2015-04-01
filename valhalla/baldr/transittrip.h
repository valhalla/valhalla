#ifndef VALHALLA_BALDR_TRANSITTRIP_H_
#define VALHALLA_BALDR_TRANSITTRIP_H_

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Information held for each transit trip. This is information not required
 * during path generation. Such information is held within the transit
 * schedules and graph edges.
 */
class TransitTrip {
 public:
  /**
   * Get the internal route Id.
   * @return  Returns the internal stop Id.
   */
  uint32_t tripid() const;

  /**
   * Get the internal route Id.
   * @return  Returns the internal stop Id.
   */
  uint32_t routeid() const;

  /**
   * Get the TransitLand one-stop Id for this trip.
   * @return  Returns the TransitLand one-stop Id.
   */
  const char* tl_tripid() const;

  /**
   * Get the text/name index for the short trip name.
   * @return  Returns the short name index in the text/name list.
   */
  uint32_t short_name_index() const;

  /**
   * Get the text/name index for the trip headsign.
   * @return  Returns the headsign index in the text/name list.
   */
  uint32_t headsign_index() const;

 protected:
  // Trip Id (internal) - for trip lookup
  uint32_t tripid_;

  // Route Id (internal)
  uint32_t routeid_;

  // TransitLand one stop Id for this trip.
  char tl_tripid_[kOneStopIdSize];

  // Short route name offset in the text/name list.
  uint32_t short_name_index_;

  // Headsign index into the text/name list. TODO - do we need this
  // or can we just pass this into each departure?
  uint32_t headsign_index_;
};

}
}

#endif  // VALHALLA_BALDR_TRANSITTRIP_H_
