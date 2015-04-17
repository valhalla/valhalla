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
  // Constructor with arguments
  TransitTrip(const uint32_t tripid,
              const uint32_t routeid, const char* tl_tripid,
              const uint32_t short_name_offset,
              const uint32_t headsign_offset);

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
   * Get the text/name offset for the short trip name.
   * @return  Returns the offset to the short name in the text/name list.
   */
  uint32_t short_name_offset() const;

  /**
   * Get the text/name offset for the trip headsign.
   * @return  Returns the offset for the headsign in the text/name list.
   */
  uint32_t headsign_offset() const;

  /**
   * operator < - for sorting. Sort by trip Id.
   * @param  other  Other transit trip to compare to.
   * @return  Returns true if trip Id < other trip Id.
   */
  bool operator < (const TransitTrip& other) const;

 protected:
  // Trip Id (internal) - for trip lookup
  uint32_t tripid_;

  // Route Id (internal)
  uint32_t routeid_;

  // TransitLand one stop Id for this trip.
  char tl_tripid_[kOneStopIdSize];

  // Short route name offset in the text/name list.
  uint32_t short_name_offset_;

  // Headsign offset into the text/name list. TODO - do we need this
  // or can we just pass this into each departure?
  uint32_t headsign_offset_;
};

}
}

#endif  // VALHALLA_BALDR_TRANSITTRIP_H_
