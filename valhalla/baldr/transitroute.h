#ifndef VALHALLA_BALDR_TRANSITROUTE_H_
#define VALHALLA_BALDR_TRANSITROUTE_H_

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

/**
 * Information held for each transit route. This is information not required
 * during path generation. Such information is held within the transit
 * schedules and graph edges.
 */
class TransitRoute {
 public:
  // Constructor with arguments
  TransitRoute(const uint32_t routeid,const char* tl_routeid,
               const uint32_t short_name_offset, const uint32_t long_name_offset,
               const uint32_t desc_offset);

  /**
   * Get the internal route Id.
   * @return  Returns the internal stop Id.
   */
  uint32_t routeid() const;

  /**
   * Get the TransitLand one stop Id for this route.
   * @return  Returns the TransitLand one-stop Id.
   */
  const char* tl_routeid() const;

  /**
   * Get the text/name offset for the short route name.
   * @return  Returns the short name offset in the text/name list.
   */
  uint32_t short_name_offset() const;

  /**
   * Get the text/name offset for the long route name.
   * @return  Returns the short name offset in the text/name list.
   */
  uint32_t long_name_offset() const;

  /**
   * Get the text/name offset for the route description.
   * @return  Returns the description offset in the text/name list.
   */
  uint32_t desc_offset() const;

  /**
   * operator < - for sorting. Sort by route Id.
   * @param  other  Other transit route to compare to.
   * @return  Returns true if route Id < other route Id.
   */
  bool operator < (const TransitRoute& other) const;

 protected:
  // Internal route Id. Used to lookup/index routes.
  uint32_t routeid_;

  // TransitLand one stop Id for this route.
  char tl_routeid_[kOneStopIdSize];

  // Short route name offset in the text/name list.
  uint32_t short_name_offset_;

  // Long route name offset in the text/name list.
  uint32_t long_name_offset_;

  // Stop description offset in the text/name list.
  uint32_t desc_offset_;
};

}
}

#endif  // VALHALLA_BALDR_TRANSITROUTE_H_
