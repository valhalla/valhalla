#ifndef VALHALLA_BALDR_TRANSITROUTE_H_
#define VALHALLA_BALDR_TRANSITROUTE_H_

#include <cstdint>
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
  TransitRoute(const TransitType route_type,
               const uint32_t one_stop_offset,
               const uint32_t op_by_onestop_id_offset,
               const uint32_t op_by_name_offset,
               const uint32_t op_by_website_offset,
               const uint32_t route_color,
               const uint32_t route_text_color,
               const uint32_t short_name_offset,
               const uint32_t long_name_offset,
               const uint32_t desc_offset);

  /**
   * Get the route type.
   * @return  Returns the route type.
   */
  TransitType route_type() const {
    return static_cast<TransitType>(route_type_);
  }

  /**
   * Get the TransitLand one stop Id offset for this route.
   * @return  Returns the TransitLand one-stop Id offset.
   */
  uint32_t one_stop_offset() const {
    return one_stop_offset_;
  }

  /**
   * Get the TransitLand operator one stop Id offset for this route.
   * @return  Returns the TransitLand operator one-stop Id offset.
   */
  uint32_t op_by_onestop_id_offset() const {
    return op_by_onestop_id_offset_;
  }

  /**
   * Get the TransitLand operator name offset for this route.
   * @return  Returns the TransitLand operator name offset.
   */
  uint32_t op_by_name_offset() const {
    return op_by_name_offset_;
  }

  /**
   * Get the TransitLand operator website offset for this route.
   * @return  Returns the TransitLand operator website offset.
   */
  uint32_t op_by_website_offset() const {
    return op_by_website_offset_;
  }

  /**
   * Get the route color route.
   * @return  Returns the route color.
   */
  uint32_t route_color() const {
    return route_color_;
  }

  /**
   * Get the route text color route.
   * @return  Returns the route text color.
   */
  uint32_t route_text_color() const {
    return route_text_color_;
  }

  /**
   * Get the text/name offset for the short route name.
   * @return  Returns the short name offset in the text/name list.
   */
  uint32_t short_name_offset() const {
    return short_name_offset_;
  }

  /**
   * Get the text/name offset for the long route name.
   * @return  Returns the short name offset in the text/name list.
   */
  uint32_t long_name_offset() const {
    return long_name_offset_;
  }

  /**
   * Get the text/name offset for the route description.
   * @return  Returns the description offset in the text/name list.
   */
  uint32_t desc_offset() const {
    return desc_offset_;
  }

  /**
   * operator < - for sorting. Sort by route Id.
   * @param  other  Other transit route to compare to.
   * @return  Returns true if route Id < other route Id.
   */
  bool operator<(const TransitRoute& other) const;

protected:
  uint32_t route_color_;      // Route color
  uint32_t route_text_color_; // Route text color

  // Offsets in the text/name list
  uint64_t route_type_ : 8;       // Internal route type
  uint64_t one_stop_offset_ : 24; // TransitLand onestop Id for this route.
  uint64_t spare1_ : 32;

  uint64_t op_by_onestop_id_offset_ : 24; // TransitLand operated by onestop id.
  uint64_t op_by_name_offset_ : 24;       // TransitLand operated by name.
  uint64_t spare2_ : 16;

  uint64_t op_by_website_offset_ : 24; // TransitLand operated by website.
  uint64_t short_name_offset_ : 24;    // Short route name.
  uint64_t spare3_ : 16;

  uint64_t long_name_offset_ : 24; // Long route name.
  uint64_t desc_offset_ : 24;      // Route description.
  uint64_t spare4_ : 16;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_TRANSITROUTE_H_
