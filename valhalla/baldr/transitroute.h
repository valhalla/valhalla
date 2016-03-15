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
  TransitRoute(const uint32_t route_type, const uint32_t one_stop_offset,
               const uint32_t op_by_onestop_id_offset, const uint32_t op_by_name_offset,
               const uint32_t op_by_website_offset, const uint32_t route_color,
               const uint32_t route_text_color, const uint32_t short_name_offset,
               const uint32_t long_name_offset, const uint32_t desc_offset);

  /**
   * Get the route type.
   * @return  Returns the route type.
   */
  uint32_t route_type() const;

  /**
   * Get the TransitLand one stop Id offset for this route.
   * @return  Returns the TransitLand one-stop Id offset.
   */
  uint32_t one_stop_offset() const;

  /**
   * Get the TransitLand operator one stop Id offset for this route.
   * @return  Returns the TransitLand operator one-stop Id offset.
   */
  uint32_t op_by_onestop_id_offset() const;

  /**
   * Get the TransitLand operator name offset for this route.
   * @return  Returns the TransitLand operator name offset.
   */
  uint32_t op_by_name_offset() const;

  /**
   * Get the TransitLand operator website offset for this route.
   * @return  Returns the TransitLand operator website offset.
   */
  uint32_t op_by_website_offset() const;

  /**
   * Get the route color route.
   * @return  Returns the route color.
   */
  uint32_t route_color() const;

  /**
   * Get the route text color route.
   * @return  Returns the route text color.
   */
  uint32_t route_text_color() const;

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
  uint32_t route_type_;       // Internal route type
  uint32_t route_color_;      // Route color
  uint32_t route_text_color_; // Route text color

  // Offsets in the text/name list
  uint32_t one_stop_offset_;          // TransitLand onestop Id for this route.
  uint32_t op_by_onestop_id_offset_;  // TransitLand operated by onestop id.
  uint32_t op_by_name_offset_;        // TransitLand operated by name.
  uint32_t op_by_website_offset_;     // TransitLand operated by website.
  uint32_t short_name_offset_;        // Short route name.
  uint32_t long_name_offset_;         // Long route name.
  uint32_t desc_offset_;              // Route description.
};

}
}

#endif  // VALHALLA_BALDR_TRANSITROUTE_H_
