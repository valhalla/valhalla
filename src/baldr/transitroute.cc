#include <string.h>
#include "baldr/transitroute.h"

namespace valhalla {
namespace baldr {

// Constructor with arguments
TransitRoute:: TransitRoute(const uint32_t routeid, uint32_t one_stop_offset,
                            const uint32_t op_by_onestop_id_offset, const uint32_t op_by_name_offset,
                            const uint32_t route_color, const uint32_t route_text_color,
                            const uint32_t short_name_offset,
                            const uint32_t long_name_offset,
                            const uint32_t desc_offset)
    : routeid_(routeid),
      one_stop_offset_(one_stop_offset),
      op_by_onestop_id_offset_(op_by_onestop_id_offset),
      op_by_name_offset_(op_by_name_offset),
      route_color_(route_color),
      route_text_color_(route_text_color),
      short_name_offset_(short_name_offset),
      long_name_offset_(long_name_offset),
      desc_offset_(desc_offset) {}

// Get the internal route Id.
uint32_t TransitRoute::routeid() const {
  return routeid_;
}

// Get the TransitLand one stop Id offset for this route.
uint32_t TransitRoute::one_stop_offset() const {
  return one_stop_offset_;
}

// Get the TransitLand operator one stop Id offset for this route.
uint32_t TransitRoute::op_by_onestop_id_offset() const {
  return op_by_onestop_id_offset_;
}

// Get the TransitLand operator name offset for this route.
uint32_t TransitRoute::op_by_name_offset() const {
  return op_by_name_offset_;
}

// Get the route color route.
uint32_t TransitRoute::route_color() const {
  return route_color_;
}

// Get the route text color route.
uint32_t TransitRoute::route_text_color() const {
  return route_text_color_;
}

// Get the text/name offset for the short route name.
uint32_t TransitRoute::short_name_offset() const {
  return short_name_offset_;
}

// Get the text/name offset for the long route name.
uint32_t TransitRoute::long_name_offset() const {
  return long_name_offset_;
}

// Get the text/name offset for the route description.
uint32_t TransitRoute::desc_offset() const {
  return desc_offset_;
}

// operator < - for sorting. Sort by route Id.
bool TransitRoute::operator < (const TransitRoute& other) const {
  return routeid() < other.routeid();
}

}
}
