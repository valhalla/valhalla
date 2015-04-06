#include "baldr/transitroute.h"

namespace valhalla {
namespace baldr {

// Get the internal route Id.
uint32_t TransitRoute::routeid() const {
  return routeid_;
}

/**
* Get the internal agency Id for this route.
* @return  Returns the internal agency Id.
*/
uint32_t TransitRoute::agencyid() const {
  return agencyid_;
}

/**
* Get the TransitLand one stop Id for this route.
* @return  Returns the TransitLand one-stop Id.
*/
const char* TransitRoute::tl_routeid() const {
  return tl_routeid_;
}

/**
* Get the text/name index for the short route name.
* @return  Returns the short name index in the text/name list.
*/
uint32_t TransitRoute::short_name_index() const {
  return short_name_index_;
}

/**
* Get the text/name index for the long route name.
* @return  Returns the short name index in the text/name list.
*/
uint32_t TransitRoute::long_name_index() const {
  return long_name_index_;
}

/**
* Get the text/name index for the route description.
* @return  Returns the description index in the text/name list.
*/
uint32_t TransitRoute::desc_index() const {
  return desc_index_;
}

}
}
